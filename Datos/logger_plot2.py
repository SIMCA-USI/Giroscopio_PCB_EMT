import os
import re
import csv
import time
import queue
import threading
from collections import deque

import serial
from serial.tools import list_ports

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Button

# ========= CONFIGURACIÓN =========

BAUDRATE = 115200

OUTPUT_DIR = "/home/adrian/Giroscopio_PCB_EMT/Resultados"
BASE_NAME = "prueba_vibraciones1"

WINDOW_SECONDS = 60
SAMPLE_HZ = 100
MAX_POINTS = WINDOW_SECONDS * SAMPLE_HZ

ACC_LIMIT_MS2 = None

ACC_SCALE = 4096.0
G_CONST = 9.80665
GYRO_SCALE = 131.0

DETECT_TIMEOUT_S = 4.0
STARTUP_SLEEP_S = 1.5
QUEUE_MAXSIZE = 5000

# ========= FIN CONFIGURACIÓN =========


def sanitize_filename_component(s: str) -> str:
    s = s.strip()
    if not s:
        return "Desconocida"
    s = s.replace(" ", "_")
    s = re.sub(r"[^A-Za-z0-9_\-\.]", "", s)
    return s or "Desconocida"


def detectar_ubicacion(ser: serial.Serial, timeout_s: float = 5.0) -> str:
    inicio = time.time()
    ubicacion = "Desconocida"

    while time.time() - inicio < timeout_s:
        line_bytes = ser.readline()
        if not line_bytes:
            continue

        try:
            line_str = line_bytes.decode("utf-8", errors="ignore").strip()
        except Exception:
            continue

        if line_str.startswith("Ubicacion:"):
            try:
                pos = line_str.find(":")
                resto = line_str[pos + 1 :].strip()
                ubicacion = resto.split("\t")[0].strip()
                print(f"Ubicación detectada en {ser.port}: '{ubicacion}'")
                break
            except Exception:
                pass

    return ubicacion


class DeviceSession:
    def __init__(self, ser: serial.Serial, ubicacion: str):
        self.ser = ser
        self.port = ser.port
        self.ubicacion = ubicacion

        self.ubicacion_safe = sanitize_filename_component(ubicacion)
        self.port_safe = sanitize_filename_component(os.path.basename(self.port))

        ts = time.strftime("%Y%m%d_%H%M%S")
        self.csv_path = os.path.join(
            OUTPUT_DIR, f"{BASE_NAME}_{self.ubicacion_safe}_{self.port_safe}_{ts}.csv"
        )
        self.png_path = os.path.join(
            OUTPUT_DIR, f"{BASE_NAME}_{self.ubicacion_safe}_{self.port_safe}_{ts}.png"
        )

        self.csvfile = open(self.csv_path, "w", newline="")
        self.csv_writer = csv.writer(self.csvfile)
        self.csv_writer.writerow(
            [
                "ubicacion",
                "t_ms",
                "pc_time_s",
                "ax_ms2",
                "ay_ms2",
                "az_ms2",
                "gx_dps",
                "gy_dps",
                "gz_dps",
            ]
        )
        self.csvfile.flush()

        self.t_buf = deque(maxlen=MAX_POINTS)
        self.ax_buf = deque(maxlen=MAX_POINTS)
        self.ay_buf = deque(maxlen=MAX_POINTS)
        self.az_buf = deque(maxlen=MAX_POINTS)

        self.start_time_s = None

        self.q = queue.Queue(maxsize=QUEUE_MAXSIZE)

        self.stop_event = threading.Event()
        self.stopped_lock = threading.Lock()
        self._stopped = False

        self.fig = None
        self.ani = None
        self._setup_plot()

        self.reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.reader_thread.start()

        print(f"[{self.port}] CSV:  {self.csv_path}")
        print(f"[{self.port}] PNG:  {self.png_path}")

    def _setup_plot(self):
        self.fig, ((ax_all, ax_x), (ax_y, ax_z)) = plt.subplots(
            2, 2, sharex="col", figsize=(10, 6)
        )
        plt.subplots_adjust(bottom=0.18)

        self.ax_all, self.ax_x, self.ax_y, self.ax_z = ax_all, ax_x, ax_y, ax_z

        self.line_all_x, = ax_all.plot([], [], label="ax", color="r")
        self.line_all_y, = ax_all.plot([], [], label="ay", color="g")
        self.line_all_z, = ax_all.plot([], [], label="az", color="b")
        ax_all.set_ylabel("Aceleración [m/s²]")
        ax_all.set_title("Aceleración XYZ")
        ax_all.legend(loc="upper right")

        self.line_x, = ax_x.plot([], [], color="r")
        ax_x.set_title("Eje X")
        ax_x.set_ylabel("ax [m/s²]")

        self.line_y, = ax_y.plot([], [], color="g")
        ax_y.set_title("Eje Y")
        ax_y.set_ylabel("ay [m/s²]")

        self.line_z, = ax_z.plot([], [], color="b")
        ax_z.set_title("Eje Z")
        ax_z.set_ylabel("az [m/s²]")

        ax_z.set_xlabel("Tiempo (s)")
        ax_y.set_xlabel("Tiempo (s)")

        self.fig.suptitle(f"ESP32 ubicada en: {self.ubicacion},\n en el puerto: ({self.port})")

        try:
            self.fig.canvas.manager.set_window_title(
                f"MPU6050 - {self.ubicacion} - {self.port}"
            )
        except Exception:
            pass

        ax_button = plt.axes([0.42, 0.04, 0.16, 0.06])
        self.btn_stop = Button(ax_button, "Detener", hovercolor="0.8")

        def on_stop(_event):
            print(f"[{self.port}] Botón 'Detener' pulsado.")
            self.stop()
            plt.close(self.fig)

        self.btn_stop.on_clicked(on_stop)

        def on_close(_event):
            print(f"[{self.port}] Ventana cerrada.")
            self.stop()

        self.fig.canvas.mpl_connect("close_event", on_close)

        self.ani = animation.FuncAnimation(self.fig, self.update, interval=100)

    def _reader_loop(self):
        while not self.stop_event.is_set():
            try:
                line_bytes = self.ser.readline()
                if not line_bytes:
                    continue

                line_str = line_bytes.decode("utf-8", errors="ignore").strip()
                if not line_str:
                    continue

                if line_str.startswith("Ubicacion:"):
                    continue

                parts = line_str.split(",")
                if len(parts) < 7:
                    continue

                try:
                    t_ms, ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw = map(
                        float, parts[:7]
                    )
                except ValueError:
                    continue

                pc_time = time.time()
                item = (t_ms, ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw, pc_time)

                if self.q.full():
                    try:
                        _ = self.q.get_nowait()
                    except queue.Empty:
                        pass
                self.q.put_nowait(item)

            except Exception:
                continue

    def stop(self):
        with self.stopped_lock:
            if self._stopped:
                return
            self._stopped = True

        self.stop_event.set()

        try:
            if self.fig is not None:
                self.fig.savefig(self.png_path)
                print(f"[{self.port}] Gráfica guardada en {self.png_path}")
        except Exception as e:
            print(f"[{self.port}] No se pudo guardar la gráfica: {e}")

        try:
            self.ser.close()
        except Exception:
            pass

        try:
            self.csvfile.close()
        except Exception:
            pass

        print(f"[{self.port}] Sesión detenida y recursos cerrados.")

    def update(self, _frame):
        while True:
            try:
                t_ms, ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw, pc_time = self.q.get_nowait()
            except queue.Empty:
                break

            ax_g = ax_raw / ACC_SCALE
            ay_g = ay_raw / ACC_SCALE
            az_g = az_raw / ACC_SCALE

            ax_ms2 = ax_g * G_CONST
            ay_ms2 = ay_g * G_CONST
            az_ms2 = az_g * G_CONST

            gx_dps = gx_raw / GYRO_SCALE
            gy_dps = gy_raw / GYRO_SCALE
            gz_dps = gz_raw / GYRO_SCALE

            if self.start_time_s is None:
                self.start_time_s = t_ms / 1000.0
            t_s = t_ms / 1000.0 - self.start_time_s

            self.t_buf.append(t_s)
            self.ax_buf.append(ax_ms2)
            self.ay_buf.append(ay_ms2)
            self.az_buf.append(az_ms2)

            self.csv_writer.writerow(
                [self.ubicacion, t_ms, pc_time, ax_ms2, ay_ms2, az_ms2, gx_dps, gy_dps, gz_dps]
            )
            self.csvfile.flush()

        if not self.t_buf:
            return (
                self.line_all_x, self.line_all_y, self.line_all_z,
                self.line_x, self.line_y, self.line_z
            )

        self.line_all_x.set_data(self.t_buf, self.ax_buf)
        self.line_all_y.set_data(self.t_buf, self.ay_buf)
        self.line_all_z.set_data(self.t_buf, self.az_buf)

        self.line_x.set_data(self.t_buf, self.ax_buf)
        self.line_y.set_data(self.t_buf, self.ay_buf)
        self.line_z.set_data(self.t_buf, self.az_buf)

        t_max = self.t_buf[-1]
        t_min = max(0, t_max - WINDOW_SECONDS)
        for axis in (self.ax_all, self.ax_x, self.ax_y, self.ax_z):
            axis.set_xlim(t_min, t_max)

        if ACC_LIMIT_MS2 is not None:
            ymin, ymax = -ACC_LIMIT_MS2, ACC_LIMIT_MS2
        else:
            all_vals = list(self.ax_buf) + list(self.ay_buf) + list(self.az_buf)
            ymin = min(all_vals)
            ymax = max(all_vals)
            if ymin == ymax:
                ymin -= 1e-3
                ymax += 1e-3
            margen = 0.1 * (ymax - ymin)
            ymin -= margen
            ymax += margen

        for axis in (self.ax_all, self.ax_x, self.ax_y, self.ax_z):
            axis.set_ylim(ymin, ymax)

        return (
            self.line_all_x, self.line_all_y, self.line_all_z,
            self.line_x, self.line_y, self.line_z
        )


def discover_esp32_devices():
    """
    Devuelve lista (ser, ubicacion) SOLO para /dev/ttyACM* y /dev/ttyUSB*
    y solo si emiten 'Ubicacion:'.
    """
    devices = []

    all_ports = list(list_ports.comports())

    # --- FILTRO AQUÍ ---
    ports = []
    for p in all_ports:
        dev = p.device or ""
        if dev.startswith("/dev/ttyACM") or dev.startswith("/dev/ttyUSB"):
            ports.append(p)

    if not ports:
        print("No se han encontrado puertos /dev/ttyACM* ni /dev/ttyUSB*.")
        return devices

    print("Puertos candidatos (filtrados):")
    for p in ports:
        desc = f"{p.device} | {p.description}"
        if p.manufacturer:
            desc += f" | {p.manufacturer}"
        if p.vid and p.pid:
            desc += f" | VID:PID={p.vid:04x}:{p.pid:04x}"
        print(" -", desc)

    print("\nBuscando ESP32 (esperando línea 'Ubicacion:')...\n")

    for p in ports:
        try:
            ser = serial.Serial(p.device, BAUDRATE, timeout=1)
        except Exception:
            continue

        try:
            time.sleep(STARTUP_SLEEP_S)

            try:
                ser.reset_input_buffer()
            except Exception:
                pass

            ubicacion = detectar_ubicacion(ser, timeout_s=DETECT_TIMEOUT_S)
            if ubicacion != "Desconocida":
                devices.append((ser, ubicacion))
            else:
                ser.close()

        except Exception:
            try:
                ser.close()
            except Exception:
                pass

    return devices


def main():
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    found = discover_esp32_devices()
    if not found:
        print("No se detectó ninguna ESP32 del proyecto (no apareció 'Ubicacion:').")
        return

    sessions = []
    for ser, ubicacion in found:
        try:
            sessions.append(DeviceSession(ser, ubicacion))
        except Exception as e:
            print(f"No se pudo crear sesión para {ser.port}: {e}")
            try:
                ser.close()
            except Exception:
                pass

    if not sessions:
        print("No se pudo iniciar ninguna sesión.")
        return

    try:
        plt.show()
    finally:
        for s in sessions:
            s.stop()


if __name__ == "__main__":
    main()
