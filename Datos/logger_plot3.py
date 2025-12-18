import os
import re
import csv
import time
import math
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

# GPS
GPS_BAUDRATE = 9600
GPS_DETECT_TIMEOUT_S = 4.0
GPS_STARTUP_SLEEP_S = 1.0

OUTPUT_DIR = "/home/adrian/Giroscopio_PCB_EMT/Resultados"
BASE_NAME = "prueba_vibracionesJueves18"

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


def is_candidate_port(dev: str) -> bool:
    return dev.startswith("/dev/ttyACM") or dev.startswith("/dev/ttyUSB")


# ================= GPS: utilidades =================

def haversine_m(lat1, lon1, lat2, lon2) -> float:
    """Distancia en metros entre dos coords en grados decimales."""
    R = 6371000.0
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2.0) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dl / 2.0) ** 2
    c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))
    return R * c


def _nmea_to_decimal(coord_str: str, hemi: str, is_lat: bool):
    """
    coord_str: ddmm.mmmm (lat) o dddmm.mmmm (lon)
    hemi: N/S/E/W
    """
    if not coord_str or not hemi:
        return None
    try:
        if is_lat:
            deg = int(coord_str[0:2])
            minutes = float(coord_str[2:])
        else:
            deg = int(coord_str[0:3])
            minutes = float(coord_str[3:])
        dec = deg + minutes / 60.0
        if hemi in ("S", "W"):
            dec = -dec
        return dec
    except Exception:
        return None


def parse_nmea_sentence(line: str):
    """
    Parse básico de NMEA para RMC/GGA. Devuelve dict con campos detectados.
    No valida checksum estrictamente (tolerante).
    """
    line = line.strip()
    if not line.startswith("$"):
        return None

    # Quitar checksum si existe
    if "*" in line:
        payload, _chk = line.split("*", 1)
    else:
        payload = line

    parts = payload.split(",")
    if not parts:
        return None

    msg = parts[0]  # $GPRMC, $GNRMC, $GPGGA, $GNGGA, etc.

    out = {"type": None}

    # RMC
    if msg.endswith("RMC") and len(parts) >= 7:
        status = parts[2] if len(parts) > 2 else ""
        lat = _nmea_to_decimal(parts[3], parts[4], is_lat=True) if len(parts) > 4 else None
        lon = _nmea_to_decimal(parts[5], parts[6], is_lat=False) if len(parts) > 6 else None

        out["type"] = "RMC"
        out["status"] = status  # A=active, V=void
        out["lat"] = lat
        out["lon"] = lon
        return out

    # GGA
    if msg.endswith("GGA") and len(parts) >= 10:
        lat = _nmea_to_decimal(parts[2], parts[3], is_lat=True) if len(parts) > 3 else None
        lon = _nmea_to_decimal(parts[4], parts[5], is_lat=False) if len(parts) > 5 else None

        fix_q = None
        sats = None
        hdop = None
        alt = None

        try:
            fix_q = int(parts[6]) if parts[6] else None
        except Exception:
            fix_q = None

        try:
            sats = int(parts[7]) if parts[7] else None
        except Exception:
            sats = None

        try:
            hdop = float(parts[8]) if parts[8] else None
        except Exception:
            hdop = None

        try:
            alt = float(parts[9]) if parts[9] else None
        except Exception:
            alt = None

        out["type"] = "GGA"
        out["lat"] = lat
        out["lon"] = lon
        out["fix_q"] = fix_q
        out["sats"] = sats
        out["hdop"] = hdop
        out["alt_m"] = alt
        return out

    return None


class GPSManager:
    """
    Lee GPS por serial (NMEA), mantiene el último estado y una secuencia 'seq'
    que incrementa cada vez que hay una actualización válida de posición.
    Calcula velocidad (km/h) por Haversine entre actualizaciones válidas.
    """
    def __init__(self, ser: serial.Serial):
        self.ser = ser
        self.port = ser.port

        self._lock = threading.Lock()
        self._stop = threading.Event()

        # Estado GPS
        self.seq = 0
        self.last_update_pc_time = None

        self.lat = None
        self.lon = None
        self.alt_m = None
        self.sats = None
        self.fix = None     # bool
        self.hdop = None

        self.velocidad_kmh = None

        # Para velocidad (punto anterior "emitido")
        self._prev_lat = None
        self._prev_lon = None
        self._prev_time = None

        # Para preferir RMC y evitar doble-emisión con GGA
        self._last_rmc_pc_time = None

        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

        print(f"[GPS] Usando puerto {self.port} @ {GPS_BAUDRATE} baudios")

    def stop(self):
        self._stop.set()
        try:
            self.ser.close()
        except Exception:
            pass
        print("[GPS] Detenido.")

    def get_snapshot(self):
        with self._lock:
            return {
                "seq": self.seq,
                "pc_time": self.last_update_pc_time,
                "lat": self.lat,
                "lon": self.lon,
                "alt_m": self.alt_m,
                "sats": self.sats,
                "fix": self.fix,
                "hdop": self.hdop,
                "velocidad": self.velocidad_kmh,
            }

    def _apply_update(self, update: dict, pc_time: float):
        """
        Fix real:
        - Evitar que una frase duplicada (GGA+RMC seguidas) pise la velocidad.
        - Emitimos 1 update por ciclo:
            * Preferimos RMC (status=A)
            * GGA solo emite si no ha habido RMC reciente.
        - Anti-duplicados basado SOLO en dt (no en distancia mal calculada).
        """
        lat = update.get("lat", None)
        lon = update.get("lon", None)

        with self._lock:
            # Complementarios
            if "alt_m" in update and update["alt_m"] is not None:
                self.alt_m = update["alt_m"]
            if "sats" in update and update["sats"] is not None:
                self.sats = update["sats"]
            if "hdop" in update and update["hdop"] is not None:
                self.hdop = update["hdop"]

            # Fix
            fix_val = self.fix
            if update.get("type") == "RMC":
                status = update.get("status", "")
                if status == "A":
                    fix_val = True
                elif status == "V":
                    fix_val = False
            if update.get("type") == "GGA":
                fix_q = update.get("fix_q", None)
                if fix_q is not None:
                    fix_val = fix_q > 0

            # Guardar lat/lon si vienen
            if lat is not None and lon is not None:
                self.lat = lat
                self.lon = lon
            self.fix = fix_val

            # Requisitos mínimos
            if lat is None or lon is None:
                return
            if fix_val is not True:
                return

            # Decidir si esta frase emite update
            emit = False
            if update.get("type") == "RMC":
                # Solo RMC activa
                if update.get("status", "") == "A":
                    emit = True
                    self._last_rmc_pc_time = pc_time
            elif update.get("type") == "GGA":
                # Solo si NO hemos visto RMC reciente (evita doble update por segundo)
                if self._last_rmc_pc_time is None or (pc_time - self._last_rmc_pc_time) > 1.2:
                    emit = True

            if not emit:
                return

            # Anti-duplicados por tiempo (GGA y RMC pueden venir con dt muy pequeño)
            MIN_DT = 0.35
            if self.last_update_pc_time is not None:
                if (pc_time - self.last_update_pc_time) < MIN_DT:
                    return

            # Calcular velocidad vs punto anterior emitido
            if self._prev_lat is not None and self._prev_lon is not None and self._prev_time is not None:
                dt = pc_time - self._prev_time
                if dt > 0:
                    dist_m = haversine_m(self._prev_lat, self._prev_lon, lat, lon)
                    self.velocidad_kmh = (dist_m / dt) * 3.6
                else:
                    self.velocidad_kmh = None
            else:
                self.velocidad_kmh = None

            self._prev_lat = lat
            self._prev_lon = lon
            self._prev_time = pc_time

            self.last_update_pc_time = pc_time
            self.seq += 1

    def _loop(self):
        try:
            time.sleep(GPS_STARTUP_SLEEP_S)
            try:
                self.ser.reset_input_buffer()
            except Exception:
                pass
        except Exception:
            pass

        while not self._stop.is_set():
            try:
                line_bytes = self.ser.readline()
                if not line_bytes:
                    continue
                line = line_bytes.decode("utf-8", errors="ignore").strip()
                if not line or not line.startswith("$"):
                    continue

                parsed = parse_nmea_sentence(line)
                if not parsed:
                    continue

                pc_time = time.time()
                self._apply_update(parsed, pc_time)

            except Exception:
                continue


# ================= Sesión ESP32 =================

class DeviceSession:
    def __init__(self, ser: serial.Serial, ubicacion: str, gps_manager, stop_all_cb):
        self.ser = ser
        self.port = ser.port
        self.ubicacion = ubicacion
        self.gps_manager = gps_manager
        self.stop_all_cb = stop_all_cb  # parar TODO

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
                "tiempo_s",
                "ax_ms2",
                "ay_ms2",
                "az_ms2",
                "gx_dps",
                "gy_dps",
                "gz_dps",
                "gps_lat",
                "gps_lon",
                "gps_alt_m",
                "gps_sats",
                "gps_fix",
                "gps_hdop",
                "velocidad",  # km/h
            ]
        )
        self.csvfile.flush()

        self.t_buf = deque(maxlen=MAX_POINTS)
        self.ax_buf = deque(maxlen=MAX_POINTS)
        self.ay_buf = deque(maxlen=MAX_POINTS)
        self.az_buf = deque(maxlen=MAX_POINTS)

        self.start_time_s = None
        self.pc_start_time = None

        self.q = queue.Queue(maxsize=QUEUE_MAXSIZE)

        self.stop_event = threading.Event()
        self.stopped_lock = threading.Lock()
        self._stopped = False

        self.last_seen_gps_seq = 0

        self.fig = None
        self.ani = None
        self.gps_text = None
        self._setup_plot()

        self.reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.reader_thread.start()

        print(f"[{self.port}] CSV:  {self.csv_path}")
        print(f"[{self.port}] PNG:  {self.png_path}")

    def _setup_plot(self):
        self.fig, ((ax_all, ax_x), (ax_y, ax_z)) = plt.subplots(
            2, 2, sharex="col", figsize=(10, 6)
        )
        plt.subplots_adjust(bottom=0.12)

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

        self.fig.suptitle(f"ESP32: {self.ubicacion}  |  Puerto: {self.port}")

        try:
            self.fig.canvas.manager.set_window_title(
                f"MPU6050 - {self.ubicacion} - {self.port}"
            )
        except Exception:
            pass

        # Panel GPS
        self.gps_text = self.fig.text(
            0.02, 0.02, "GPS: (no detectado)",
            fontsize=9, va="bottom", ha="left"
        )

        # Si el usuario cierra cualquier ventana -> parar TODO
        def on_close(_event):
            print(f"[{self.port}] Ventana cerrada -> detener todo.")
            self.stop_all_cb()

        self.fig.canvas.mpl_connect("close_event", on_close)

        self.ani = animation.FuncAnimation(self.fig, self.update, interval=100)

    def _reader_loop(self):
        while not self.stop_event.is_set():
            try:
                line_bytes = self.ser.readline()
                if not line_bytes:
                    continue

                line_str = line_bytes.decode("utf-8", errors="ignore").strip()
                if not line_str or line_str.startswith("Ubicacion:"):
                    continue

                parts = line_str.split(",")
                if len(parts) < 7:
                    continue

                try:
                    t_ms, ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw = map(float, parts[:7])
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

        print(f"[{self.port}] Sesión detenida.")

    def _format_gps_text(self):
        if not self.gps_manager:
            return "GPS: (no detectado)"

        s = self.gps_manager.get_snapshot()
        lat, lon = s["lat"], s["lon"]
        alt, sats = s["alt_m"], s["sats"]
        fix, hdop = s["fix"], s["hdop"]
        vel = s["velocidad"]
        pc_t = s["pc_time"]

        age = (time.time() - pc_t) if pc_t is not None else None

        def fmt(x, nd=6):
            if x is None:
                return "-"
            try:
                return f"{x:.{nd}f}"
            except Exception:
                return str(x)

        fix_str = "OK" if fix is True else ("NO" if fix is False else "-")

        return (
            f"GPS:\n"
            f"  lat: {fmt(lat, 6)}\n"
            f"  lon: {fmt(lon, 6)}\n"
            f"  alt(m): {fmt(alt, 1)}\n"
            f"  sats: {sats if sats is not None else '-'}\n"
            f"  fix: {fix_str}\n"
            f"  hdop: {fmt(hdop, 1)}\n"
            f"  velocidad(km/h): {fmt(vel, 2)}\n"
            f"  age(s): {fmt(age, 1)}"
        )

    def update(self, _frame):
        # Actualizar HUD GPS
        try:
            if self.gps_text is not None:
                self.gps_text.set_text(self._format_gps_text())
        except Exception:
            pass

        while True:
            try:
                t_ms, ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw, pc_time = self.q.get_nowait()
            except queue.Empty:
                break

            ax_ms2 = (ax_raw / ACC_SCALE) * G_CONST
            ay_ms2 = (ay_raw / ACC_SCALE) * G_CONST
            az_ms2 = (az_raw / ACC_SCALE) * G_CONST

            gx_dps = gx_raw / GYRO_SCALE
            gy_dps = gy_raw / GYRO_SCALE
            gz_dps = gz_raw / GYRO_SCALE

            if self.start_time_s is None:
                self.start_time_s = t_ms / 1000.0
            t_s = t_ms / 1000.0 - self.start_time_s

            if self.pc_start_time is None:
                self.pc_start_time = pc_time
            tiempo_s = pc_time - self.pc_start_time

            self.t_buf.append(t_s)
            self.ax_buf.append(ax_ms2)
            self.ay_buf.append(ay_ms2)
            self.az_buf.append(az_ms2)

            # GPS solo en la fila que "le toca"
            gps_lat = gps_lon = gps_alt = gps_sats = gps_fix = gps_hdop = velocidad = ""
            if self.gps_manager:
                snap = self.gps_manager.get_snapshot()
                seq = snap["seq"]
                if seq != self.last_seen_gps_seq and snap["lat"] is not None and snap["lon"] is not None:
                    self.last_seen_gps_seq = seq
                    gps_lat = snap["lat"]
                    gps_lon = snap["lon"]
                    gps_alt = snap["alt_m"] if snap["alt_m"] is not None else ""
                    gps_sats = snap["sats"] if snap["sats"] is not None else ""
                    gps_fix = 1 if snap["fix"] is True else (0 if snap["fix"] is False else "")
                    gps_hdop = snap["hdop"] if snap["hdop"] is not None else ""
                    velocidad = snap["velocidad"] if snap["velocidad"] is not None else ""

            self.csv_writer.writerow(
                [
                    self.ubicacion, t_ms, pc_time, tiempo_s,
                    ax_ms2, ay_ms2, az_ms2,
                    gx_dps, gy_dps, gz_dps,
                    gps_lat, gps_lon, gps_alt, gps_sats, gps_fix, gps_hdop,
                    velocidad
                ]
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


# ================= Descubrimiento dispositivos =================

def _detect_gps_on_serial(dev: str) -> bool:
    try:
        ser = serial.Serial(dev, GPS_BAUDRATE, timeout=1)
    except Exception:
        return False

    try:
        time.sleep(GPS_STARTUP_SLEEP_S)
        try:
            ser.reset_input_buffer()
        except Exception:
            pass

        t0 = time.time()
        while time.time() - t0 < GPS_DETECT_TIMEOUT_S:
            b = ser.readline()
            if not b:
                continue
            s = b.decode("utf-8", errors="ignore").strip()
            if s.startswith("$") and len(s) > 6:
                ser.close()
                return True
    except Exception:
        pass

    try:
        ser.close()
    except Exception:
        pass
    return False


def discover_esp32_devices_and_gps():
    esp32_devices = []
    gps_candidates = []

    all_ports = list(list_ports.comports())
    ports = [p for p in all_ports if is_candidate_port(p.device or "")]

    if not ports:
        print("No se han encontrado puertos /dev/ttyACM* ni /dev/ttyUSB*.")
        return esp32_devices, None

    print("Puertos candidatos (filtrados):")
    for p in ports:
        desc = f"{p.device} | {p.description}"
        if p.manufacturer:
            desc += f" | {p.manufacturer}"
        if p.vid and p.pid:
            desc += f" | VID:PID={p.vid:04x}:{p.pid:04x}"
        print(" -", desc)

    print("\nDetectando ESP32 (Ubicacion:) y GPS (NMEA)...\n")

    for p in ports:
        dev = p.device

        # 1) ESP32
        ser_esp = None
        try:
            ser_esp = serial.Serial(dev, BAUDRATE, timeout=1)
            time.sleep(STARTUP_SLEEP_S)
            try:
                ser_esp.reset_input_buffer()
            except Exception:
                pass

            ubicacion = detectar_ubicacion(ser_esp, timeout_s=DETECT_TIMEOUT_S)
            if ubicacion != "Desconocida":
                esp32_devices.append((ser_esp, ubicacion))
                continue
        except Exception:
            pass

        try:
            if ser_esp is not None and ser_esp.is_open:
                ser_esp.close()
        except Exception:
            pass

        # 2) GPS
        try:
            if _detect_gps_on_serial(dev):
                gps_candidates.append(dev)
        except Exception:
            pass

    gps_serial = None
    if gps_candidates:
        if len(gps_candidates) > 1:
            print("[GPS] Aviso: varios candidatos:", gps_candidates)
            print("[GPS] Se usará el primero:", gps_candidates[0])

        try:
            gps_serial = serial.Serial(gps_candidates[0], GPS_BAUDRATE, timeout=1)
        except Exception as e:
            print(f"[GPS] No se pudo abrir el GPS en {gps_candidates[0]}: {e}")
            gps_serial = None
    else:
        print("[GPS] No se detectó ningún GPS (NMEA).")

    return esp32_devices, gps_serial


def create_control_window(stop_all_cb):
    fig = plt.figure(figsize=(4.0, 1.6))
    try:
        fig.canvas.manager.set_window_title("Control")
    except Exception:
        pass
    fig.suptitle("Control general", fontsize=12)

    ax_button = fig.add_axes([0.2, 0.25, 0.6, 0.5])
    btn = Button(ax_button, "Detener TODO", hovercolor="0.8")

    def on_stop(_event):
        print("[CONTROL] Detener TODO pulsado.")
        stop_all_cb()

    btn.on_clicked(on_stop)

    def on_close(_event):
        print("[CONTROL] Ventana control cerrada -> detener todo.")
        stop_all_cb()

    fig.canvas.mpl_connect("close_event", on_close)

    # 🔒 MUY IMPORTANTE: mantener referencias para que el botón sea clicable
    fig._control_ax = ax_button
    fig._control_btn = btn

    return fig



def main():
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    found, gps_ser = discover_esp32_devices_and_gps()
    if not found:
        print("No se detectó ninguna ESP32 del proyecto (no apareció 'Ubicacion:').")
        if gps_ser:
            try:
                gps_ser.close()
            except Exception:
                pass
        return

    gps_manager = GPSManager(gps_ser) if gps_ser else None

    sessions = []
    stop_lock = threading.Lock()
    stopped = {"done": False}

    def stop_all():
        with stop_lock:
            if stopped["done"]:
                return
            stopped["done"] = True

        for s in sessions:
            try:
                s.stop()
            except Exception:
                pass

        if gps_manager:
            try:
                gps_manager.stop()
            except Exception:
                pass

        try:
            plt.close("all")
        except Exception:
            pass

    # Ventana de control con botón único
    _control_fig = create_control_window(stop_all)

    # Crear sesiones
    for ser, ubicacion in found:
        try:
            sessions.append(DeviceSession(ser, ubicacion, gps_manager, stop_all))
        except Exception as e:
            print(f"No se pudo crear sesión para {ser.port}: {e}")
            try:
                ser.close()
            except Exception:
                pass

    if not sessions:
        print("No se pudo iniciar ninguna sesión.")
        stop_all()
        return

    try:
        plt.show()
    finally:
        stop_all()


if __name__ == "__main__":
    main()
