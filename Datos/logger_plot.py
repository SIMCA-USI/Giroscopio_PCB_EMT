import os
import serial
import csv
import time
from collections import deque

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Button

# ========= CONFIGURACIÓN =========

# Puerto serie de la ESP32
SERIAL_PORT = "/dev/ttyACM0"   # En Windows sería algo tipo "COM3"
BAUDRATE = 115200

# Carpeta donde se guardarán CSV y gráfica
OUTPUT_DIR = "/home/adrian/Giroscopio_PCB_EMT/Resultados"

# Nombre base de los ficheros (sin extensión)
BASE_NAME = "prueba_vibraciones1"

# Ventana de tiempo para la gráfica (segundos)
WINDOW_SECONDS = 60

# Frecuencia de muestras aproximada (Hz) -> para dimensionar el buffer
SAMPLE_HZ = 100
MAX_POINTS = WINDOW_SECONDS * SAMPLE_HZ

# Límite vertical opcional para la aceleración (en m/s²).
# Si es None, se autoescala según los datos.
ACC_LIMIT_MS2 = None

# Conversión de crudo a g (tiene que coincidir con el rango de la MPU6050 en la ESP32)
ACC_SCALE = 4096.0  # LSB/g (±8 g)

# Constante de gravedad
G_CONST = 9.80665    # m/s² por 1 g

# Escala del giroscopio (si quieres guardarlo en dps)
GYRO_SCALE = 131.0   # LSB / (°/s) para ±250 dps

# ========= FIN CONFIGURACIÓN =========


def detectar_ubicacion(ser, timeout_s=5.0):
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
                resto = line_str[pos + 1:].strip()  # "Asiento\tAccel[g]: ..."
                ubicacion = resto.split("\t")[0].strip()
                print(f"Ubicación detectada en serie: '{ubicacion}'")
                break
            except Exception:
                pass

    return ubicacion


def main():
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    csv_path = os.path.join(OUTPUT_DIR, BASE_NAME + ".csv")
    plot_path = os.path.join(OUTPUT_DIR, BASE_NAME + ".png")

    print(f"CSV:   {csv_path}")
    print(f"Plot:  {plot_path}")

    # Abrir puerto serie
    print(f"Abrriendo puerto serie {SERIAL_PORT} a {BAUDRATE} baudios...")
    try:
        ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
    except Exception as e:
        print(f"Error al abrir el puerto serie: {e}")
        return

    time.sleep(2)

    # Detectar ubicación leyendo las primeras líneas
    ubicacion = detectar_ubicacion(ser)
    print(f"Ubicación usada: {ubicacion}")

    # Abrir archivo CSV (sobrescribe si existe)
    try:
        csvfile = open(csv_path, "w", newline="")
    except Exception as e:
        print(f"No se pudo abrir el archivo CSV para escribir: {e}")
        ser.close()
        return

    csv_writer = csv.writer(csvfile)
    # columnas: ubicacion + t_ms + timestamp PC + datos EN m/s² + giroscopio en dps
    csv_writer.writerow([
        "ubicacion", "t_ms", "pc_time_s",
        "ax_ms2", "ay_ms2", "az_ms2",
        "gx_dps", "gy_dps", "gz_dps"
    ])
    print(f"Guardando datos en {csv_path}")

    # Buffers para gráfica (últimos N puntos, valores en m/s²)
    t_buf = deque(maxlen=MAX_POINTS)
    ax_buf = deque(maxlen=MAX_POINTS)
    ay_buf = deque(maxlen=MAX_POINTS)
    az_buf = deque(maxlen=MAX_POINTS)

    # FIGURA CON 4 SUBPLOTS
    fig, ((ax_all, ax_x), (ax_y, ax_z)) = plt.subplots(
        2, 2, sharex="col", figsize=(10, 6)
    )

    # Hueco abajo para el botón
    plt.subplots_adjust(bottom=0.18)

    line_all_x, = ax_all.plot([], [], label="ax", color="r")
    line_all_y, = ax_all.plot([], [], label="ay", color="g")
    line_all_z, = ax_all.plot([], [], label="az", color="b")
    ax_all.set_ylabel("Aceleración [m/s²]")
    ax_all.set_title("Aceleración XYZ")
    ax_all.legend(loc="upper right")

    line_x, = ax_x.plot([], [], color="r")
    ax_x.set_title("Eje X")
    ax_x.set_ylabel("ax [m/s²]")

    line_y, = ax_y.plot([], [], color="g")
    ax_y.set_title("Eje Y")
    ax_y.set_ylabel("ay [m/s²]")

    line_z, = ax_z.plot([], [], color="b")
    ax_z.set_title("Eje Z")
    ax_z.set_ylabel("az [m/s²]")

    ax_z.set_xlabel("Tiempo (s)")
    ax_y.set_xlabel("Tiempo (s)")

    # Título general con la ubicación
    fig.suptitle(f"MPU6050 en tiempo real - {ubicacion}")

    # Título de la ventana (si el backend lo permite)
    try:
        fig.canvas.manager.set_window_title(f"MPU6050 - {ubicacion}")
    except Exception:
        pass

    # ---- Botón para detener ----
    ax_button = plt.axes([0.42, 0.04, 0.16, 0.06])  # [left, bottom, width, height]
    btn_stop = Button(ax_button, "Detener", hovercolor="0.8")

    def on_stop(event):
        print("Botón 'Detener' pulsado. Cerrando ventana...")
        plt.close(fig)

    btn_stop.on_clicked(on_stop)

    start_time_s = None

    def update(frame):
        nonlocal start_time_s

        # Leer todas las líneas disponibles
        while ser.in_waiting:
            try:
                line_bytes = ser.readline()
                if not line_bytes:
                    continue
                line_str = line_bytes.decode("utf-8", errors="ignore").strip()
                if not line_str:
                    continue

                # Esperamos: t_ms, ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw
                parts = line_str.split(",")
                if len(parts) < 7:
                    continue

                try:
                    t_ms, ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw = map(
                        float, parts[:7]
                    )
                except ValueError:
                    continue

                # Timestamp del PC en segundos (UNIX time)
                pc_time = time.time()

                # --- conversión a unidades físicas ---

                # de cuentas → g
                ax_g = ax_raw / ACC_SCALE
                ay_g = ay_raw / ACC_SCALE
                az_g = az_raw / ACC_SCALE

                # de g → m/s²
                ax_ms2 = ax_g * G_CONST
                ay_ms2 = ay_g * G_CONST
                az_ms2 = az_g * G_CONST

                # giroscopio: de cuentas → dps
                gx_dps = gx_raw / GYRO_SCALE
                gy_dps = gy_raw / GYRO_SCALE
                gz_dps = gz_raw / GYRO_SCALE

                # Tiempo en segundos relativo al inicio (basado en t_ms)
                if start_time_s is None:
                    start_time_s = t_ms / 1000.0
                t_s = t_ms / 1000.0 - start_time_s

                # Guardar en buffers para gráfica (m/s²)
                t_buf.append(t_s)
                ax_buf.append(ax_ms2)
                ay_buf.append(ay_ms2)
                az_buf.append(az_ms2)

                # Guardar en CSV (m/s² + dps + timestamp PC)
                csv_writer.writerow([
                    ubicacion, t_ms, pc_time,
                    ax_ms2, ay_ms2, az_ms2,
                    gx_dps, gy_dps, gz_dps
                ])
                csvfile.flush()

            except Exception:
                continue

        if not t_buf:
            return (line_all_x, line_all_y, line_all_z,
                    line_x, line_y, line_z)

        # Actualizar líneas
        line_all_x.set_data(t_buf, ax_buf)
        line_all_y.set_data(t_buf, ay_buf)
        line_all_z.set_data(t_buf, az_buf)

        line_x.set_data(t_buf, ax_buf)
        line_y.set_data(t_buf, ay_buf)
        line_z.set_data(t_buf, az_buf)

        # Ventana de tiempo común en el eje horizontal
        t_max = t_buf[-1]
        t_min = max(0, t_max - WINDOW_SECONDS)
        for axis in (ax_all, ax_x, ax_y, ax_z):
            axis.set_xlim(t_min, t_max)

        # ---- Rango vertical común para los 3 ejes X, Y y Z ----
        if ACC_LIMIT_MS2 is not None:
            ymin = -ACC_LIMIT_MS2
            ymax = ACC_LIMIT_MS2
        else:
            
            all_vals = list(ax_buf) + list(ay_buf) + list(az_buf)
            ymin = min(all_vals)
            ymax = max(all_vals)
            if ymin == ymax:
                ymin -= 1e-3
                ymax += 1e-3
            margen = 0.1 * (ymax - ymin)
            ymin -= margen
            ymax += margen

        # Aplicar el mismo rango vertical a todas las gráficas
        for axis in (ax_all, ax_x, ax_y, ax_z):
            axis.set_ylim(ymin, ymax)

        return (line_all_x, line_all_y, line_all_z,
                line_x, line_y, line_z)

    ani = animation.FuncAnimation(fig, update, interval=100)

    try:
        plt.show()
    finally:
        print("Guardando imagen de la gráfica...")
        try:
            fig.savefig(plot_path)
            print(f"Gráfica guardada en {plot_path}")
        except Exception as e:
            print(f"No se pudo guardar la gráfica: {e}")

        print("Cerrando puerto serie y archivo...")
        ser.close()
        csvfile.close()


if __name__ == "__main__":
    main()
