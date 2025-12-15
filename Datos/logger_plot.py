import os
import serial
import csv
import time
from collections import deque

import matplotlib.pyplot as plt
import matplotlib.animation as animation

# ========= CONFIGURACIÓN =========

# Puerto serie de la ESP32
SERIAL_PORT = "/dev/ttyACM0"   # En Windows sería algo tipo "COM3"
BAUDRATE = 115200

# Carpeta donde se guardarán CSV y gráfica
OUTPUT_DIR = "/home/adrian/Giroscopio_PCB_EMT/Resultados"  # CAMBIA ESTO SI QUIERES

# Nombre base de los ficheros (sin extensión)
BASE_NAME = "prueba_vibraciones1"  # CAMBIA ESTO PARA CADA ENSAYO

# Cuántas muestras usamos para calibrar con la placa en reposo
CALIBRATION_SAMPLES = 500   # ~5 s si vas a 100 Hz

# Umbral para considerar que es “cero” (en cuentas crudas)
NOISE_THRESHOLD = 100

# Ventana de tiempo para la gráfica (segundos)
WINDOW_SECONDS = 60

# Frecuencia de muestras aproximada (Hz) -> para dimensionar el buffer
SAMPLE_HZ = 100
MAX_POINTS = WINDOW_SECONDS * SAMPLE_HZ

# ========= FIN CONFIGURACIÓN =========


def main():
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    csv_path = os.path.join(OUTPUT_DIR, BASE_NAME + ".csv")
    plot_path = os.path.join(OUTPUT_DIR, BASE_NAME + ".png")

    print(f"CSV:   {csv_path}")
    print(f"Plot:  {plot_path}")

    print(f"Abrriendo puerto serie {SERIAL_PORT} a {BAUDRATE} baudios...")
    try:
        ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
    except Exception as e:
        print(f"Error al abrir el puerto serie: {e}")
        return

    time.sleep(2)

    try:
        csvfile = open(csv_path, "w", newline="")
    except Exception as e:
        print(f"No se pudo abrir el archivo CSV para escribir: {e}")
        ser.close()
        return

    csv_writer = csv.writer(csvfile)
    # añadimos columnas corregidas ax_c, ay_c, az_c
    csv_writer.writerow(["t_ms", "ax", "ay", "az", "gx", "gy", "gz",
                         "ax_c", "ay_c", "az_c"])
    print(f"Guardando datos en {csv_path}")

    # Buffers para gráfica (valores corregidos)
    t_buf = deque(maxlen=MAX_POINTS)
    ax_buf = deque(maxlen=MAX_POINTS)
    ay_buf = deque(maxlen=MAX_POINTS)
    az_buf = deque(maxlen=MAX_POINTS)

    # FIGURA CON 4 SUBPLOTS
    fig, ((ax_all, ax_x), (ax_y, ax_z)) = plt.subplots(2, 2, sharex="col", figsize=(10, 6))

    line_all_x, = ax_all.plot([], [], label="ax", color="r")
    line_all_y, = ax_all.plot([], [], label="ay", color="g")
    line_all_z, = ax_all.plot([], [], label="az", color="b")
    ax_all.set_ylabel("Accel (raw)")
    ax_all.set_title("Aceleración XYZ")
    ax_all.legend(loc="upper right")

    line_x, = ax_x.plot([], [], color="r")
    ax_x.set_title("Eje X")
    ax_x.set_ylabel("ax (raw)")

    line_y, = ax_y.plot([], [], color="g")
    ax_y.set_title("Eje Y")
    ax_y.set_ylabel("ay (raw)")

    line_z, = ax_z.plot([], [], color="b")
    ax_z.set_title("Eje Z")
    ax_z.set_ylabel("az (raw)")

    ax_z.set_xlabel("Tiempo (s)")
    ax_y.set_xlabel("Tiempo (s)")

    fig.suptitle("MPU6050 en tiempo real (filtrado / centrado)")

    start_time_s = None

    # ---- variables para calibración ----
    calib_count = 0
    sum_ax = sum_ay = sum_az = 0.0
    offset_ax = offset_ay = offset_az = 0.0
    calibrating = True

    print(f"Calibrando... mantén la PCB quieta durante {CALIBRATION_SAMPLES / SAMPLE_HZ:.1f} s")

    def update(frame):
        nonlocal start_time_s, calib_count, sum_ax, sum_ay, sum_az
        nonlocal offset_ax, offset_ay, offset_az, calibrating

        while ser.in_waiting:
            try:
                line_bytes = ser.readline()
                line_str = line_bytes.decode("utf-8", errors="ignore").strip()
                if not line_str:
                    continue

                parts = line_str.split(",")
                if len(parts) != 7:
                    continue

                t_ms, ax_v, ay_v, az_v, gx_v, gy_v, gz_v = map(float, parts)

                # ---- CALIBRACIÓN ----
                if calibrating:
                    sum_ax += ax_v
                    sum_ay += ay_v
                    sum_az += az_v
                    calib_count += 1

                    if calib_count >= CALIBRATION_SAMPLES:
                        offset_ax = sum_ax / calib_count
                        offset_ay = sum_ay / calib_count
                        offset_az = sum_az / calib_count
                        calibrating = False
                        print("Calibración completada.")
                        print(f"Offsets -> ax: {offset_ax:.1f}, ay: {offset_ay:.1f}, az: {offset_az:.1f}")
                # aplicar corrección (aunque estemos calibrando, así ves cómo evoluciona)
                ax_c = ax_v - offset_ax
                ay_c = ay_v - offset_ay
                az_c = az_v - offset_az

                # Umbral de ruido alrededor de 0
                if abs(ax_c) < NOISE_THRESHOLD:
                    ax_c = 0.0
                if abs(ay_c) < NOISE_THRESHOLD:
                    ay_c = 0.0
                if abs(az_c) < NOISE_THRESHOLD:
                    az_c = 0.0

                # ---- tiempo relativo ----
                if start_time_s is None:
                    start_time_s = t_ms / 1000.0
                t_s = t_ms / 1000.0 - start_time_s

                # Guardar en buffers (corregidos)
                t_buf.append(t_s)
                ax_buf.append(ax_c)
                ay_buf.append(ay_c)
                az_buf.append(az_c)

                # Guardar TODO en CSV: crudo + corregido
                csv_writer.writerow([t_ms, ax_v, ay_v, az_v, gx_v, gy_v, gz_v,
                                     ax_c, ay_c, az_c])
                csvfile.flush()

            except Exception:
                continue

        if not t_buf:
            return (line_all_x, line_all_y, line_all_z, line_x, line_y, line_z)

        # Actualizar líneas
        line_all_x.set_data(t_buf, ax_buf)
        line_all_y.set_data(t_buf, ay_buf)
        line_all_z.set_data(t_buf, az_buf)

        line_x.set_data(t_buf, ax_buf)
        line_y.set_data(t_buf, ay_buf)
        line_z.set_data(t_buf, az_buf)

        # Ventana de tiempo común
        t_max = t_buf[-1]
        t_min = max(0, t_max - WINDOW_SECONDS)
        for axis in (ax_all, ax_x, ax_y, ax_z):
            axis.set_xlim(t_min, t_max)

        # Y general
        ymin_all = min(min(ax_buf), min(ay_buf), min(az_buf))
        ymax_all = max(max(ax_buf), max(ay_buf), max(az_buf))
        if ymin_all == ymax_all:
            ymin_all -= 1
            ymax_all += 1
        margen_all = 0.1 * (ymax_all - ymin_all)
        ax_all.set_ylim(ymin_all - margen_all, ymax_all + margen_all)

        # Y individuales
        def set_ylim_from_buffer(axis, buf):
            ymin = min(buf)
            ymax = max(buf)
            if ymin == ymax:
                ymin -= 1
                ymax += 1
            margen = 0.1 * (ymax - ymin)
            axis.set_ylim(ymin - margen, ymax + margen)

        set_ylim_from_buffer(ax_x, ax_buf)
        set_ylim_from_buffer(ax_y, ay_buf)
        set_ylim_from_buffer(ax_z, az_buf)

        return (line_all_x, line_all_y, line_all_z, line_x, line_y, line_z)

    ani = animation.FuncAnimation(fig, update, interval=100)

    try:
        plt.tight_layout()
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
