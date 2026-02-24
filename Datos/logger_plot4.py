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

OUTPUT_DIR = "/home/adrian/Giroscopio_PCB_EMT/Prueba"
BASE_NAME = "Martes_24"

WINDOW_SECONDS = 60
SAMPLE_HZ = 50                     # SOLO para el plot/ventana
MAX_POINTS = WINDOW_SECONDS * SAMPLE_HZ

# --- NUEVO: Hz objetivo para lo que se guarda/procesa ---
TARGET_LOG_HZ = 50.0               # fija el CSV por downsample usando t_ms (si la entrada no permite 50, quedará por debajo)
# --------------------------------------------------------

ACC_LIMIT_MS2 = None

DETECT_TIMEOUT_S = 6.0
STARTUP_SLEEP_S = 1.5
QUEUE_MAXSIZE = 20000               # subido para evitar pérdidas por picos

# CSV flush por bloques
CSV_FLUSH_ROWS = 500
CSV_FLUSH_SECS = 1.0

# Sincronización
SYNC_BROADCAST_ON_START = True
SYNC_WARMUP_SAMPLES = 300
SYNC_MAX_LAG_S = 5.0

# Filtro de saltos absurdos en t_ms (para evitar “horas”)
TMS_BACKWARD_MS = 1000.0            # si retrocede más de 1s -> descartar
TMS_JUMP_FORWARD_MS = 2000.0        # si salta más de 2s -> descartar

# ========= FIN CONFIGURACIÓN =========


def sanitize_filename_component(s):
    s = (s or "").strip()
    if not s:
        return "Desconocida"
    s = s.replace(" ", "_")
    s = re.sub(r"[^A-Za-z0-9_\-\.]", "", s)
    return s or "Desconocida"


def detectar_ubicacion(ser, timeout_s=5.0):
    try:
        ser.write(b"WHO\n")
        ser.flush()
    except Exception:
        pass

    inicio = time.time()
    ubicacion = "Desconocida"

    while time.time() - inicio < timeout_s:
        line_bytes = ser.readline()
        if not line_bytes:
            continue

        line_str = line_bytes.decode("utf-8", errors="ignore").strip()
        if not line_str:
            continue

        if line_str.startswith("Ubicacion:"):
            resto = line_str.split(":", 1)[1].strip()
            ubicacion = resto.split("\t")[0].strip()
            print(f"Ubicación detectada en {ser.port}: '{ubicacion}'")
            break

    return ubicacion


def is_candidate_port(dev):
    return dev.startswith("/dev/ttyACM") or dev.startswith("/dev/ttyUSB")


# ================= GPS: utilidades =================

def haversine_m(lat1, lon1, lat2, lon2):
    R = 6371000.0
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2.0) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dl / 2.0) ** 2
    c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))
    return R * c


def _nmea_to_decimal(coord_str, hemi, is_lat):
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


def parse_nmea_sentence(line):
    line = line.strip()
    if not line.startswith("$"):
        return None

    if "*" in line:
        payload, _chk = line.split("*", 1)
    else:
        payload = line

    parts = payload.split(",")
    if not parts:
        return None

    msg = parts[0]
    out = {"type": None}

    if msg.endswith("RMC") and len(parts) >= 7:
        status = parts[2] if len(parts) > 2 else ""
        lat = _nmea_to_decimal(parts[3], parts[4], is_lat=True) if len(parts) > 4 else None
        lon = _nmea_to_decimal(parts[5], parts[6], is_lat=False) if len(parts) > 6 else None

        out["type"] = "RMC"
        out["status"] = status
        out["lat"] = lat
        out["lon"] = lon
        return out

    if msg.endswith("GGA") and len(parts) >= 10:
        lat = _nmea_to_decimal(parts[2], parts[3], is_lat=True) if len(parts) > 3 else None
        lon = _nmea_to_decimal(parts[4], parts[5], is_lat=False) if len(parts) > 5 else None

        fix_q = sats = hdop = alt = None

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
    def __init__(self, ser):
        self.ser = ser
        self.port = ser.port

        self._lock = threading.Lock()
        self._stop = threading.Event()

        self.seq = 0
        self.last_update_pc_time = None
        self.lat = None
        self.lon = None
        self.alt_m = None
        self.sats = None
        self.fix = None
        self.hdop = None
        self.velocidad_kmh = None

        self._prev_lat = None
        self._prev_lon = None
        self._prev_time = None
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

    def _apply_update(self, update, pc_time):
        lat = update.get("lat", None)
        lon = update.get("lon", None)

        with self._lock:
            if "alt_m" in update and update["alt_m"] is not None:
                self.alt_m = update["alt_m"]
            if "sats" in update and update["sats"] is not None:
                self.sats = update["sats"]
            if "hdop" in update and update["hdop"] is not None:
                self.hdop = update["hdop"]

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

            if lat is not None and lon is not None:
                self.lat = lat
                self.lon = lon
            self.fix = fix_val

            if lat is None or lon is None:
                return
            if fix_val is not True:
                return

            emit = False
            if update.get("type") == "RMC":
                if update.get("status", "") == "A":
                    emit = True
                    self._last_rmc_pc_time = pc_time
            elif update.get("type") == "GGA":
                if self._last_rmc_pc_time is None or (pc_time - self._last_rmc_pc_time) > 1.2:
                    emit = True

            if not emit:
                return

            MIN_DT = 0.35
            if self.last_update_pc_time is not None:
                if (pc_time - self.last_update_pc_time) < MIN_DT:
                    return

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

                pc_time = time.perf_counter()
                self._apply_update(parsed, pc_time)

            except Exception:
                continue


class TimeSyncEstimator:
    def __init__(self, warmup_samples=SYNC_WARMUP_SAMPLES):
        self.warmup = warmup_samples
        self.n = 0
        self.sum_t = 0.0
        self.sum_p = 0.0
        self.sum_tt = 0.0
        self.sum_tp = 0.0
        self.a = None
        self.b = None

    def update(self, t_s, pc_s):
        self.n += 1
        self.sum_t += t_s
        self.sum_p += pc_s
        self.sum_tt += t_s * t_s
        self.sum_tp += t_s * pc_s

        if self.n >= max(20, self.warmup):
            denom = (self.n * self.sum_tt - self.sum_t * self.sum_t)
            if abs(denom) > 1e-9:
                b = (self.n * self.sum_tp - self.sum_t * self.sum_p) / denom
                a = (self.sum_p - b * self.sum_t) / self.n
                self.a = a
                self.b = b

    def estimate_pc(self, t_s):
        if self.a is None or self.b is None:
            return None
        return self.a + self.b * t_s


class DeviceSession:
    def __init__(self, ser, ubicacion, gps_manager, stop_all_cb, global_start_pc):
        self.ser = ser
        self.port = ser.port
        self.ubicacion = ubicacion
        self.gps_manager = gps_manager
        self.stop_all_cb = stop_all_cb

        self.ubicacion_safe = sanitize_filename_component(ubicacion)
        self.port_safe = sanitize_filename_component(os.path.basename(self.port))

        ts = time.strftime("%Y%m%d_%H%M%S")
        self.csv_path = os.path.join(OUTPUT_DIR, f"{BASE_NAME}_{self.ubicacion_safe}_{self.port_safe}_{ts}.csv")
        self.png_path = os.path.join(OUTPUT_DIR, f"{BASE_NAME}_{self.ubicacion_safe}_{self.port_safe}_{ts}.png")

        self.q = queue.Queue(maxsize=QUEUE_MAXSIZE)
        self.csv_q = queue.Queue(maxsize=QUEUE_MAXSIZE)

        self.stop_event = threading.Event()
        self.stopped_lock = threading.Lock()
        self._stopped = False

        self.t_buf = deque(maxlen=MAX_POINTS)
        self.ax_buf = deque(maxlen=MAX_POINTS)
        self.ay_buf = deque(maxlen=MAX_POINTS)
        self.az_buf = deque(maxlen=MAX_POINTS)

        self._baseline_ready = False
        self._baseline_n = 200
        self._bx = 0.0
        self._by = 0.0
        self._bz = 0.0
        self._baseline_buf = []

        self.global_start_pc = global_start_pc
        self.sync_est = TimeSyncEstimator()
        self._tms0 = None

        # --- downsample por t_ms ---
        self.target_log_hz = float(TARGET_LOG_HZ)
        self._min_dt_ms = 1000.0 / self.target_log_hz
        self._last_saved_t_ms = None
        self._saved_count = 0
        self._saved_t0_pc = time.perf_counter()
        # --------------------------

        # --- NUEVO: control de saltos de tiempo en entrada (DeviceSession, no GPS) ---
        self._last_rx_t_ms = None
        self.bad_time_jumps = 0
        # ---------------------------------------------------------------------------

        self._last_t_ms = None
        self.drop_gaps = 0
        self.total_samples = 0
        self.max_gap_ms = 0.0

        self.csvfile = open(self.csv_path, "w", newline="")
        self.csv_writer = csv.writer(self.csvfile)
        self.csv_writer.writerow([
            "ubicacion", "t_ms", "pc_time_s", "tiempo_s",
            "ax_ms2", "ay_ms2", "az_ms2",
            "gx_dps", "gy_dps", "gz_dps",
            "gps_lat", "gps_lon", "gps_alt_m", "gps_sats", "gps_fix", "gps_hdop", "velocidad",
            "sync_time_s", "pc_recv_s"
        ])
        self.csvfile.flush()

        self._rows_since_flush = 0
        self._last_flush_time = time.time()

        self.fig = None
        self.ani = None
        self.gps_text = None
        self._ylim = None
        self._setup_plot()

        self.reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.processor_thread = threading.Thread(target=self._processor_loop, daemon=True)
        self.writer_thread = threading.Thread(target=self._csv_writer_loop, daemon=True)

        self.reader_thread.start()
        self.processor_thread.start()
        self.writer_thread.start()

        print(f"[{self.port}] CSV:  {self.csv_path}")
        print(f"[{self.port}] PNG:  {self.png_path}")

    def _setup_plot(self):
        self.fig, ((ax_all, ax_x), (ax_y, ax_z)) = plt.subplots(2, 2, sharex="col", figsize=(10, 6))
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
            self.fig.canvas.manager.set_window_title(f"MPU6050 - {self.ubicacion} - {self.port}")
        except Exception:
            pass

        self.gps_text = self.fig.text(0.02, 0.02, "GPS: (no detectado)", fontsize=9, va="bottom", ha="left")

        def on_close(_event):
            print(f"[{self.port}] Ventana cerrada -> detener todo.")
            self.stop_all_cb()

        self.fig.canvas.mpl_connect("close_event", on_close)
        self.ani = animation.FuncAnimation(self.fig, self.update_plot, interval=100)

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
            self.csvfile.flush()
            self.csvfile.close()
        except Exception:
            pass

        dt_run = time.perf_counter() - self._saved_t0_pc
        saved_hz = (self._saved_count / dt_run) if dt_run > 0 else 0.0
        print(
            f"[{self.port}] Sesión detenida. total_rx={self.total_samples} "
            f"saved={self._saved_count} saved_hz~{saved_hz:.1f} "
            f"bad_time_jumps={self.bad_time_jumps} "
            f"gaps={self.drop_gaps} max_gap_ms={self.max_gap_ms:.1f}"
        )

    def _format_gps_text(self):
        if not self.gps_manager:
            return "GPS: (no detectado)"

        s = self.gps_manager.get_snapshot()
        lat, lon = s["lat"], s["lon"]
        alt, sats = s["alt_m"], s["sats"]
        fix, hdop = s["fix"], s["hdop"]
        vel = s["velocidad"]
        pc_t = s["pc_time"]

        age = (time.perf_counter() - pc_t) if pc_t is not None else None

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

    def _reader_loop(self):
        try:
            self.ser.reset_input_buffer()
        except Exception:
            pass

        rx_count = 0
        last_print = time.perf_counter()

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

                # filtro de saltos absurdos en t_ms (evita “horas” por una línea corrupta o reset raro)
                if self._last_rx_t_ms is not None:
                    dt = t_ms - self._last_rx_t_ms
                    if dt < -TMS_BACKWARD_MS or dt > TMS_JUMP_FORWARD_MS:
                        self.bad_time_jumps += 1
                        # NO actualizar _last_rx_t_ms para no encadenar basura
                        continue
                self._last_rx_t_ms = t_ms

                pc_recv = time.perf_counter()
                item = (t_ms, ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw, pc_recv)

                if self.q.full():
                    try:
                        _ = self.q.get_nowait()
                    except queue.Empty:
                        pass
                self.q.put_nowait(item)

                rx_count += 1
                now = time.perf_counter()
                if now - last_print > 2.0:
                    dt_run = now - self._saved_t0_pc
                    saved_hz = (self._saved_count / dt_run) if dt_run > 0 else 0.0
                    print(
                        f"[{self.port}] RX={rx_count} q={self.q.qsize()} csvq={self.csv_q.qsize()} "
                        f"saved={self._saved_count} saved_hz~{saved_hz:.1f} "
                        f"bad_time_jumps={self.bad_time_jumps} gaps={self.drop_gaps}"
                    )
                    last_print = now

            except Exception as e:
                # No tragues errores silenciosamente: esto fue lo que te generó CSV vacíos.
                print(f"[{self.port}] ERROR reader: {e}")
                continue

    def _processor_loop(self):
        while not self.stop_event.is_set():
            try:
                t_ms, ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw, pc_recv = self.q.get(timeout=0.2)
            except queue.Empty:
                continue

            self.total_samples += 1

            if self._last_t_ms is not None:
                gap = float(t_ms - self._last_t_ms)
                if gap > self.max_gap_ms:
                    self.max_gap_ms = gap
                if gap > (2.5 * self._min_dt_ms):
                    self.drop_gaps += 1
            self._last_t_ms = float(t_ms)

            # downsample para guardar ~TARGET_LOG_HZ
            if self._last_saved_t_ms is not None:
                if (t_ms - self._last_saved_t_ms) < self._min_dt_ms:
                    continue
            self._last_saved_t_ms = float(t_ms)
            self._saved_count += 1

            ax_ms2 = float(ax_raw)
            ay_ms2 = float(ay_raw)
            az_ms2 = float(az_raw)

            if not self._baseline_ready:
                self._baseline_buf.append((ax_ms2, ay_ms2, az_ms2))
                if len(self._baseline_buf) >= self._baseline_n:
                    self._bx = sum(v[0] for v in self._baseline_buf) / self._baseline_n
                    self._by = sum(v[1] for v in self._baseline_buf) / self._baseline_n
                    self._bz = sum(v[2] for v in self._baseline_buf) / self._baseline_n
                    self._baseline_ready = True
                    self._baseline_buf.clear()
                    print(f"[{self.port}] Baseline listo: bx={self._bx:.4f}, by={self._by:.4f}, bz={self._bz:.4f}")

            if self._baseline_ready:
                ax_ms2 -= self._bx
                ay_ms2 -= self._by
                az_ms2 -= self._bz

            gx_dps = float(gx_raw)
            gy_dps = float(gy_raw)
            gz_dps = float(gz_raw)

            if self._tms0 is None:
                self._tms0 = float(t_ms)
            tiempo_s = (float(t_ms) - self._tms0) / 1000.0
            pc_time_s = pc_recv

            self.sync_est.update(tiempo_s, pc_recv)
            pc_est = self.sync_est.estimate_pc(tiempo_s)
            if pc_est is None:
                sync_time_s = ""
            else:
                sync_time_s = pc_est - self.global_start_pc

            self.t_buf.append(tiempo_s)
            self.ax_buf.append(ax_ms2)
            self.ay_buf.append(ay_ms2)
            self.az_buf.append(az_ms2)

            gps_lat = gps_lon = gps_alt = gps_sats = gps_fix = gps_hdop = velocidad = ""
            if self.gps_manager:
                snap = self.gps_manager.get_snapshot()
                if snap["lat"] is not None and snap["lon"] is not None:
                    gps_lat = snap["lat"]
                    gps_lon = snap["lon"]
                    gps_alt = snap["alt_m"] if snap["alt_m"] is not None else ""
                    gps_sats = snap["sats"] if snap["sats"] is not None else ""
                    gps_fix = 1 if snap["fix"] is True else (0 if snap["fix"] is False else "")
                    gps_hdop = snap["hdop"] if snap["hdop"] is not None else ""
                    velocidad = snap["velocidad"] if snap["velocidad"] is not None else ""

            row = [
                self.ubicacion, t_ms, pc_time_s, tiempo_s,
                ax_ms2, ay_ms2, az_ms2,
                gx_dps, gy_dps, gz_dps,
                gps_lat, gps_lon, gps_alt, gps_sats, gps_fix, gps_hdop, velocidad,
                sync_time_s, pc_recv
            ]

            if self.csv_q.full():
                try:
                    _ = self.csv_q.get_nowait()
                except queue.Empty:
                    pass
            self.csv_q.put_nowait(row)

    def _csv_writer_loop(self):
        while not self.stop_event.is_set():
            try:
                row = self.csv_q.get(timeout=0.2)
            except queue.Empty:
                continue

            try:
                self.csv_writer.writerow(row)
                self._rows_since_flush += 1
                now = time.time()
                if self._rows_since_flush >= CSV_FLUSH_ROWS or (now - self._last_flush_time) >= CSV_FLUSH_SECS:
                    self.csvfile.flush()
                    self._rows_since_flush = 0
                    self._last_flush_time = now
            except Exception:
                continue

    def update_plot(self, _frame):
        try:
            if self.gps_text is not None:
                self.gps_text.set_text(self._format_gps_text())
        except Exception:
            pass

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
            if not all_vals:
                ymin, ymax = -1.0, 1.0
            else:
                abs_vals = sorted(abs(v) for v in all_vals)
                k = int(0.99 * (len(abs_vals) - 1))
                amp = abs_vals[k] if abs_vals else 0.0

                MIN_RANGE = 0.5
                amp = max(amp, MIN_RANGE / 2.0)

                new_ymin, new_ymax = -amp, amp
                margin = 0.15 * (new_ymax - new_ymin)
                new_ymin -= margin
                new_ymax += margin

                SMOOTH = 0.2
                if self._ylim is None:
                    ymin, ymax = new_ymin, new_ymax
                else:
                    oymin, oymax = self._ylim
                    ymin = (1 - SMOOTH) * oymin + SMOOTH * new_ymin
                    ymax = (1 - SMOOTH) * oymax + SMOOTH * new_ymax

                self._ylim = (ymin, ymax)

        for axis in (self.ax_all, self.ax_x, self.ax_y, self.ax_z):
            axis.set_ylim(ymin, ymax)

        return (
            self.line_all_x, self.line_all_y, self.line_all_z,
            self.line_x, self.line_y, self.line_z
        )


def _detect_gps_on_serial(dev):
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

        ser_esp = None
        try:
            ser_esp = serial.Serial(dev, BAUDRATE, timeout=1)

            try:
                ser_esp.dtr = False
                ser_esp.rts = False
                time.sleep(0.15)
                ser_esp.dtr = True
                ser_esp.rts = True
                time.sleep(0.35)
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

    fig._control_ax = ax_button
    fig._control_btn = btn

    return fig


def broadcast_sync(esp32_list):
    for ser, _loc in esp32_list:
        try:
            ser.write(b"SYNC\n")
            ser.flush()
        except Exception:
            pass


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

    global_start_pc = time.perf_counter()

    if SYNC_BROADCAST_ON_START:
        broadcast_sync(found)
        time.sleep(0.1)

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

    _control_fig = create_control_window(stop_all)

    for ser, ubicacion in found:
        try:
            sessions.append(DeviceSession(ser, ubicacion, gps_manager, stop_all, global_start_pc))
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