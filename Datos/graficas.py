import os
import csv
import time
import math
from dataclasses import dataclass
from typing import List, Dict

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button, CheckButtons


# ===================== CONFIG (EDITA AQUÍ) =====================

CSV_PATHS = [
    "/home/adrian/Giroscopio_PCB_EMT/Cocheras_VIernes_13/prueba_Cocheras_OUT_1_Asiento_ttyUSB0_20260213_103433_fixed.csv",
]

OUTPUT_DIR = "/home/adrian/Giroscopio_PCB_EMT/Resultados/Plots"
OUTPUT_BASENAME = "Pruebas_Viernes_13"   # nombre base del PNG resultante

DEFAULT_WINDOW_S = 30.0

# ===============================================================


@dataclass
class Dataset:
    name: str
    t: np.ndarray
    ax: np.ndarray
    ay: np.ndarray
    az: np.ndarray
    v: np.ndarray


def _safe_float(x):
    if x is None:
        return math.nan
    s = str(x).strip()
    if s == "" or s.lower() in ("nan", "none"):
        return math.nan
    try:
        return float(s)
    except Exception:
        return math.nan


def load_dataset(csv_path: str):
    with open(csv_path, "r", newline="") as f:
        reader = csv.DictReader(f)
        rows = list(reader)

    if not rows:
        raise ValueError(f"CSV vacío: {csv_path}")

    t_list = []
    ax_list, ay_list, az_list, v_list = [], [], [], []

    has_tiempo = "tiempo_s" in rows[0]
    has_pc = "pc_time_s" in rows[0]

    pc0 = None
    for r in rows:
        if has_tiempo:
            t = _safe_float(r.get("tiempo_s"))
        elif has_pc:
            pc = _safe_float(r.get("pc_time_s"))
            if math.isnan(pc):
                t = math.nan
            else:
                if pc0 is None:
                    pc0 = pc
                t = pc - pc0
        else:
            t_ms = _safe_float(r.get("t_ms"))
            t = (t_ms / 1000.0) if not math.isnan(t_ms) else math.nan

        t_list.append(t)
        ax_list.append(_safe_float(r.get("ax_ms2")))
        ay_list.append(_safe_float(r.get("ay_ms2")))
        az_list.append(_safe_float(r.get("az_ms2")))
        v_list.append(_safe_float(r.get("velocidad")))  # km/h (puede ser NaN)

    t_arr = np.array(t_list, dtype=float)
    ax_arr = np.array(ax_list, dtype=float)
    ay_arr = np.array(ay_list, dtype=float)
    az_arr = np.array(az_list, dtype=float)
    v_arr = np.array(v_list, dtype=float)

    mask = np.isfinite(t_arr)
    t_arr = t_arr[mask]
    ax_arr = ax_arr[mask]
    ay_arr = ay_arr[mask]
    az_arr = az_arr[mask]
    v_arr = v_arr[mask]

    name = os.path.basename(csv_path)
    return Dataset(name=name, t=t_arr, ax=ax_arr, ay=ay_arr, az=az_arr, v=v_arr)


def _slice_by_time(t: np.ndarray, t0: float, t1: float):
    i0 = np.searchsorted(t, t0, side="left")
    i1 = np.searchsorted(t, t1, side="right")
    return np.arange(i0, i1)


def main():
    if not CSV_PATHS:
        raise SystemExit("Añade rutas en CSV_PATHS antes de ejecutar.")

    os.makedirs(OUTPUT_DIR, exist_ok=True)
    datasets: List[Dataset] = [load_dataset(p) for p in CSV_PATHS]

    t_max = max(ds.t[-1] for ds in datasets if ds.t.size > 0)
    t_min = 0.0
    if not np.isfinite(t_max) or t_max <= 0:
        t_max = 1.0

    # --------- Figura ---------
    fig, (ax_acc, ax_vel) = plt.subplots(2, 1, sharex=True, figsize=(12, 7))
    plt.subplots_adjust(left=0.08, right=0.78, bottom=0.18, top=0.92)

    fig.suptitle("Gráfico Vibraciones y Velocidad")

    ax_acc.set_ylabel("Aceleración [m/s²]")
    ax_acc.set_title("Aceleración XYZ")

    ax_vel.set_ylabel("Velocidad [km/h]")
    ax_vel.set_xlabel("Tiempo [s]")
    ax_vel.set_title("Velocidad vehículo")

    # --------- Líneas ---------
    lines: Dict[str, Dict[str, any]] = {}

    for ds in datasets:
        w0 = min(DEFAULT_WINDOW_S, t_max)
        idx = _slice_by_time(ds.t, 0.0, w0)

        l_ax, = ax_acc.plot(ds.t[idx], ds.ax[idx], label=f"{ds.name} | Eje X")
        l_ay, = ax_acc.plot(ds.t[idx], ds.ay[idx], label=f"{ds.name} | Eje Y")
        l_az, = ax_acc.plot(ds.t[idx], ds.az[idx], label=f"{ds.name} | Eje Z")

        tv = ds.t[idx]
        vv = ds.v[idx]
        m = np.isfinite(vv)
        l_v, = ax_vel.plot(tv[m], vv[m], "-o", label=f"{ds.name} | v", markersize=3)

        lines[ds.name] = {"Eje X": l_ax, "Eje Y": l_ay, "Eje Z": l_az, "v": l_v}

    ax_acc.legend(loc="upper left", fontsize=8)
    ax_vel.legend(loc="upper left", fontsize=8)

    # --------- Sliders ---------
    ax_start = fig.add_axes([0.08, 0.08, 0.60, 0.03])
    ax_win = fig.add_axes([0.08, 0.04, 0.60, 0.03])

    s_start = Slider(ax_start, "Inicio (s)", t_min, max(t_min, t_max - 0.1), valinit=0.0, valstep=0.1)
    s_win = Slider(ax_win, "Ventana (s)", 1.0, max(1.0, t_max), valinit=min(DEFAULT_WINDOW_S, t_max), valstep=0.5)

    # --------- CheckButtons señales ---------
    ax_checks = fig.add_axes([0.80, 0.67, 0.18, 0.25])
    labels = ["Eje X", "Eje Y", "Eje Z", "Velocidad"]
    actives = [True, True, True, True]
    checks = CheckButtons(ax_checks, labels, actives)

    # --------- Toggle auto-escala Y ---------
    ax_autoscale = fig.add_axes([0.80, 0.43, 0.18, 0.25])
    autoscale_btn = CheckButtons(ax_autoscale, ["Auto-escalado Eje Y"], [True])
    autoscale_enabled = {"on": True}

    # --------- Botones ---------
    ax_save = fig.add_axes([0.80, 0.28, 0.18, 0.07])
    btn_save = Button(ax_save, "Guardar PNG", hovercolor="0.85")

    ax_close = fig.add_axes([0.80, 0.18, 0.18, 0.07])
    btn_close = Button(ax_close, "Cerrar", hovercolor="0.85")

    visible = {"Eje X": True, "Eje Y": True, "Eje Z": True, "v": True}

    def auto_ylim(ax, arrays):
        vals = []
        for arr in arrays:
            if arr is None:
                continue
            a = arr[np.isfinite(arr)]
            if a.size:
                vals.append(a)
        if vals:
            allv = np.concatenate(vals)
            ymin = float(np.min(allv))
            ymax = float(np.max(allv))
            if ymin == ymax:
                ymin -= 1e-3
                ymax += 1e-3
            margen = 0.1 * (ymax - ymin)
            ax.set_ylim(ymin - margen, ymax + margen)

    def update_plot(_=None):
        t0 = float(s_start.val)
        w = float(s_win.val)
        t1 = min(t_max, t0 + w)

        # Actualiza datos por ventana
        for ds in datasets:
            idx = _slice_by_time(ds.t, t0, t1)

            lines[ds.name]["Eje X"].set_data(ds.t[idx], ds.ax[idx])
            lines[ds.name]["Eje Y"].set_data(ds.t[idx], ds.ay[idx])
            lines[ds.name]["Eje Z"].set_data(ds.t[idx], ds.az[idx])

            tv = ds.t[idx]
            vv = ds.v[idx]
            m = np.isfinite(vv)
            lines[ds.name]["v"].set_data(tv[m], vv[m])

            lines[ds.name]["Eje X"].set_visible(visible["Eje X"])
            lines[ds.name]["Eje Y"].set_visible(visible["Eje Y"])
            lines[ds.name]["Eje Z"].set_visible(visible["Eje Z"])
            lines[ds.name]["v"].set_visible(visible["v"])

        ax_acc.set_xlim(t0, t1)

        # Solo auto-escala si está activado
        if autoscale_enabled["on"]:
            # Aceleración: usar datos visibles
            acc_arrays = []
            for ds in datasets:
                idx = _slice_by_time(ds.t, t0, t1)
                if visible["Eje X"]:
                    acc_arrays.append(ds.ax[idx])
                if visible["Eje Y"]:
                    acc_arrays.append(ds.ay[idx])
                if visible["Eje Z"]:
                    acc_arrays.append(ds.az[idx])
            auto_ylim(ax_acc, acc_arrays)

            # Velocidad: SOLO puntos válidos (sin NaN) dentro de la ventana
            vel_arrays = []
            if visible["v"]:
                for ds in datasets:
                    idx = _slice_by_time(ds.t, t0, t1)
                    vv = ds.v[idx]
                    vel_arrays.append(vv[np.isfinite(vv)])
            auto_ylim(ax_vel, vel_arrays)

        fig.canvas.draw_idle()

    def on_check(label):
        if label == "Velocidad":
            visible["v"] = not visible["v"]
        else:
            visible[label] = not visible[label]
        update_plot()

    def on_autoscale(_label):
        autoscale_enabled["on"] = not autoscale_enabled["on"]
        update_plot()

    def on_save(_event):
        ts = time.strftime("%Y%m%d_%H%M%S")
        out_path = os.path.join(OUTPUT_DIR, f"{OUTPUT_BASENAME}_{ts}.png")
        fig.savefig(out_path, dpi=150)
        print(f"[OK] Guardado: {out_path}")

    def on_close(_event):
        plt.close(fig)

    s_start.on_changed(update_plot)
    s_win.on_changed(update_plot)
    checks.on_clicked(on_check)
    autoscale_btn.on_clicked(on_autoscale)
    btn_save.on_clicked(on_save)
    btn_close.on_clicked(on_close)

    # Mantener referencias para que sea clicable
    fig._widgets = (s_start, s_win, checks, autoscale_btn, btn_save, btn_close)

    update_plot()
    plt.show()


if __name__ == "__main__":
    main()
