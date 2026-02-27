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
    "/home/adrian/Giroscopio_PCB_EMT/Solo_Pruebas/Viernes_27/Viernes_27_80HZ_Suelo_ttyUSB0_20260227_103901.csv",
]

# Modo comparación: añade aquí un segundo CSV para comparar lado a lado.
# Déjalo vacío ([]) para usar el modo normal con los CSVs de arriba.
CSV_COMPARE = [
     "/home/adrian/Giroscopio_PCB_EMT/Solo_Pruebas/Viernes_27/Viernes_27_80HZ_Asiento_ttyUSB1_20260227_103901.csv",
]

OUTPUT_DIR = "/home/adrian/Giroscopio_PCB_EMT/Resultados/Plots"
OUTPUT_BASENAME = "Viernes_27_Suelo"   # nombre base del PNG resultante

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


# ======================== MODO NORMAL ========================

def main_normal():
    """Modo original: todos los CSV superpuestos en las mismas gráficas."""
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


# =================== MODO COMPARACIÓN ===================

def main_compare():
    """Modo comparación: dos CSV lado a lado, mismos controles sincronizados."""
    datasets_left: List[Dataset] = [load_dataset(p) for p in CSV_PATHS]
    datasets_right: List[Dataset] = [load_dataset(p) for p in CSV_COMPARE]

    os.makedirs(OUTPUT_DIR, exist_ok=True)

    all_datasets = datasets_left + datasets_right
    t_max = max(ds.t[-1] for ds in all_datasets if ds.t.size > 0)
    t_min = 0.0
    if not np.isfinite(t_max) or t_max <= 0:
        t_max = 1.0

    # --------- Colores consistentes para comparar ---------
    COLORS = {
        "Eje X": "#e6194b",  # rojo
        "Eje Y": "#3cb44b",  # verde
        "Eje Z": "#4363d8",  # azul
        "v":     "#f58231",  # naranja
    }

    # --------- Figura 2x2: izquierda=CSV1, derecha=CSV2 ---------
    fig, axes = plt.subplots(2, 2, figsize=(16, 8))
    plt.subplots_adjust(left=0.06, right=0.82, bottom=0.16, top=0.90,
                        wspace=0.22, hspace=0.35)

    ax_acc_L, ax_acc_R = axes[0, 0], axes[0, 1]
    ax_vel_L, ax_vel_R = axes[1, 0], axes[1, 1]

    # Compartir ejes Y entre izquierda y derecha para comparar visualmente
    ax_acc_R.sharey(ax_acc_L)
    ax_vel_R.sharey(ax_vel_L)

    # Nombres cortos para títulos
    name_L = datasets_left[0].name if datasets_left else "CSV 1"
    name_R = datasets_right[0].name if datasets_right else "CSV 2"

    fig.suptitle("Comparación de Vibraciones y Velocidad", fontsize=13, fontweight="bold")

    ax_acc_L.set_title(f"Aceleración – {name_L}", fontsize=9)
    ax_acc_R.set_title(f"Aceleración – {name_R}", fontsize=9)
    ax_vel_L.set_title(f"Velocidad – {name_L}", fontsize=9)
    ax_vel_R.set_title(f"Velocidad – {name_R}", fontsize=9)

    ax_acc_L.set_ylabel("Aceleración [m/s²]")
    ax_vel_L.set_ylabel("Velocidad [km/h]")
    ax_vel_L.set_xlabel("Tiempo [s]")
    ax_vel_R.set_xlabel("Tiempo [s]")

    # --------- Función auxiliar: dibujar líneas en un par de ejes ---------
    def _draw_side(ax_a, ax_v, dsets):
        side_lines = {}
        for ds in dsets:
            w0 = min(DEFAULT_WINDOW_S, t_max)
            idx = _slice_by_time(ds.t, 0.0, w0)

            l_ax, = ax_a.plot(ds.t[idx], ds.ax[idx], color=COLORS["Eje X"],
                              label="Eje X", linewidth=0.8)
            l_ay, = ax_a.plot(ds.t[idx], ds.ay[idx], color=COLORS["Eje Y"],
                              label="Eje Y", linewidth=0.8)
            l_az, = ax_a.plot(ds.t[idx], ds.az[idx], color=COLORS["Eje Z"],
                              label="Eje Z", linewidth=0.8)

            tv = ds.t[idx]
            vv = ds.v[idx]
            m = np.isfinite(vv)
            l_v, = ax_v.plot(tv[m], vv[m], "-o", color=COLORS["v"],
                             label="Velocidad", markersize=2, linewidth=0.8)

            side_lines[ds.name] = {"Eje X": l_ax, "Eje Y": l_ay, "Eje Z": l_az, "v": l_v}

        ax_a.legend(loc="upper left", fontsize=7)
        ax_v.legend(loc="upper left", fontsize=7)
        return side_lines

    lines_L = _draw_side(ax_acc_L, ax_vel_L, datasets_left)
    lines_R = _draw_side(ax_acc_R, ax_vel_R, datasets_right)

    # --------- Sliders (sincronizados) ---------
    ax_start = fig.add_axes([0.06, 0.07, 0.65, 0.025])
    ax_win = fig.add_axes([0.06, 0.035, 0.65, 0.025])

    s_start = Slider(ax_start, "Inicio (s)", t_min,
                     max(t_min, t_max - 0.1), valinit=0.0, valstep=0.1)
    s_win = Slider(ax_win, "Ventana (s)", 1.0, max(1.0, t_max),
                   valinit=min(DEFAULT_WINDOW_S, t_max), valstep=0.5)

    # --------- CheckButtons señales ---------
    ax_checks = fig.add_axes([0.83, 0.65, 0.15, 0.22])
    labels = ["Eje X", "Eje Y", "Eje Z", "Velocidad"]
    actives = [True, True, True, True]
    checks = CheckButtons(ax_checks, labels, actives)

    # --------- Toggle auto-escala Y + Sync Y ---------
    ax_autoscale = fig.add_axes([0.83, 0.42, 0.15, 0.20])
    autoscale_btn = CheckButtons(ax_autoscale,
                                 ["Auto-escalado Y", "Sync ejes Y"],
                                 [True, True])
    autoscale_enabled = {"on": True}
    sync_y_enabled = {"on": True}

    # --------- Botones ---------
    ax_save = fig.add_axes([0.83, 0.28, 0.15, 0.06])
    btn_save = Button(ax_save, "Guardar PNG", hovercolor="0.85")

    ax_close = fig.add_axes([0.83, 0.20, 0.15, 0.06])
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
            return ymin - margen, ymax + margen
        return None

    def _update_side_lines(dsets, side_lines, t0, t1):
        for ds in dsets:
            idx = _slice_by_time(ds.t, t0, t1)
            sl = side_lines[ds.name]
            sl["Eje X"].set_data(ds.t[idx], ds.ax[idx])
            sl["Eje Y"].set_data(ds.t[idx], ds.ay[idx])
            sl["Eje Z"].set_data(ds.t[idx], ds.az[idx])

            tv = ds.t[idx]
            vv = ds.v[idx]
            m = np.isfinite(vv)
            sl["v"].set_data(tv[m], vv[m])

            sl["Eje X"].set_visible(visible["Eje X"])
            sl["Eje Y"].set_visible(visible["Eje Y"])
            sl["Eje Z"].set_visible(visible["Eje Z"])
            sl["v"].set_visible(visible["v"])

    def _collect_arrays(dsets, t0, t1, key):
        arrays = []
        for ds in dsets:
            idx = _slice_by_time(ds.t, t0, t1)
            if key == "v":
                vv = ds.v[idx]
                arrays.append(vv[np.isfinite(vv)])
            else:
                mapping = {"Eje X": "ax", "Eje Y": "ay", "Eje Z": "az"}
                arrays.append(getattr(ds, mapping[key])[idx])
        return arrays

    def update_plot(_=None):
        t0 = float(s_start.val)
        w = float(s_win.val)
        t1 = min(t_max, t0 + w)

        _update_side_lines(datasets_left, lines_L, t0, t1)
        _update_side_lines(datasets_right, lines_R, t0, t1)

        for ax in [ax_acc_L, ax_acc_R, ax_vel_L, ax_vel_R]:
            ax.set_xlim(t0, t1)

        if autoscale_enabled["on"]:
            # Recopilar arrays visibles
            acc_L, acc_R, vel_L, vel_R = [], [], [], []
            for key in ["Eje X", "Eje Y", "Eje Z"]:
                if visible[key]:
                    acc_L.extend(_collect_arrays(datasets_left, t0, t1, key))
                    acc_R.extend(_collect_arrays(datasets_right, t0, t1, key))
            if visible["v"]:
                vel_L.extend(_collect_arrays(datasets_left, t0, t1, "v"))
                vel_R.extend(_collect_arrays(datasets_right, t0, t1, "v"))

            if sync_y_enabled["on"]:
                # Misma escala para ambos lados
                lim_acc = auto_ylim(None, acc_L + acc_R)
                lim_vel = auto_ylim(None, vel_L + vel_R)
                if lim_acc:
                    ax_acc_L.set_ylim(*lim_acc)
                    ax_acc_R.set_ylim(*lim_acc)
                if lim_vel:
                    ax_vel_L.set_ylim(*lim_vel)
                    ax_vel_R.set_ylim(*lim_vel)
            else:
                # Escala independiente
                lim = auto_ylim(None, acc_L)
                if lim:
                    ax_acc_L.set_ylim(*lim)
                lim = auto_ylim(None, acc_R)
                if lim:
                    ax_acc_R.set_ylim(*lim)
                lim = auto_ylim(None, vel_L)
                if lim:
                    ax_vel_L.set_ylim(*lim)
                lim = auto_ylim(None, vel_R)
                if lim:
                    ax_vel_R.set_ylim(*lim)

        fig.canvas.draw_idle()

    def on_check(label):
        if label == "Velocidad":
            visible["v"] = not visible["v"]
        else:
            visible[label] = not visible[label]
        update_plot()

    def on_autoscale(label):
        if label == "Auto-escalado Y":
            autoscale_enabled["on"] = not autoscale_enabled["on"]
        elif label == "Sync ejes Y":
            sync_y_enabled["on"] = not sync_y_enabled["on"]
        update_plot()

    def on_save(_event):
        ts = time.strftime("%Y%m%d_%H%M%S")
        out_path = os.path.join(OUTPUT_DIR, f"{OUTPUT_BASENAME}_comparacion_{ts}.png")
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

    fig._widgets = (s_start, s_win, checks, autoscale_btn, btn_save, btn_close)

    update_plot()
    plt.show()


# ======================== ENTRY POINT ========================

def main():
    if not CSV_PATHS:
        raise SystemExit("Añade rutas en CSV_PATHS antes de ejecutar.")

    if CSV_COMPARE:
        print("[INFO] Modo COMPARACIÓN activado: dos CSV lado a lado.")
        main_compare()
    else:
        print("[INFO] Modo normal: gráficas superpuestas.")
        main_normal()


if __name__ == "__main__":
    main()
