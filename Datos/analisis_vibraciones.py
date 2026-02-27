#!/usr/bin/env python3
"""
analisis_vibraciones.py — Análisis post-proceso de señales de vibración (v2)

Mejoras sobre v1:
  · ISO 2631-1: ponderación Wk (eje Z) / Wd (X/Y), VDV, Crest Factor
  · Jerk (tasa de cambio de aceleración) + RMS deslizante
  · Espectro FFT de amplitudes → CSV exportado
  · Tabla de picos frecuenciales dominantes (frecuencia + amplitud)
  · Nuevas gráficas: señal ponderada, VDV acumulado, jerk
  · Argparse: --no_iso, --jerk_window_s, --n_peaks
"""
import argparse
import json
import os
from dataclasses import dataclass

import numpy as np
import pandas as pd
from scipy.signal import butter, sosfiltfilt, sosfreqz, welch, correlate, find_peaks
import matplotlib.pyplot as plt


# ─────────────────────────────────────────────────────────────────────────────
#  Utilidades de tiempo y carga
# ─────────────────────────────────────────────────────────────────────────────

def pick_time_column(df: pd.DataFrame) -> str:
    for c in ["sync_time_s", "tiempo_s", "t_ms"]:
        if c in df.columns:
            return c
    raise ValueError("No encuentro columna de tiempo (sync_time_s/tiempo_s/t_ms).")


def get_time_seconds(df: pd.DataFrame, time_col: str):
    t = pd.to_numeric(df[time_col], errors="coerce").to_numpy(dtype=float)
    m = np.isfinite(t)
    t = t[m]
    if time_col == "t_ms":
        t = t / 1000.0
    t = t - t[0]
    return t, m


def pick_axis(df: pd.DataFrame, axis: str) -> str:
    col = {"x": "ax_ms2", "y": "ay_ms2", "z": "az_ms2"}[axis]
    if col not in df.columns:
        raise ValueError(f"No existe columna {col} en el CSV.")
    return col


def estimate_fs_from_t(t: np.ndarray):
    dt = np.diff(t)
    dt = dt[np.isfinite(dt) & (dt > 0)]
    if len(dt) < 10:
        return np.nan, np.nan, dt
    fs_med = 1.0 / np.median(dt)
    dt_p99 = np.percentile(dt, 99)
    return fs_med, dt_p99, dt


# ─────────────────────────────────────────────────────────────────────────────
#  Filtros digitales
# ─────────────────────────────────────────────────────────────────────────────

def bandpass_sos(fs: float, hp: float, lp: float, order: int = 4):
    hp = max(hp, 1e-6)
    lp = min(lp, 0.499 * fs)
    if lp <= hp:
        raise ValueError(f"Bandpass inválido: hp={hp}, lp={lp}, fs={fs}")
    return butter(order, [hp / (fs / 2.0), lp / (fs / 2.0)], btype="band", output="sos")


def lowpass_sos(fs: float, lp: float, order: int = 6):
    lp = min(lp, 0.499 * fs)
    return butter(order, lp / (fs / 2.0), btype="low", output="sos")


def highpass_sos(fs: float, hp: float, order: int = 2):
    hp = max(hp, 1e-6)
    hp = min(hp, 0.499 * fs)
    return butter(order, hp / (fs / 2.0), btype="high", output="sos")


# ─────────────────────────────────────────────────────────────────────────────
#  ISO 2631-1: filtros de ponderación
# ─────────────────────────────────────────────────────────────────────────────

def design_iso2631_filters(fs: float):
    """
    Diseña los filtros de ponderación ISO 2631-1 como SOS digitales.

    Wk (eje Z vertical):
      HP 2º@0.4Hz + bandpass@5.6Hz Q=0.91 + LP 2º@12.5Hz
    Wd (ejes X/Y horizontales):
      HP 2º@0.4Hz + LP 2º@2.0Hz

    Devuelve (sos_wk, gain_wk, sos_wd, gain_wd).
    """
    nyq = fs / 2.0

    # --- Wk ---
    sos_hp = butter(2, 0.4 / nyq, btype="high", output="sos")
    f0_k, Q_k = 5.6, 0.91
    bw_k = f0_k / Q_k
    fl_k = max(0.2, f0_k - bw_k / 2)
    fh_k = min(nyq * 0.9, f0_k + bw_k / 2)
    sos_bp = butter(2, [fl_k / nyq, fh_k / nyq], btype="band", output="sos")
    sos_lp_k = butter(2, min(12.5 / nyq, 0.9), btype="low", output="sos")
    sos_wk = np.vstack([sos_hp, sos_bp, sos_lp_k])
    _, H = sosfreqz(sos_wk, worN=[2 * np.pi * f0_k / fs])
    gain_wk = max(float(abs(H[0])), 1e-12)

    # --- Wd ---
    sos_lp_d = butter(2, min(2.0 / nyq, 0.9), btype="low", output="sos")
    sos_wd = np.vstack([sos_hp, sos_lp_d])
    _, H = sosfreqz(sos_wd, worN=[2 * np.pi * 1.5 / fs])
    gain_wd = max(float(abs(H[0])), 1e-12)

    return sos_wk, gain_wk, sos_wd, gain_wd


def apply_iso2631(x: np.ndarray, axis: str, fs: float) -> np.ndarray:
    """Aplica ponderación ISO 2631-1 (fase cero, sosfiltfilt) a la señal x."""
    sos_wk, gain_wk, sos_wd, gain_wd = design_iso2631_filters(fs)
    if axis == "z":
        return sosfiltfilt(sos_wk, x) / gain_wk
    else:  # x o y
        return sosfiltfilt(sos_wd, x) / gain_wd


# ─────────────────────────────────────────────────────────────────────────────
#  Métricas ISO 2631-1
# ─────────────────────────────────────────────────────────────────────────────

def iso_metrics(xw: np.ndarray, fs: float) -> dict:
    """
    Calcula métricas globales ISO 2631-1 sobre la señal ponderada xw.

    Devuelve:
      aw_rms  — RMS de la señal ponderada (m/s²)
      vdv     — Vibration Dose Value (m/s^1.75): (Σ xw⁴·dt)^0.25
      cf      — Crest Factor: |pico| / aw_rms  (si CF>9: preferir VDV)
    """
    dt = 1.0 / fs
    aw_rms = float(np.sqrt(np.mean(np.square(xw))))
    vdv = float((np.sum(xw ** 4) * dt) ** 0.25)
    cf = float(np.max(np.abs(xw)) / aw_rms) if aw_rms > 1e-12 else float("nan")
    return {"aw_rms": aw_rms, "vdv": vdv, "cf": cf}


def vdv_cumulative(xw: np.ndarray, fs: float) -> np.ndarray:
    """Devuelve el VDV acumulado muestra a muestra."""
    dt = 1.0 / fs
    cum = np.cumsum(xw ** 4) * dt
    return cum ** 0.25


# ─────────────────────────────────────────────────────────────────────────────
#  Jerk
# ─────────────────────────────────────────────────────────────────────────────

def compute_jerk(x: np.ndarray, fs: float) -> np.ndarray:
    """Jerk = d(aceleración)/dt por diferencia hacia atrás [m/s³]."""
    j = np.diff(x, prepend=x[0]) * fs
    return j


def rolling_rms(x: np.ndarray, fs: float, window_s: float) -> np.ndarray:
    """RMS deslizante de ventana window_s segundos."""
    n = max(1, int(window_s * fs))
    out = np.zeros_like(x)
    for i in range(len(x)):
        i0 = max(0, i - n + 1)
        out[i] = np.sqrt(np.mean(np.square(x[i0:i + 1])))
    return out


def jerk_metrics(x: np.ndarray, fs: float) -> dict:
    j = compute_jerk(x, fs)
    return {
        "rms_ms3": float(np.sqrt(np.mean(np.square(j)))),
        "p99_ms3": float(np.percentile(np.abs(j), 99)),
        "max_ms3": float(np.max(np.abs(j))),
    }


# ─────────────────────────────────────────────────────────────────────────────
#  FFT de amplitudes
# ─────────────────────────────────────────────────────────────────────────────

def compute_fft_amplitude(x: np.ndarray, fs: float, fmax: float):
    """
    Espectro de amplitudes via FFT (ventana Hann, promediado en bloques).

    Usa el mismo número de segmentos que Welch para coherencia.
    Devuelve (freqs, amp) donde amp está en m/s² (amplitud de pico).
    """
    nfft = int(min(len(x), max(2048, fs * 8)))
    if nfft % 2 == 1:
        nfft -= 1

    window = np.hanning(nfft)
    step = nfft // 2
    n_segs = max(1, (len(x) - nfft) // step + 1)

    acc = np.zeros(nfft // 2 + 1)
    for k in range(n_segs):
        seg = x[k * step: k * step + nfft]
        if len(seg) < nfft:
            break
        seg = (seg - np.mean(seg)) * window
        sp = np.fft.rfft(seg, n=nfft)
        mag = np.abs(sp) * (2.0 / np.sum(window))  # amplitud pico
        acc += mag
    acc /= n_segs

    freqs = np.fft.rfftfreq(nfft, d=1.0 / fs)
    m = (freqs >= 0.1) & (freqs <= fmax)
    return freqs[m], acc[m]


# ─────────────────────────────────────────────────────────────────────────────
#  Picos dominantes en espectro
# ─────────────────────────────────────────────────────────────────────────────

def find_spectral_peaks(freqs: np.ndarray, amp: np.ndarray,
                        n_peaks: int = 10, min_prom_rel: float = 0.05) -> pd.DataFrame:
    """
    Detecta hasta n_peaks picos dominantes en el espectro de amplitud.

    min_prom_rel: prominencia mínima como fracción del máximo de amp.
    """
    if len(amp) == 0:
        return pd.DataFrame(columns=["freq_hz", "amplitude_ms2", "prominence"])

    prom_abs = min_prom_rel * float(np.max(amp))
    peaks, props = find_peaks(amp, prominence=prom_abs, distance=3)

    if len(peaks) == 0:
        return pd.DataFrame(columns=["freq_hz", "amplitude_ms2", "prominence"])

    order = np.argsort(props["prominences"])[::-1][:n_peaks]
    idx = peaks[order]
    rows = [
        {
            "freq_hz": float(freqs[i]),
            "amplitude_ms2": float(amp[i]),
            "prominence": float(props["prominences"][order[j]]),
        }
        for j, i in enumerate(idx)
    ]
    df = pd.DataFrame(rows).sort_values("freq_hz").reset_index(drop=True)
    return df


# ─────────────────────────────────────────────────────────────────────────────
#  Tercios de octava y PSD (existentes, refactorizados)
# ─────────────────────────────────────────────────────────────────────────────

def third_octave_centers(fmin=0.5, fmax=80.0):
    fc = []
    f = 1.0
    while f > fmin:
        f /= 2 ** (1 / 3)
    f = max(f, fmin)
    while f <= fmax * 1.001:
        fc.append(f)
        f *= 2 ** (1 / 3)
    return np.array(fc, dtype=float)


def third_octave_edges(fc: np.ndarray):
    k = 2 ** (1 / 6)
    return fc / k, fc * k


def rms(x):
    return float(np.sqrt(np.mean(np.square(x))))


def integrate_psd_to_rms(f, Pxx, f1, f2):
    m = (f >= f1) & (f < f2)
    if not np.any(m):
        return np.nan
    var = np.trapz(Pxx[m], f[m])
    return float(np.sqrt(max(var, 0.0)))


def compute_third_octave_rms(xu, fs_u, fmax):
    fc = third_octave_centers(0.5, fmax)
    lo, hi = third_octave_edges(fc)
    rows = []
    for f1, f2, fcenter in zip(lo, hi, fc):
        if f2 >= 0.49 * fs_u:
            continue
        sos = bandpass_sos(fs_u, f1, f2, order=4)
        y = sosfiltfilt(sos, xu - np.mean(xu))
        rows.append({"fc_hz": float(fcenter), "f_lo": float(f1), "f_hi": float(f2), "rms": rms(y)})
    return pd.DataFrame(rows)


def compute_psd(xu, fs_u, fmax):
    nper = int(min(len(xu), max(1024, fs_u * 8)))
    if nper % 2 == 1:
        nper -= 1
    f, P = welch(xu - np.mean(xu), fs=fs_u, nperseg=nper,
                 noverlap=nper // 2, window="hann", detrend=False)
    m = (f >= 0.5) & (f <= fmax)
    return f[m], P[m]


def compute_peaks(xu, fs_u, fmax, prom=1.0, min_dist_s=0.3):
    sos = bandpass_sos(fs_u, 0.5, min(fmax, 0.45 * fs_u))
    y = sosfiltfilt(sos, xu - np.mean(xu))
    peaks, _ = find_peaks(np.abs(y), prominence=prom, distance=int(min_dist_s * fs_u))
    v = np.abs(y)[peaks]
    if len(v) == 0:
        return {"n": 0}
    return {
        "n": int(len(v)),
        "p95": float(np.percentile(v, 95)),
        "p99": float(np.percentile(v, 99)),
        "max": float(np.max(v)),
    }


def read_reference_curve(path: str):
    df = pd.read_csv(path)
    if "fc_hz" not in df.columns or "ref" not in df.columns:
        raise ValueError("Curva de referencia debe tener columnas: fc_hz, ref")
    return df[["fc_hz", "ref"]].copy()


# ─────────────────────────────────────────────────────────────────────────────
#  Re-muestreo y alineación
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class Signal:
    name: str
    t: np.ndarray
    x: np.ndarray
    fs_med: float
    dt_p99: float


def load_signal(path: str, axis: str):
    df = pd.read_csv(path)
    tcol = pick_time_column(df)
    t, m = get_time_seconds(df, tcol)
    col = pick_axis(df, axis)
    x = pd.to_numeric(df.loc[m, col], errors="coerce").to_numpy(dtype=float)
    ok = np.isfinite(t) & np.isfinite(x)
    t = t[ok]
    x = x[ok]
    fs_med, dt_p99, _ = estimate_fs_from_t(t)
    return df, Signal(os.path.basename(path), t, x, fs_med, dt_p99), tcol, col


def choose_fmax(fs_med_a, dt_p99_a, fs_med_b, dt_p99_b, fmax_target):
    fs_p99_a = 1.0 / dt_p99_a if np.isfinite(dt_p99_a) and dt_p99_a > 0 else np.nan
    fs_p99_b = 1.0 / dt_p99_b if np.isfinite(dt_p99_b) and dt_p99_b > 0 else np.nan
    fs_like = np.nanmin([fs_p99_a, fs_p99_b, fs_med_a, fs_med_b])

    if not np.isfinite(fs_like) or fs_like <= 0:
        return min(20.0, fmax_target), "fallback"

    fmax_safe = fs_like / 2.5
    fmax = min(fmax_target, fmax_safe)
    fmax = max(5.0, fmax)
    return fmax, f"fs_like={fs_like:.2f}, fmax_safe={fmax_safe:.2f}"


def resample_uniform(t: np.ndarray, x: np.ndarray, fs_u: float):
    tu = np.arange(t[0], t[-1], 1.0 / fs_u)
    xu = np.interp(tu, t, x)
    return tu, xu


def robust_offset_by_xcorr(xa, xb, fs, max_lag_s=5.0):
    sos = bandpass_sos(fs, 0.5, min(10.0, 0.45 * fs))
    a = sosfiltfilt(sos, xa - np.mean(xa))
    b = sosfiltfilt(sos, xb - np.mean(xb))

    max_lag = int(max_lag_s * fs)
    c = correlate(a, b, mode="full")
    lags = np.arange(-len(b) + 1, len(a))

    mid = len(c) // 2
    c_win = c[mid - max_lag: mid + max_lag + 1]
    l_win = lags[mid - max_lag: mid + max_lag + 1]

    lag = int(l_win[np.argmax(c_win)])
    return lag / fs


# ─────────────────────────────────────────────────────────────────────────────
#  Gráficas
# ─────────────────────────────────────────────────────────────────────────────

def plot_outputs(outdir, axis, t_common, x1_u, x2_u, fs_u, fmax,
                 df_to_1, df_to_2, psd_1, psd_2,
                 xw1=None, xw2=None,
                 jerk1=None, jerk2=None, jerk_window_s=1.0,
                 ref_curve=None):
    os.makedirs(outdir, exist_ok=True)

    # ── 1) Señal temporal (últimos 60 s) ─────────────────────────────────────
    plt.figure()
    span = 60.0
    tmax = t_common[-1]
    tmin = max(0.0, tmax - span)
    m = (t_common >= tmin) & (t_common <= tmax)
    plt.plot(t_common[m], x1_u[m], label="CSV1")
    plt.plot(t_common[m], x2_u[m], label="CSV2")
    plt.xlabel("Tiempo (s)")
    plt.ylabel(f"a{axis} (m/s²)")
    plt.title(f"Señal temporal (últimos {span:.0f}s) | fs_u={fs_u:.1f} Hz | fmax={fmax:.1f} Hz")
    plt.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(outdir, f"time_{axis}.png"), dpi=160)
    plt.close()

    # ── 2) PSD ───────────────────────────────────────────────────────────────
    f1, P1 = psd_1
    f2, P2 = psd_2
    plt.figure()
    plt.semilogy(f1, P1, label="CSV1")
    plt.semilogy(f2, P2, label="CSV2")
    plt.xlabel("Frecuencia (Hz)")
    plt.ylabel("PSD (m²/s⁴/Hz)")
    plt.title("PSD (Welch)")
    plt.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(outdir, f"psd_{axis}.png"), dpi=160)
    plt.close()

    # ── 3) Tercios de octava ─────────────────────────────────────────────────
    merged = pd.merge(df_to_1, df_to_2, on="fc_hz", suffixes=("_1", "_2"))
    plt.figure(figsize=(10, 4.5))
    x = np.arange(len(merged))
    plt.bar(x - 0.2, merged["rms_1"], width=0.4, label="CSV1")
    plt.bar(x + 0.2, merged["rms_2"], width=0.4, label="CSV2")
    plt.xticks(x, [f"{v:.2g}" for v in merged["fc_hz"]], rotation=60)
    plt.xlabel("Centro de banda (Hz) – 1/3 octava")
    plt.ylabel("RMS (m/s²)")
    plt.title("RMS por tercios de octava")
    if ref_curve is not None:
        ref_interp = np.interp(merged["fc_hz"], ref_curve["fc_hz"], ref_curve["ref"])
        plt.plot(x, ref_interp, marker="o", linewidth=1.5, label="Referencia")
    plt.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(outdir, f"third_octave_{axis}.png"), dpi=160)
    plt.close()

    # ── 4) Ratio CSV2/CSV1 ───────────────────────────────────────────────────
    plt.figure(figsize=(10, 4.5))
    ratio = merged["rms_2"] / np.maximum(merged["rms_1"], 1e-9)
    plt.bar(x, ratio)
    plt.xticks(x, [f"{v:.2g}" for v in merged["fc_hz"]], rotation=60)
    plt.axhline(1.0, linestyle="--")
    plt.xlabel("Centro de banda (Hz) – 1/3 octava")
    plt.ylabel("CSV2 / CSV1 (RMS)")
    plt.title("Comparación por banda (>1 amplifica, <1 atenúa)")
    plt.tight_layout()
    plt.savefig(os.path.join(outdir, f"ratio_{axis}.png"), dpi=160)
    plt.close()

    # ── 5) Señal ponderada ISO 2631-1 (últimos 60 s) ─────────────────────────
    if xw1 is not None and xw2 is not None:
        plt.figure()
        plt.plot(t_common[m], xw1[m], label="CSV1 (ponderada)")
        plt.plot(t_common[m], xw2[m], label="CSV2 (ponderada)")
        label_filter = "Wk" if axis == "z" else "Wd"
        plt.xlabel("Tiempo (s)")
        plt.ylabel(f"a{axis} ponderada {label_filter} (m/s²)")
        plt.title(f"Señal ISO 2631-1 ({label_filter}) — últimos {span:.0f}s")
        plt.legend()
        plt.tight_layout()
        plt.savefig(os.path.join(outdir, f"time_weighted_{axis}.png"), dpi=160)
        plt.close()

        # ── 5b) VDV acumulado ─────────────────────────────────────────────────
        vdv1 = vdv_cumulative(xw1, fs_u)
        vdv2 = vdv_cumulative(xw2, fs_u)
        plt.figure()
        plt.plot(t_common, vdv1, label="CSV1")
        plt.plot(t_common, vdv2, label="CSV2")
        plt.xlabel("Tiempo (s)")
        plt.ylabel("VDV acumulado (m/s^1.75)")
        plt.title("Vibration Dose Value acumulado (ISO 2631-1)")
        plt.legend()
        plt.tight_layout()
        plt.savefig(os.path.join(outdir, f"vdv_{axis}.png"), dpi=160)
        plt.close()

    # ── 6) Jerk (RMS deslizante) ──────────────────────────────────────────────
    if jerk1 is not None and jerk2 is not None:
        jr1 = rolling_rms(jerk1, fs_u, jerk_window_s)
        jr2 = rolling_rms(jerk2, fs_u, jerk_window_s)
        plt.figure()
        plt.plot(t_common[m], jr1[m], label="CSV1")
        plt.plot(t_common[m], jr2[m], label="CSV2")
        plt.xlabel("Tiempo (s)")
        plt.ylabel(f"Jerk RMS ({jerk_window_s:.1f}s ventana) (m/s³)")
        plt.title("Jerk — brusquedad de la vibración")
        plt.legend()
        plt.tight_layout()
        plt.savefig(os.path.join(outdir, f"jerk_{axis}.png"), dpi=160)
        plt.close()


def plot_fft_amplitudes(outdir, axis, freqs1, amp1, freqs2, amp2,
                        peaks1: pd.DataFrame, peaks2: pd.DataFrame):
    """Gráfica del espectro FFT de amplitudes con los picos marcados."""
    plt.figure(figsize=(10, 5))
    plt.plot(freqs1, amp1, label="CSV1", alpha=0.8)
    plt.plot(freqs2, amp2, label="CSV2", alpha=0.8)

    # Marcadores de picos CSV1
    if not peaks1.empty:
        plt.scatter(peaks1["freq_hz"], peaks1["amplitude_ms2"],
                    marker="v", s=60, zorder=5, label="Picos CSV1")
        for _, row in peaks1.iterrows():
            plt.annotate(f"{row['freq_hz']:.2f} Hz",
                         xy=(row["freq_hz"], row["amplitude_ms2"]),
                         xytext=(4, 4), textcoords="offset points", fontsize=7)

    # Marcadores de picos CSV2
    if not peaks2.empty:
        plt.scatter(peaks2["freq_hz"], peaks2["amplitude_ms2"],
                    marker="^", s=60, zorder=5, label="Picos CSV2")
        for _, row in peaks2.iterrows():
            plt.annotate(f"{row['freq_hz']:.2f} Hz",
                         xy=(row["freq_hz"], row["amplitude_ms2"]),
                         xytext=(4, -12), textcoords="offset points", fontsize=7)

    plt.xlabel("Frecuencia (Hz)")
    plt.ylabel("Amplitud (m/s²)")
    plt.title("Espectro de amplitudes FFT con picos dominantes")
    plt.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(outdir, f"fft_amplitudes_{axis}.png"), dpi=160)
    plt.close()


# ─────────────────────────────────────────────────────────────────────────────
#  main
# ─────────────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(
        description="Análisis post-proceso de vibraciones: PSD, tercios de octava, "
                    "ISO 2631-1, FFT de amplitudes y tabla de picos."
    )
    ap.add_argument("--csv1", required=True, help="Primer CSV")
    ap.add_argument("--csv2", required=True, help="Segundo CSV")
    ap.add_argument("--axis", default="z", choices=["x", "y", "z"],
                    help="Eje a analizar (vertical normalmente z)")
    ap.add_argument("--fmax", type=float, default=80.0,
                    help="Objetivo f_max Hz (se recorta al límite de muestreo)")
    ap.add_argument("--out", default="out_vibraciones", help="Directorio de salida")
    ap.add_argument("--ref_curve", default=None,
                    help="CSV opcional con columnas: fc_hz,ref")
    ap.add_argument("--prom", type=float, default=1.0,
                    help="Prominence para picos temporales (m/s²)")
    ap.add_argument("--min_peak_dist", type=float, default=0.30,
                    help="Distancia mínima entre picos temporales (s)")
    # Nuevas opciones
    ap.add_argument("--no_iso", action="store_true",
                    help="Desactiva el análisis ISO 2631-1 (Wk/Wd, VDV, CF)")
    ap.add_argument("--jerk_window_s", type=float, default=1.0,
                    help="Ventana para RMS deslizante del jerk (s)")
    ap.add_argument("--n_peaks", type=int, default=10,
                    help="Número máximo de picos espectrales a reportar")
    args = ap.parse_args()

    # ── Carga de señales ──────────────────────────────────────────────────────
    _, s1, tcol1, acol1 = load_signal(args.csv1, args.axis)
    _, s2, tcol2, acol2 = load_signal(args.csv2, args.axis)

    fmax, note = choose_fmax(s1.fs_med, s1.dt_p99, s2.fs_med, s2.dt_p99, args.fmax)

    fs_u = float(max(100.0, 5.0 * fmax))
    fs_u_cap = (1.2 * min(s1.fs_med, s2.fs_med)
                if np.isfinite(s1.fs_med) and np.isfinite(s2.fs_med) else fs_u)
    fs_u = float(min(fs_u, max(100.0, fs_u_cap)))

    # ── Alineación temporal ───────────────────────────────────────────────────
    t0 = max(s1.t[0], s2.t[0])
    t1 = min(s1.t[-1], s2.t[-1])
    if t1 <= t0:
        raise SystemExit("No hay solape temporal entre CSVs.")

    m1 = (s1.t >= t0) & (s1.t <= t1)
    m2 = (s2.t >= t0) & (s2.t <= t1)
    t1c, x1c = s1.t[m1], s1.x[m1]
    t2c, x2c = s2.t[m2], s2.x[m2]

    fs_sync = float(min(s1.fs_med, s2.fs_med, 120.0))
    tt1, xx1 = resample_uniform(t1c, x1c, fs_sync)
    tt2, xx2 = resample_uniform(t2c, x2c, fs_sync)
    offset = robust_offset_by_xcorr(xx1, xx2, fs_sync, max_lag_s=5.0)

    t2_al = s2.t + offset
    t0 = max(s1.t[0], t2_al[0])
    t1 = min(s1.t[-1], t2_al[-1])
    if t1 <= t0:
        raise SystemExit("No hay solape temporal tras alinear. Revisa eje/tiempo.")

    m1 = (s1.t >= t0) & (s1.t <= t1)
    m2 = (t2_al >= t0) & (t2_al <= t1)
    t1c, x1c = s1.t[m1], s1.x[m1]
    t2c, x2c = t2_al[m2], s2.x[m2]

    t_common, x1_u = resample_uniform(t1c, x1c, fs_u)
    _, x2_u = resample_uniform(t2c, x2c, fs_u)

    # ── Filtrado paso-bajo ────────────────────────────────────────────────────
    lp = min(fmax, 0.45 * fs_u)
    sos_lp = lowpass_sos(fs_u, lp)
    x1_u = sosfiltfilt(sos_lp, x1_u - np.mean(x1_u))
    x2_u = sosfiltfilt(sos_lp, x2_u - np.mean(x2_u))

    # ── Tercios de octava + PSD ───────────────────────────────────────────────
    df_to_1 = compute_third_octave_rms(x1_u, fs_u, fmax).rename(columns={"rms": "rms_1"})
    df_to_2 = compute_third_octave_rms(x2_u, fs_u, fmax).rename(columns={"rms": "rms_2"})
    psd_1 = compute_psd(x1_u, fs_u, fmax)
    psd_2 = compute_psd(x2_u, fs_u, fmax)

    # ── Picos temporales ──────────────────────────────────────────────────────
    peaks_t1 = compute_peaks(x1_u, fs_u, fmax, prom=args.prom, min_dist_s=args.min_peak_dist)
    peaks_t2 = compute_peaks(x2_u, fs_u, fmax, prom=args.prom, min_dist_s=args.min_peak_dist)

    f1, P1 = psd_1
    f2, P2 = psd_2
    rms_total_1 = integrate_psd_to_rms(f1, P1, 0.5, fmax)
    rms_total_2 = integrate_psd_to_rms(f2, P2, 0.5, fmax)

    # ── FFT de amplitudes ─────────────────────────────────────────────────────
    freqs1, amp1 = compute_fft_amplitude(x1_u, fs_u, fmax)
    freqs2, amp2 = compute_fft_amplitude(x2_u, fs_u, fmax)

    peaks_f1 = find_spectral_peaks(freqs1, amp1, n_peaks=args.n_peaks)
    peaks_f2 = find_spectral_peaks(freqs2, amp2, n_peaks=args.n_peaks)

    # ── ISO 2631-1 ────────────────────────────────────────────────────────────
    xw1 = xw2 = None
    jerk1 = jerk2 = None
    iso_data = None

    if not args.no_iso:
        xw1 = apply_iso2631(x1_u, args.axis, fs_u)
        xw2 = apply_iso2631(x2_u, args.axis, fs_u)

        iso1 = iso_metrics(xw1, fs_u)
        iso2 = iso_metrics(xw2, fs_u)

        jerk1 = compute_jerk(x1_u, fs_u)
        jerk2 = compute_jerk(x2_u, fs_u)
        jm1 = jerk_metrics(x1_u, fs_u)
        jm2 = jerk_metrics(x2_u, fs_u)

        label_filter = "Wk" if args.axis == "z" else "Wd"
        iso_data = {
            "filter_used": label_filter,
            "aw_rms":      {"csv1": iso1["aw_rms"],   "csv2": iso2["aw_rms"]},
            "vdv":         {"csv1": iso1["vdv"],       "csv2": iso2["vdv"]},
            "cf":          {"csv1": iso1["cf"],        "csv2": iso2["cf"]},
            "cf_note":     "Si CF > 9: el VDV es más representativo que el RMS (ISO 2631-1 §5.3)",
            "jerk_rms_ms3":{"csv1": jm1["rms_ms3"],   "csv2": jm2["rms_ms3"]},
            "jerk_p99_ms3":{"csv1": jm1["p99_ms3"],   "csv2": jm2["p99_ms3"]},
            "jerk_max_ms3":{"csv1": jm1["max_ms3"],   "csv2": jm2["max_ms3"]},
        }

    # ── Curva de referencia ───────────────────────────────────────────────────
    ref_curve = read_reference_curve(args.ref_curve) if args.ref_curve else None

    # ── Guardar resultados ────────────────────────────────────────────────────
    os.makedirs(args.out, exist_ok=True)

    # Tercios de octava CSV
    merged = pd.merge(df_to_1, df_to_2, on="fc_hz", how="inner")
    merged["ratio_2_over_1"] = merged["rms_2"] / np.maximum(merged["rms_1"], 1e-9)
    merged.to_csv(os.path.join(args.out, f"third_octave_{args.axis}.csv"), index=False)

    # FFT amplitudes CSV
    fft_df = pd.DataFrame({
        "freq_hz": freqs1,
        "amplitude_csv1_ms2": amp1,
        "amplitude_csv2_ms2": np.interp(freqs1, freqs2, amp2),
    })
    fft_df.to_csv(os.path.join(args.out, f"fft_amplitudes_{args.axis}.csv"), index=False)

    # Picos espectrales CSV
    if not peaks_f1.empty:
        peaks_f1.to_csv(os.path.join(args.out, f"spectral_peaks_csv1_{args.axis}.csv"), index=False)
    if not peaks_f2.empty:
        peaks_f2.to_csv(os.path.join(args.out, f"spectral_peaks_csv2_{args.axis}.csv"), index=False)

    # JSON de resumen
    summary = {
        "csv1": args.csv1,
        "csv2": args.csv2,
        "axis": args.axis,
        "time_cols": {"csv1": tcol1, "csv2": tcol2},
        "acc_cols":  {"csv1": acol1, "csv2": acol2},
        "fs_med_hz": {"csv1": float(s1.fs_med), "csv2": float(s2.fs_med)},
        "dt_p99_s":  {"csv1": float(s1.dt_p99), "csv2": float(s2.dt_p99)},
        "fmax_target_hz": float(args.fmax),
        "fmax_used_hz":   float(fmax),
        "fmax_reason":    note,
        "fs_u_used_hz":   float(fs_u),
        "offset_csv2_to_csv1_s": float(offset),
        "rms_total_0.5_fmax": {"csv1": rms_total_1, "csv2": rms_total_2},
        "temporal_peaks":     {"csv1": peaks_t1,    "csv2": peaks_t2},
        "spectral_peaks_n_reported": {
            "csv1": int(len(peaks_f1)),
            "csv2": int(len(peaks_f2)),
        },
    }
    if iso_data is not None:
        summary["iso2631"] = iso_data

    with open(os.path.join(args.out, f"summary_{args.axis}.json"), "w") as fh:
        json.dump(summary, fh, indent=2)

    # ── Gráficas ──────────────────────────────────────────────────────────────
    plot_outputs(
        args.out, args.axis, t_common, x1_u, x2_u, fs_u, fmax,
        df_to_1, df_to_2, psd_1, psd_2,
        xw1=xw1, xw2=xw2,
        jerk1=jerk1, jerk2=jerk2, jerk_window_s=args.jerk_window_s,
        ref_curve=ref_curve,
    )
    plot_fft_amplitudes(args.out, args.axis, freqs1, amp1, freqs2, amp2, peaks_f1, peaks_f2)

    # ── Reporte en consola ────────────────────────────────────────────────────
    print("=== OK ===")
    print(f"Salida:   {args.out}")
    print(f"fmax:     {fmax:.2f} Hz | fs_u: {fs_u:.1f} Hz | offset csv2→csv1: {offset:+.3f} s")
    print(f"RMS total (0.5-{fmax:.1f}Hz)  csv1={rms_total_1:.4f} m/s²  |  csv2={rms_total_2:.4f} m/s²")
    print(f"Picos temporales  csv1={peaks_t1}  |  csv2={peaks_t2}")
    print(f"Picos espectrales csv1={len(peaks_f1)} | csv2={len(peaks_f2)}")

    if not peaks_f1.empty:
        print(f"\nTop picos espectrales CSV1 (eje {args.axis}):")
        print(peaks_f1.to_string(index=False))
    if not peaks_f2.empty:
        print(f"\nTop picos espectrales CSV2 (eje {args.axis}):")
        print(peaks_f2.to_string(index=False))

    if iso_data:
        lbl = iso_data["filter_used"]
        print(f"\n=== ISO 2631-1 ({lbl}) ===")
        print(f"  aw_rms   csv1={iso_data['aw_rms']['csv1']:.4f} m/s²  |  csv2={iso_data['aw_rms']['csv2']:.4f} m/s²")
        print(f"  VDV      csv1={iso_data['vdv']['csv1']:.4f} m/s^1.75  |  csv2={iso_data['vdv']['csv2']:.4f} m/s^1.75")
        print(f"  CF       csv1={iso_data['cf']['csv1']:.2f}  |  csv2={iso_data['cf']['csv2']:.2f}  (>9 → preferir VDV)")
        print(f"  Jerk RMS csv1={iso_data['jerk_rms_ms3']['csv1']:.3f} m/s³  |  csv2={iso_data['jerk_rms_ms3']['csv2']:.3f} m/s³")
        print(f"  Jerk p99 csv1={iso_data['jerk_p99_ms3']['csv1']:.3f} m/s³  |  csv2={iso_data['jerk_p99_ms3']['csv2']:.3f} m/s³")

    print("\nFicheros generados:")
    for fn in sorted(os.listdir(args.out)):
        print(f"  {fn}")


if __name__ == "__main__":
    main()