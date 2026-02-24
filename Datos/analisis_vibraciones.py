#!/usr/bin/env python3
import argparse
import json
import os
from dataclasses import dataclass
import numpy as np
import pandas as pd
from scipy.signal import butter, sosfiltfilt, welch, correlate, find_peaks
import matplotlib.pyplot as plt


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

def bandpass_sos(fs: float, hp: float, lp: float, order: int = 4):
    hp = max(hp, 1e-6)
    lp = min(lp, 0.999 * (fs / 2.0))
    if lp <= hp:
        raise ValueError(f"Bandpass inválido: hp={hp}, lp={lp}, fs={fs}")
    return butter(order, [hp/(fs/2.0), lp/(fs/2.0)], btype="band", output="sos")

def lowpass_sos(fs: float, lp: float, order: int = 6):
    lp = min(lp, 0.999 * (fs / 2.0))
    return butter(order, lp/(fs/2.0), btype="low", output="sos")

def resample_uniform(t: np.ndarray, x: np.ndarray, fs_u: float):
    tu = np.arange(t[0], t[-1], 1.0/fs_u)
    xu = np.interp(tu, t, x)
    return tu, xu

def robust_offset_by_xcorr(xa, xb, fs, max_lag_s=5.0):
    sos = bandpass_sos(fs, 0.5, min(10.0, 0.45*fs))
    a = sosfiltfilt(sos, xa - np.mean(xa))
    b = sosfiltfilt(sos, xb - np.mean(xb))

    max_lag = int(max_lag_s * fs)
    c = correlate(a, b, mode="full")
    lags = np.arange(-len(b)+1, len(a))

    mid = len(c)//2
    c_win = c[mid-max_lag:mid+max_lag+1]
    l_win = lags[mid-max_lag:mid+max_lag+1]

    lag = int(l_win[np.argmax(c_win)])
    return lag / fs

def third_octave_centers(fmin=0.5, fmax=80.0):
    fc = []
    f = 1.0
    while f > fmin:
        f /= 2**(1/3)
        if f < fmin:
            break
    f = max(f, fmin)
    while f <= fmax * 1.001:
        fc.append(f)
        f *= 2**(1/3)
    return np.array(fc, dtype=float)

def third_octave_edges(fc: np.ndarray):
    k = 2**(1/6)
    lo = fc / k
    hi = fc * k
    return lo, hi

def rms(x):
    return float(np.sqrt(np.mean(np.square(x))))

def integrate_psd_to_rms(f, Pxx, f1, f2):
    m = (f >= f1) & (f < f2)
    if not np.any(m):
        return np.nan
    var = np.trapz(Pxx[m], f[m])
    return float(np.sqrt(max(var, 0.0)))

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
    t = t[ok]; x = x[ok]
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
    nper = int(min(len(xu), max(1024, fs_u*8)))
    if nper % 2 == 1:
        nper -= 1
    f, P = welch(xu - np.mean(xu), fs=fs_u, nperseg=nper, noverlap=nper//2, window="hann", detrend=False)
    m = (f >= 0.5) & (f <= fmax)
    return f[m], P[m]

def compute_peaks(xu, fs_u, fmax, prom=1.0, min_dist_s=0.3):
    sos = bandpass_sos(fs_u, 0.5, min(fmax, 0.45*fs_u))
    y = sosfiltfilt(sos, xu - np.mean(xu))
    peaks, _ = find_peaks(np.abs(y), prominence=prom, distance=int(min_dist_s*fs_u))
    v = np.abs(y)[peaks]
    if len(v) == 0:
        return {"n": 0}
    return {
        "n": int(len(v)),
        "p95": float(np.percentile(v, 95)),
        "p99": float(np.percentile(v, 99)),
        "max": float(np.max(v)),
        "prom_used": float(prom),
        "min_dist_s": float(min_dist_s),
    }

def read_reference_curve(path: str):
    df = pd.read_csv(path)
    if "fc_hz" not in df.columns or "ref" not in df.columns:
        raise ValueError("Curva de referencia debe tener columnas: fc_hz, ref")
    return df[["fc_hz", "ref"]].copy()

def plot_outputs(outdir, axis, t_common, x1_u, x2_u, fs_u, fmax,
                 df_to_1, df_to_2, psd_1, psd_2, ref_curve=None):
    os.makedirs(outdir, exist_ok=True)

    # 1) Tiempo
    plt.figure()
    span = 60.0
    tmax = t_common[-1]
    tmin = max(0.0, tmax - span)
    m = (t_common >= tmin) & (t_common <= tmax)
    plt.plot(t_common[m], x1_u[m], label="CSV1")
    plt.plot(t_common[m], x2_u[m], label="CSV2")
    plt.xlabel("Tiempo (s)")
    plt.ylabel(f"a{axis} (m/s²)")
    plt.title(f"Señal temporal (últimos {span:.0f}s) | fs_u={fs_u:.1f}Hz | fmax={fmax:.1f}Hz")
    plt.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(outdir, f"time_{axis}.png"), dpi=160)
    plt.close()

    # 2) PSD
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

    # 3) Tercios de octava
    merged = pd.merge(df_to_1, df_to_2, on="fc_hz", suffixes=("_1", "_2"))
    plt.figure(figsize=(10,4.5))
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

    # 4) Ratio CSV2/CSV1
    plt.figure(figsize=(10,4.5))
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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv1", required=True, help="Primer CSV")
    ap.add_argument("--csv2", required=True, help="Segundo CSV")
    ap.add_argument("--axis", default="z", choices=["x","y","z"], help="Eje a analizar (vertical normalmente z)")
    ap.add_argument("--fmax", type=float, default=80.0, help="Objetivo f_max (se recorta a lo que soporte tu muestreo)")
    ap.add_argument("--out", default="out_vibraciones", help="Directorio de salida")
    ap.add_argument("--ref_curve", default=None, help="CSV opcional con columnas: fc_hz,ref")
    ap.add_argument("--prom", type=float, default=1.0, help="Prominence para picos (m/s²)")
    ap.add_argument("--min_peak_dist", type=float, default=0.30, help="Distancia mínima entre picos (s)")
    args = ap.parse_args()

    _, s1, tcol1, acol1 = load_signal(args.csv1, args.axis)
    _, s2, tcol2, acol2 = load_signal(args.csv2, args.axis)

    fmax, note = choose_fmax(s1.fs_med, s1.dt_p99, s2.fs_med, s2.dt_p99, args.fmax)

    fs_u = float(max(100.0, 5.0*fmax))
    fs_u_cap = 1.2 * min(s1.fs_med, s2.fs_med) if np.isfinite(s1.fs_med) and np.isfinite(s2.fs_med) else fs_u
    fs_u = float(min(fs_u, max(100.0, fs_u_cap)))

    # solape inicial
    t0 = max(s1.t[0], s2.t[0])
    t1 = min(s1.t[-1], s2.t[-1])
    if t1 <= t0:
        raise SystemExit("No hay solape temporal entre CSVs (en el eje de tiempo elegido).")

    m1 = (s1.t >= t0) & (s1.t <= t1)
    m2 = (s2.t >= t0) & (s2.t <= t1)
    t1c, x1c = s1.t[m1], s1.x[m1]
    t2c, x2c = s2.t[m2], s2.x[m2]

    fs_sync = float(min(s1.fs_med, s2.fs_med, 120.0))
    tt1, xx1 = resample_uniform(t1c, x1c, fs_sync)
    tt2, xx2 = resample_uniform(t2c, x2c, fs_sync)
    offset = robust_offset_by_xcorr(xx1, xx2, fs_sync, max_lag_s=5.0)

    # alineamos CSV2 a CSV1
    t2_al = s2.t + offset

    # solape final
    t0 = max(s1.t[0], t2_al[0])
    t1 = min(s1.t[-1], t2_al[-1])
    if t1 <= t0:
        raise SystemExit("No hay solape temporal tras alinear. Revisa eje/tiempo.")

    m1 = (s1.t >= t0) & (s1.t <= t1)
    m2 = (t2_al >= t0) & (t2_al <= t1)
    t1c, x1c = s1.t[m1], s1.x[m1]
    t2c, x2c = t2_al[m2], s2.x[m2]

    t_common, x1_u = resample_uniform(t1c, x1c, fs_u)
    _,        x2_u = resample_uniform(t2c, x2c, fs_u)

    lp = min(fmax, 0.45*fs_u)
    sos_lp = lowpass_sos(fs_u, lp)
    x1_u = sosfiltfilt(sos_lp, x1_u - np.mean(x1_u))
    x2_u = sosfiltfilt(sos_lp, x2_u - np.mean(x2_u))

    df_to_1 = compute_third_octave_rms(x1_u, fs_u, fmax).rename(columns={"rms":"rms_1"})
    df_to_2 = compute_third_octave_rms(x2_u, fs_u, fmax).rename(columns={"rms":"rms_2"})

    psd_1 = compute_psd(x1_u, fs_u, fmax)
    psd_2 = compute_psd(x2_u, fs_u, fmax)

    peaks_1 = compute_peaks(x1_u, fs_u, fmax, prom=args.prom, min_dist_s=args.min_peak_dist)
    peaks_2 = compute_peaks(x2_u, fs_u, fmax, prom=args.prom, min_dist_s=args.min_peak_dist)

    f1, P1 = psd_1
    f2, P2 = psd_2
    rms_total_1 = integrate_psd_to_rms(f1, P1, 0.5, fmax)
    rms_total_2 = integrate_psd_to_rms(f2, P2, 0.5, fmax)

    ref_curve = read_reference_curve(args.ref_curve) if args.ref_curve else None

    os.makedirs(args.out, exist_ok=True)
    merged = pd.merge(df_to_1, df_to_2, on="fc_hz", how="inner")
    merged["ratio_2_over_1"] = merged["rms_2"] / np.maximum(merged["rms_1"], 1e-9)
    merged.to_csv(os.path.join(args.out, f"third_octave_{args.axis}.csv"), index=False)

    summary = {
        "csv1": args.csv1,
        "csv2": args.csv2,
        "axis": args.axis,
        "time_cols": {"csv1": tcol1, "csv2": tcol2},
        "acc_cols": {"csv1": acol1, "csv2": acol2},
        "fs_med_hz": {"csv1": float(s1.fs_med), "csv2": float(s2.fs_med)},
        "dt_p99_s": {"csv1": float(s1.dt_p99), "csv2": float(s2.dt_p99)},
        "fmax_target_hz": float(args.fmax),
        "fmax_used_hz": float(fmax),
        "fmax_reason": note,
        "fs_u_used_hz": float(fs_u),
        "offset_csv2_to_csv1_s": float(offset),
        "rms_total_0.5_fmax": {"csv1": rms_total_1, "csv2": rms_total_2},
        "peaks": {"csv1": peaks_1, "csv2": peaks_2},
    }
    with open(os.path.join(args.out, f"summary_{args.axis}.json"), "w") as f:
        json.dump(summary, f, indent=2)

    plot_outputs(args.out, args.axis, t_common, x1_u, x2_u, fs_u, fmax,
                 df_to_1, df_to_2, psd_1, psd_2, ref_curve=ref_curve)

    print("=== OK ===")
    print(f"Salida: {args.out}")
    print(f"fmax usado: {fmax:.2f} Hz | fs_u: {fs_u:.1f} Hz | offset csv2->csv1: {offset:+.3f} s")
    print(f"RMS total (0.5-{fmax:.1f}Hz) csv1={rms_total_1:.3f} m/s² | csv2={rms_total_2:.3f} m/s²")
    print(f"Picos csv1: {peaks_1} | Picos csv2: {peaks_2}")


if __name__ == "__main__":
    main()