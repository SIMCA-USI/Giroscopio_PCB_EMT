import pandas as pd
import numpy as np
from scipy.signal import butter, filtfilt, correlate

def pick_time_col(df):
    # Preferencia: reloj de PC > tiempo_s > t_ms
    candidates = ["pc_time_s", "tiempo_s", "t_ms"]
    for c in candidates:
        if c in df.columns:
            t = df[c].to_numpy()
            m = np.isfinite(t)
            if m.sum() < 10:
                continue
            tt = t[m]
            # monotónico "casi siempre"
            if np.mean(np.diff(tt) > 0) > 0.95:
                return c
    raise ValueError("No encuentro una columna de tiempo válida.")

def get_time_seconds(df, time_col):
    t = df[time_col].to_numpy().astype(float)
    m = np.isfinite(t)
    t = t[m]
    if time_col == "t_ms":
        t = t / 1000.0
    # tiempo relativo desde 0
    t = t - t[0]
    return t, m

def pick_signal_col(df, preferred="az_ms2"):
    if preferred in df.columns:
        return preferred
    # fallback razonable
    for c in ["az", "acc_z", "accel_z", "a_z", "z_ms2"]:
        if c in df.columns:
            return c
    # último recurso: primera columna que parezca aceleración en m/s²
    for c in df.columns:
        if c.endswith("_ms2"):
            return c
    raise ValueError("No encuentro columna de aceleración (e.g. az_ms2).")

def estimate_fs(t):
    dt = np.diff(t)
    dt = dt[np.isfinite(dt) & (dt > 0)]
    fs_med = 1.0 / np.median(dt)
    return fs_med, dt

def bandpass(x, fs, hp=0.5, lp=10.0, order=4):
    x = x - np.nanmean(x)
    b, a = butter(order, [hp/(fs/2), lp/(fs/2)], btype="band")
    return filtfilt(b, a, x)

def resample_uniform(t, x, fs_u):
    tu = np.arange(t[0], t[-1], 1/fs_u)
    xu = np.interp(tu, t, x)
    return tu, xu

def sync_and_resample(path_a, path_b, preferred_sig="az_ms2", fs_u=200.0, max_lag_s=5.0):
    A = pd.read_csv(path_a)
    B = pd.read_csv(path_b)

    ta_col = pick_time_col(A)
    tb_col = pick_time_col(B)

    sa_col = pick_signal_col(A, preferred_sig)
    sb_col = pick_signal_col(B, preferred_sig)

    ta, ma = get_time_seconds(A, ta_col)
    tb, mb = get_time_seconds(B, tb_col)

    xa = A.loc[ma, sa_col].to_numpy().astype(float)
    xb = B.loc[mb, sb_col].to_numpy().astype(float)

    fsa, dta = estimate_fs(ta)
    fsb, dtb = estimate_fs(tb)

    print(f"[A] time={ta_col} sig={sa_col} fs_med={fsa:.1f}Hz dt_p99={np.percentile(dta,99):.4f}s")
    print(f"[B] time={tb_col} sig={sb_col} fs_med={fsb:.1f}Hz dt_p99={np.percentile(dtb,99):.4f}s")

    # Chequeo duro para 80Hz
    fs_p01_a = 1.0 / np.percentile(dta, 99)  # "casi peor caso"
    fs_p01_b = 1.0 / np.percentile(dtb, 99)
    fs_min_like = min(fs_p01_a, fs_p01_b)
    fmax_safe = fs_min_like / 2.5
    print(f"f_max recomendado (margen) ~ {fmax_safe:.1f} Hz (si quieres ir conservador)")

    # Re-muestreo provisional para correlación
    fs_sync = min(fsa, fsb, 200.0)
    ta_u, xa_u = resample_uniform(ta, xa, fs_sync)
    tb_u, xb_u = resample_uniform(tb, xb, fs_sync)

    xa_f = bandpass(xa_u, fs_sync, 0.5, 10.0)
    xb_f = bandpass(xb_u, fs_sync, 0.5, 10.0)

    max_lag = int(max_lag_s * fs_sync)
    c = correlate(xa_f, xb_f, mode="full")
    lags = np.arange(-len(xb_f)+1, len(xa_f))

    mid = len(c)//2
    c_win = c[mid-max_lag:mid+max_lag+1]
    l_win = lags[mid-max_lag:mid+max_lag+1]

    lag = l_win[np.argmax(c_win)]
    offset_s = lag / fs_sync
    print(f"offset estimado (A respecto B) = {offset_s:+.3f} s")

    # aplica offset a B y recorta solape
    tb_al = tb + offset_s
    t0 = max(ta[0], tb_al[0])
    t1 = min(ta[-1], tb_al[-1])
    if t1 <= t0:
        raise ValueError("No hay solape temporal tras alinear. Revisa tiempo/eje/ruido.")

    ma2 = (ta >= t0) & (ta <= t1)
    mb2 = (tb_al >= t0) & (tb_al <= t1)

    ta_c, xa_c = ta[ma2], xa[ma2]
    tb_c, xb_c = tb_al[mb2], xb[mb2]

    # Re-muestreo final uniforme
    tu, xa_fin = resample_uniform(ta_c, xa_c, fs_u)
    _,  xb_fin = resample_uniform(tb_c, xb_c, fs_u)

    return tu, xa_fin, xb_fin, fs_u

# USO con tus rutas:
t, z_asiento, z_suelo, fsu = sync_and_resample(
    "/home/adrian/Giroscopio_PCB_EMT/Cocheras_VIernes_13/prueba_Cocheras_OUT_1_Asiento_ttyUSB0_20260213_103433_fixed.csv",
    "/home/adrian/Giroscopio_PCB_EMT/Cocheras_VIernes_13/prueba_Cocheras_OUT_1_Suelo_ttyUSB1_20260213_103433_fixed.csv",
    preferred_sig="az_ms2",
    fs_u=200.0
)
print("OK:", len(t), "muestras a", fsu, "Hz")