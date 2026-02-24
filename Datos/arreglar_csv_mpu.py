#!/usr/bin/env python3
import argparse
import os
import pandas as pd
import numpy as np

G = 9.80665

AX_CANDS = ["ax_ms2", "ay_ms2", "az_ms2"]
AG_CANDS = ["ax_g", "ay_g", "az_g"]
GX_CANDS = ["gx_dps", "gy_dps", "gz_dps"]

def _norm_median(df, cols):
    arr = df[cols].to_numpy(dtype=float, copy=True)
    n = np.linalg.norm(arr, axis=1)
    n = n[np.isfinite(n)]
    if n.size == 0:
        return None
    return float(np.median(n))

def _norm_max(df, cols):
    arr = df[cols].to_numpy(dtype=float, copy=True)
    n = np.linalg.norm(arr, axis=1)
    n = n[np.isfinite(n)]
    if n.size == 0:
        return None
    return float(np.max(n))

def detect_acc_mode(df):
    """
    Devuelve (mode, cols):
      - ("g", ["ax_g","ay_g","az_g"])         -> convertir a m/s^2 multiplicando por G
      - ("ms2", ["ax_ms2","ay_ms2","az_ms2"]) -> ya está en m/s^2
      - ("ms2_div4096", ["ax_ms2","ay_ms2","az_ms2"]) -> estaba aplastado /4096, corregir *4096
      - ("unknown", None)
    """
    if all(c in df.columns for c in AG_CANDS):
        return ("g", AG_CANDS)

    if all(c in df.columns for c in AX_CANDS):
        med = _norm_median(df, AX_CANDS)
        mx = _norm_max(df, AX_CANDS)
        if med is None or mx is None:
            return ("ms2", AX_CANDS)

        # Heurística:
        # - si está por ~9.8 ya es m/s^2
        # - si está por ~1 es g mal etiquetado como ms2
        # - si está por ~0.002-0.02 suele ser m/s^2 aplastado por /4096
        if 6.0 <= med <= 14.0:
            return ("ms2", AX_CANDS)
        if 0.2 <= med <= 3.0:
            return ("g_as_ms2", AX_CANDS)  # realmente g, pero lo llamaron ms2
        if med < 0.05 and mx < 0.5:
            return ("ms2_div4096", AX_CANDS)
        return ("ms2", AX_CANDS)

    return ("unknown", None)

def fix_acc(df, mode, cols, add_gravity_z=False):
    out = df.copy()

    if mode == "g":
        out["ax_ms2"] = out[cols[0]].astype(float) * G
        out["ay_ms2"] = out[cols[1]].astype(float) * G
        out["az_ms2"] = out[cols[2]].astype(float) * G

    elif mode == "g_as_ms2":
        # Estaban guardados como ax_ms2 pero realmente eran g
        out["ax_ms2"] = out[cols[0]].astype(float) * G
        out["ay_ms2"] = out[cols[1]].astype(float) * G
        out["az_ms2"] = out[cols[2]].astype(float) * G

    elif mode == "ms2_div4096":
        out["ax_ms2"] = out[cols[0]].astype(float) * 4096.0
        out["ay_ms2"] = out[cols[1]].astype(float) * 4096.0
        out["az_ms2"] = out[cols[2]].astype(float) * 4096.0

    elif mode == "ms2":
        # Ya está en ms2: asegurar nombres estándar
        out["ax_ms2"] = out[cols[0]].astype(float)
        out["ay_ms2"] = out[cols[1]].astype(float)
        out["az_ms2"] = out[cols[2]].astype(float)

    else:
        raise ValueError("No se detectan columnas de aceleración (ax_g/ax_ms2...).")

    if add_gravity_z:
        # ATENCIÓN: esto es una aproximación (asume Z alineado con gravedad en reposo)
        out["az_ms2"] = out["az_ms2"] + G

    return out

def fix_gyro(df, multiply_131=False):
    out = df.copy()
    if all(c in out.columns for c in GX_CANDS) and multiply_131:
        # Solo si tus datos quedaron como dps/131 por un error de escalado.
        out["gx_dps"] = out["gx_dps"].astype(float) * 131.0
        out["gy_dps"] = out["gy_dps"].astype(float) * 131.0
        out["gz_dps"] = out["gz_dps"].astype(float) * 131.0
    return out

def main():
    ap = argparse.ArgumentParser(description="Arregla CSV de MPU6050 (auto-detección de unidades).")
    ap.add_argument("csv_path", help="Ruta al CSV de entrada")
    ap.add_argument("-o", "--out", default=None, help="Ruta del CSV de salida (por defecto: *_fixed.csv)")
    ap.add_argument("--add-gravity-z", action="store_true",
                    help="Suma +9.80665 m/s^2 a Z (solo si QUIERES reconstrucción aproximada de gravedad).")
    ap.add_argument("--gyro-mul-131", action="store_true",
                    help="Multiplica gx/gy/gz por 131 (solo si quedaron divididos por error).")
    args = ap.parse_args()

    in_path = args.csv_path
    if not os.path.exists(in_path):
        raise SystemExit(f"No existe: {in_path}")

    df = pd.read_csv(in_path)

    mode, cols = detect_acc_mode(df)
    df2 = fix_acc(df, mode, cols, add_gravity_z=args.add_gravity_z)
    df2 = fix_gyro(df2, multiply_131=args.gyro_mul_131)

    out_path = args.out
    if out_path is None:
        base, ext = os.path.splitext(in_path)
        out_path = f"{base}_fixed.csv"

    df2.to_csv(out_path, index=False)

    print(f"Entrada: {in_path}")
    print(f"Salida : {out_path}")
    print(f"Accel detectado: {mode}")
    if args.add_gravity_z:
        print("Nota: se ha sumado +G en Z (aproximación).")
    if args.gyro_mul_131:
        print("Nota: gyro multiplicado por 131 (solo válido si estaba escalado mal).")

if __name__ == "__main__":
    main()