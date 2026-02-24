#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import time
from pathlib import Path
import re

import pandas as pd
import folium


def parse_range(s: str):
    """
    Parsea un rango tipo "0:5" => (0.0, 5.0)
    """
    a, b = s.split(":")
    return float(a), float(b)


def speed_to_color(v: float, green_rng, yellow_rng, red_rng):
    """
    Devuelve un color según el rango donde cae la velocidad.
    - Verde:   [g0, g1)
    - Amarillo:[y0, y1)
    - Rojo:    [r0, r1] (incluye el máximo)
    Si cae fuera de rangos:
      - por debajo -> verde
      - por encima -> rojo
    """
    g0, g1 = green_rng
    y0, y1 = yellow_rng
    r0, r1 = red_rng

    if v is None:
        return "#808080"  # gris si no hay dato

    if g0 <= v < g1:
        return "green"
    if y0 <= v < y1:
        return "yellow"
    if r0 <= v <= r1:
        return "red"

    # fuera de rangos
    if v < min(g0, y0, r0):
        return "green"
    return "red"


def detect_sort_column(df: pd.DataFrame):
    """
    Intenta detectar una columna temporal para ordenar puntos (si existe).
    """
    candidates = ["timestamp", "time", "datetime", "date", "fecha", "hora"]
    norm = {c: c.strip().lower() for c in df.columns}
    for c in df.columns:
        if any(k in norm[c] for k in candidates):
            return c
    return None


def coerce_decimal_commas(series: pd.Series) -> pd.Series:
    """
    Si vienen números como texto con coma decimal (ej: "12,3"), los convierte a "12.3".
    No afecta si ya son números.
    """
    if series.dtype == object:
        return series.astype(str).str.replace(",", ".", regex=False)
    return series


def build_folium_map_with_speed(
    df: pd.DataFrame,
    lat_col: str,
    lon_col: str,
    speed_col: str,
    tiles: str,
    green_rng,
    yellow_rng,
    red_rng,
):
    # Selección y limpieza
    d = df[[lat_col, lon_col, speed_col]].copy()

    d[lat_col] = pd.to_numeric(coerce_decimal_commas(d[lat_col]), errors="coerce")
    d[lon_col] = pd.to_numeric(coerce_decimal_commas(d[lon_col]), errors="coerce")
    d[speed_col] = pd.to_numeric(coerce_decimal_commas(d[speed_col]), errors="coerce")

    d = d.dropna(subset=[lat_col, lon_col])

    if len(d) < 2:
        raise ValueError("No hay suficientes puntos válidos (mínimo 2) para dibujar una ruta.")

    center = [float(d[lat_col].mean()), float(d[lon_col].mean())]
    m = folium.Map(location=center, zoom_start=13, tiles=tiles, control_scale=True)

    coords = list(zip(d[lat_col].tolist(), d[lon_col].tolist()))
    speeds = d[speed_col].tolist()

    # Ruta por segmentos coloreados
    for i in range(len(coords) - 1):
        p0 = coords[i]
        p1 = coords[i + 1]

        v0 = speeds[i]
        v1 = speeds[i + 1]

        v = None
        if pd.notna(v0) and pd.notna(v1):
            v = (float(v0) + float(v1)) / 2.0
        elif pd.notna(v0):
            v = float(v0)
        elif pd.notna(v1):
            v = float(v1)

        color = speed_to_color(v, green_rng, yellow_rng, red_rng)

        folium.PolyLine(
            locations=[p0, p1],
            weight=6,
            opacity=0.9,
            color=color,
        ).add_to(m)

    # Marcadores inicio/fin
    folium.Marker(coords[0], tooltip="Inicio", icon=folium.Icon(color="green")).add_to(m)
    folium.Marker(coords[-1], tooltip="Fin", icon=folium.Icon(color="red")).add_to(m)

    m.fit_bounds(coords)
    return m, coords


def html_to_png_selenium(html_path: str, png_path: str, map_div_id: str, window_size=(1400, 900), wait_s=3.0):
    """
    Convierte el HTML de Folium a PNG con Selenium (Chrome/Chromium headless).
    Requiere: selenium + webdriver-manager + chrome/chromium instalado.
    """
    from selenium import webdriver
    from selenium.webdriver.common.by import By
    from selenium.webdriver.chrome.options import Options
    from webdriver_manager.chrome import ChromeDriverManager
    from selenium.webdriver.chrome.service import Service

    chrome_options = Options()
    chrome_options.add_argument("--headless=new")
    chrome_options.add_argument("--disable-gpu")
    chrome_options.add_argument("--no-sandbox")
    chrome_options.add_argument("--disable-dev-shm-usage")
    chrome_options.add_argument(f"--window-size={window_size[0]},{window_size[1]}")

    service = Service(ChromeDriverManager().install())
    driver = webdriver.Chrome(service=service, options=chrome_options)

    try:
        file_url = Path(html_path).absolute().as_uri()
        driver.get(file_url)

        # Espera a que carguen tiles/canvas
        time.sleep(wait_s)

        map_el = driver.find_element(By.ID, map_div_id)
        map_el.screenshot(png_path)
    finally:
        driver.quit()


def main():
    parser = argparse.ArgumentParser(
        description="Lee un CSV con gps_lat/gps_lon y velocidad, pinta ruta por colores y genera HTML y (opcional) PNG."
    )
    parser.add_argument("--csv", required=True, help="Ruta al CSV de entrada")
    parser.add_argument("--out", default="ruta.png", help="Salida PNG (por defecto: ruta.png)")
    parser.add_argument("--html", default="ruta.html", help="Salida HTML (por defecto: ruta.html)")
    parser.add_argument("--tiles", default="OpenStreetMap", help="Tiles Folium (por defecto: OpenStreetMap)")

    # Columnas (por defecto según tu CSV)
    parser.add_argument("--lat-col", default="gps_lat", help="Nombre columna latitud (por defecto: gps_lat)")
    parser.add_argument("--lon-col", default="gps_lon", help="Nombre columna longitud (por defecto: gps_lon)")
    parser.add_argument("--speed-col", default="velocidad", help="Nombre columna velocidad (por defecto: velocidad)")

    # Rangos de color (min:max)
    parser.add_argument("--green", default="0:10", help="Rango verde (min:max), ej: 0:10")
    parser.add_argument("--yellow", default="10:30", help="Rango amarillo (min:max), ej: 10:30")
    parser.add_argument("--red", default="30:9999", help="Rango rojo (min:max), ej: 30:9999")

    # Ordenación
    parser.add_argument("--no-sort", action="store_true", help="No ordenar por columna temporal aunque exista")

    # PNG opcional
    parser.add_argument("--no-png", action="store_true", help="No generar PNG (solo HTML)")
    parser.add_argument("--wait", type=float, default=3.0, help="Segundos de espera antes de capturar PNG (default 3.0)")

    args = parser.parse_args()

    df = pd.read_csv(args.csv)

    # Ordenar si hay columna temporal (a menos que se desactive)
    if not args.no_sort:
        sort_col = detect_sort_column(df)
        if sort_col is not None:
            try:
                df[sort_col] = pd.to_datetime(df[sort_col], errors="ignore")
            except Exception:
                pass
            df = df.sort_values(by=sort_col)
    else:
        sort_col = None

    # Rangos
    green_rng = parse_range(args.green)
    yellow_rng = parse_range(args.yellow)
    red_rng = parse_range(args.red)

    # Construir mapa
    m, coords = build_folium_map_with_speed(
        df=df,
        lat_col=args.lat_col,
        lon_col=args.lon_col,
        speed_col=args.speed_col,
        tiles=args.tiles,
        green_rng=green_rng,
        yellow_rng=yellow_rng,
        red_rng=red_rng,
    )

    # Guardar HTML
    m.save(args.html)

    # Folium usa un id tipo "map_xxxxx" que coincide con el div id
    map_div_id = m.get_name()

    print(f"OK ✅ HTML: {args.html}")
    print(f"Puntos usados: {len(coords)} | columnas: lat='{args.lat_col}', lon='{args.lon_col}', vel='{args.speed_col}' | sort='{sort_col}'")
    print(f"Rangos: verde={green_rng} amarillo={yellow_rng} rojo={red_rng}")

    # PNG opcional
    if not args.no_png:
        try:
            html_to_png_selenium(args.html, args.out, map_div_id=map_div_id, wait_s=args.wait)
            print(f"OK ✅ PNG:  {args.out}")
        except Exception as e:
            print("⚠️ No se pudo generar PNG con Selenium.")
            print("   (El HTML está bien generado; abre el HTML en el navegador si lo necesitas.)")
            print(f"   Error: {e}")


if __name__ == "__main__":
    main()
