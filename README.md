# Codigo_Vibraciones

Sistema de medición de vibraciones en vehículos EMT usando una PCB con ESP32-C6 y sensor MPU6050.

---

## Estructura del proyecto

```
Codigo_Vibraciones/
├── README.md
├── Datos/                        ← Scripts Python (ejecutar en el PC)
│   ├── logger_plot.py            ← v1: logger básico, 1 dispositivo
│   ├── logger_plot2.py           ← v2: multi-dispositivo, threads
│   ├── logger_plot3.py           ← v3: añade GPS (NMEA) y baseline automático
│   ├── logger_plot4.py           ← v4: versión completa (producción)
│   ├── graficas.py               ← Visor offline de CSVs grabados
│   ├── ruta.py                   ← Genera mapa HTML coloreado por velocidad
│   ├── correlacion.py            ← Sincroniza 2 sensores por correlación cruzada
│   ├── analisis_vibraciones.py   ← Análisis frecuencial (PSD, tercios de octava)
│   └── arreglar_csv_mpu.py       ← Corrector de unidades mal escaladas en CSV
└── vibraciones_EMT/              ← Firmware PlatformIO (para la PCB)
    ├── platformio.ini
    └── src/
        └── main.cpp
```

---

## Hardware

| Componente | Detalle |
|---|---|
| Microcontrolador | ESP32-C6 (RISC-V) |
| Sensor IMU | MPU6050 — acelerómetro ±8g + giroscopio ±250°/s |
| Comunicación con PC | USB Serie a **460800 baudios** |
| I2C | SDA=21, SCL=22, 400 kHz |

---

## Firmware (`vibraciones_EMT/src/main.cpp`)

El firmware mide a **80 Hz** (configurable) y emite por serie una línea CSV por muestra:

```
t_ms, ax_ms2, ay_ms2, az_ms2, gx_dps, gy_dps, gz_dps
```

Al arrancar, emite también:
```
Ubicacion:<nombre>
```

El nombre de ubicación (ej: `"Asiento"`, `"Suelo"`) se configura en `platformio.ini`:

```ini
[env:esp32-c6-devkitm-1]
build_flags = -DUBICACION=\"Asiento\" -DSAMPLE_HZ=80
```

### Comandos serie disponibles

| Comando | Respuesta |
|---|---|
| `WHO\n` | `Ubicacion:<nombre>` |
| `SYNC\n` | `SYNC_OK` |

---

## Scripts Python (`Datos/`)

### `logger_plot4.py` — Logger principal (v4, usar éste)

Graba datos de uno o varios ESP32 simultáneamente, con GPS opcional.

**Configurar antes de ejecutar** (parte superior del fichero):

```python
OUTPUT_DIR = "/ruta/donde/guardar"
BASE_NAME  = "Nombre_de_la_sesion"
SAMPLE_HZ  = 30        # Hz del plot visual
TARGET_LOG_HZ = 30.0   # Hz de guardado en CSV
```

**Columnas del CSV generado:**

```
ubicacion | t_ms | pc_time_s | tiempo_s | ax_ms2 | ay_ms2 | az_ms2 |
gx_dps | gy_dps | gz_dps | gps_lat | gps_lon | gps_alt_m | gps_sats |
gps_fix | gps_hdop | velocidad | sync_time_s | pc_recv_s | segment_id
```

**Ejecutar:**
```bash
python Datos/logger_plot4.py
```

---

### `graficas.py` — Visor offline

Lee uno o varios CSVs ya grabados y muestra una interfaz interactiva con sliders para navegar por la grabación.

**Configurar:**
```python
CSV_PATHS = ["ruta/al/archivo.csv"]
OUTPUT_DIR = "ruta/salida"
```

```bash
python Datos/graficas.py
```

---

### `ruta.py` — Mapa de ruta por velocidad

Genera un HTML interactivo (Folium) con la ruta pintada en colores según velocidad GPS.

```bash
python Datos/ruta.py --csv grabacion.csv --html ruta.html --no-png
```

Rangos de color por defecto: verde 0-10 km/h, amarillo 10-30, rojo >30.

---

### `analisis_vibraciones.py` — Análisis frecuencial completo (v2)

Compara **dos CSVs** (ej: asiento vs suelo del conductor). Complementa a `logger_plot4.py`
realizando en post-proceso el análisis normativo completo que no puede hacerse en tiempo real.

#### Uso básico

```bash
python Datos/analisis_vibraciones.py \
  --csv1 asiento.csv \
  --csv2 suelo.csv \
  --axis z \
  --fmax 80 \
  --out resultados/
```

#### Argumentos completos

| Argumento | Por defecto | Descripción |
|---|---|---|
| `--csv1` | *(obligatorio)* | Primer CSV (ej: asiento del conductor) |
| `--csv2` | *(obligatorio)* | Segundo CSV (ej: suelo de la cabina) |
| `--axis` | `z` | Eje a analizar: `x`, `y` o `z` (vertical = `z`) |
| `--fmax` | `80.0` | Frecuencia máxima de análisis en Hz |
| `--out` | `out_vibraciones` | Directorio de salida |
| `--ref_curve` | — | CSV con curva de referencia (`fc_hz`, `ref`) |
| `--prom` | `1.0` | Prominencia mínima de picos temporales (m/s²) |
| `--min_peak_dist` | `0.30` | Distancia mínima entre picos temporales (s) |
| `--no_iso` | — | Desactiva el análisis ISO 2631-1 |
| `--jerk_window_s` | `1.0` | Ventana RMS deslizante del jerk (s) |
| `--n_peaks` | `10` | Nº máximo de picos espectrales a reportar |

#### Ficheros generados en `--out`

| Fichero | Contenido |
|---|---|
| `time_{eje}.png` | Señal temporal (últimos 60s), ambos CSVs superpuestos |
| `psd_{eje}.png` | PSD Welch — densidad espectral de potencia (m²/s⁴/Hz) |
| `third_octave_{eje}.png` | RMS por bandas de 1/3 de octava (0.5–80 Hz) |
| `ratio_{eje}.png` | Ratio CSV2/CSV1 por banda (transmisibilidad / SEAT) |
| `time_weighted_{eje}.png` | Señal ponderada ISO 2631-1 (Wk o Wd), ambos CSVs |
| `vdv_{eje}.png` | VDV acumulado en el tiempo para ambos CSVs |
| `jerk_{eje}.png` | Jerk (brusquedad) — RMS deslizante de 1s |
| `fft_amplitudes_{eje}.png` | Espectro FFT de amplitudes con picos marcados |
| `third_octave_{eje}.csv` | Tabla de RMS por banda + ratio CSV2/CSV1 |
| `fft_amplitudes_{eje}.csv` | Espectro completo: `freq_hz`, `amplitude_csv1_ms2`, `amplitude_csv2_ms2` |
| `spectral_peaks_csv1_{eje}.csv` | Picos dominantes CSV1: frecuencia, amplitud, prominencia |
| `spectral_peaks_csv2_{eje}.csv` | Picos dominantes CSV2: frecuencia, amplitud, prominencia |
| `summary_{eje}.json` | Resumen completo: fs, offset, RMS total, picos, ISO 2631 |

#### Métricas ISO 2631-1 calculadas (sección `iso2631` del JSON)

| Métrica | Descripción |
|---|---|
| `aw_rms` | RMS de la señal ponderada Wk/Wd (m/s²) |
| `vdv` | Vibration Dose Value total (m/s^1.75) |
| `cf` | Crest Factor — si CF > 9: usar VDV en lugar de RMS |
| `jerk_rms_ms3` | RMS del jerk de toda la señal (m/s³) |
| `jerk_p99_ms3` | Percentil 99 del jerk absoluto (m/s³) |

---

### `arreglar_csv_mpu.py` — Corrector de unidades

Detecta y corrige automáticamente CSVs con datos de aceleración mal escalados (problema de versiones antiguas del firmware).

```bash
python Datos/arreglar_csv_mpu.py grabacion.csv
# Genera: grabacion_fixed.csv
```

---

### `correlacion.py` — Sincronización de sensores

Estima el desfase temporal entre dos grabaciones simultáneas (de distinto ESP32) mediante correlación cruzada. Actualmente tiene las rutas hardcodeadas; editar directamente el archivo para apuntarlas a los CSVs deseados.

---

## Flujo de trabajo típico

```
1. Conectar ESP32(s) y GPS (opcional) por USB
2. Ejecutar logger_plot4.py
3. Recoger datos → se genera CSV automáticamente
4. Analizar con graficas.py / analisis_vibraciones.py / ruta.py
```

---

## Dependencias Python

```bash
pip install pyserial matplotlib pandas numpy scipy folium
```

> Para exportar PNG desde `ruta.py` también se necesita: `pip install selenium webdriver-manager`

---

## Notas

- Las rutas de salida (`OUTPUT_DIR`) en todos los scripts apuntan a `/home/adrian/...` (entorno Linux de desarrollo). **Deben cambiarse** antes de ejecutar.
- El baseline (sesgo DC del sensor) se calcula automáticamente con las primeras 200 muestras. Los datos se guardan **con baseline ya sustraído**.
- Si usas varias PCBs a la vez, cada una debe tener diferente `UBICACION` configurada en su `platformio.ini`.
