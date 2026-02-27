# Datos — Justificación ISO 2631 y guía de análisis

## ¿Qué mide este sistema y por qué?

El sistema mide **vibraciones de cuerpo entero** (WBV, *Whole Body Vibration*) según **ISO 2631-1:1997**, centrado exclusivamente en la **cabina del conductor** del autobús EMT.

> El experimento **no evalúa el habitáculo de pasajeros**. El foco es la exposición laboral del conductor, que pasa jornadas completas expuesto a las vibraciones del vehículo. ISO 2631-1 tiene especial relevancia en este contexto porque establece zonas de precaución sanitaria para exposición profesional prolongada.

El sensor MPU6050 se coloca en el **asiento del conductor y/o el suelo de la cabina** y registra aceleraciones en los tres ejes (X, Y, Z) a **83 Hz** — suficiente para cubrir el rango de análisis ISO 2631 (0.5–80 Hz, Nyquist ≥ 40 Hz).

---

## Columnas del CSV y su justificación

### Convención de nomenclatura

Los nombres de columna siguen un patrón consistente. Leer cada parte de izquierda a derecha:

| Parte | Significado | Ejemplos |
|---|---|---|
| `a` | aceleración | `ax`, `ay`, `az` |
| `g` | giroscopio (velocidad angular) | `gx`, `gy`, `gz` |
| `x` / `y` / `z` | eje cartesiano del sensor | `ax_ms2` = aceleración en eje X |
| `h` | horizontal = combinación de X e Y | `aw_h` = RMS horizontal ponderado |
| `_ms2` | unidad: metros por segundo² | aceleración |
| `_ms3` | unidad: metros por segundo³ | jerk (tirón) |
| `_dps` | unidad: grados por segundo | giroscopio |
| `_wk` | filtro de ponderación **Wk** (ISO 2631, eje vertical Z) | `az_wk` |
| `_wd` | filtro de ponderación **Wd** (ISO 2631, ejes horizontales X/Y) | `ax_wd` |
| `aw_` | aceleración ponderada (*a-weighted*) según ISO 2631 | `aw_z_rms1s` |
| `_rms1s` | RMS calculado sobre ventana de **1 segundo** deslizante | `aw_z_rms1s` |
| `vdv_` | Vibration Dose Value acumulado desde inicio de sesión | `vdv_z` |
| `cf_` | Crest Factor en ventana de 1 s | `cf_z` |
| `jerk_` | tirón (derivada de la aceleración) | `jerk_z_ms3` |
| `gps_` | dato de posicionamiento GPS | `gps_lat`, `gps_lon` |
| `t_ms` | timestamp del firmware en milisegundos (relativo al arranque) | — |
| `pc_time_s` | timestamp del PC en segundos (tiempo de recepción del dato) | — |
| `tiempo_s` | tiempo relativo en segundos desde el inicio de la sesión | — |
| `sync_time_s` | tiempo estimado sincronizado firmware↔PC (regresión lineal) | — |
| `segment_id` | identificador de segmento (se incrementa si hay ruptura temporal) | — |

**Ejemplos de lectura:**
- `az_wk` → aceleración en eje **Z**, filtrada con **Wk** (ponderación vertical ISO 2631)
- `aw_z_rms1s` → aceleración **ponderada** eje **Z**, **RMS de 1 segundo** → es el indicador principal de decomodidad
- `vdv_h` → **VDV** acumulado en dirección **horizontal** (combinación X+Y)
- `jerk_z_ms3` → **jerk** (tirón) en eje **Z**, en m/s³


| Columna | Unidad | Justificación |
|---|---|---|
| `ax_ms2`, `ay_ms2`, `az_ms2` | m/s² | Aceleración bruta en los 3 ejes. Dato base para todos los cálculos. |
| `gx_dps`, `gy_dps`, `gz_dps` | °/s | Velocidad angular. Complementaria para detectar oscilaciones/balanceo. |
| `t_ms`, `tiempo_s`, `pc_time_s` | ms / s | Timestamps del sensor y PC para alineación temporal multi-sensor. |

### Ponderación frecuencial ISO 2631-1

La norma define que la aceleración **no es igualmente dañina en todas las frecuencias**. El cuerpo humano es más sensible a ciertos rangos, por lo que aplica filtros de ponderación:

| Columna | Filtro | Rango sensible | Eje |
|---|---|---|---|
| `az_wk` | **Wk** | 4–8 Hz (resonancia espinal) | Vertical Z |
| `ax_wd`, `ay_wd` | **Wd** | 1–2 Hz (sensibilidad horizontal) | Horizontal X/Y |

Estos valores ponderados son los que usa la norma para evaluar el daño real percibido.

### RMS de 1 segundo — indicador principal

| Columna | Descripción | Umbrales ISO 2631-1 |
|---|---|---|
| `aw_z_rms1s` | RMS de 1s de `az_wk` → **a_wz** | < 0.315 m/s² = sin incomodidad; 0.315–0.63 = algo incómodo; 0.63–1.0 = bastante incómodo; > 1.0 = muy incómodo |
| `aw_h_rms1s` | RMS de 1s de √(ax_wd²+ay_wd²) → **a_wh** | Misma escala. En cabina de conductor el eje horizontal recoge movimientos de dirección y baches laterales. |

> **Nota para exposición laboral:** ISO 2631-1 establece una **zona de precaución sanitaria** para conductores, que depende de `a_w` y la duración de la jornada. A partir de `aw ≈ 0.5 m/s²` durante 8 h se entra en zona de riesgo para la columna vertebral.

### VDV — Vibration Dose Value (Annex B ISO 2631-1)

```
VDV(T) = (∫₀ᵀ aₓ(t)⁴ dt)^(1/4)   [m/s^1.75]
```

| Columna | Descripción | Umbral orientativo |
|---|---|---|
| `vdv_z` | VDV acumulado eje Z desde inicio de sesión | > 8.5 m/s^1.75 → zona de precaución; > 17 m/s^1.75 → zona de riesgo |
| `vdv_h` | VDV acumulado horizontal | Igual |

> En cabina de conductor, el VDV es especialmente relevante porque los badenes y baches urbanos generan picos altos que el RMS enmascara.

**Por qué existe VDV:** el RMS infravalora los impactos puntuales (baches, badenes). El VDV eleva a la cuarta potencia, amplificando los picos. ISO 2631-1 § 5.3 obliga a usar VDV cuando el **Crest Factor > 9**.

### Crest Factor (ISO 2631-1 § 5.3)

```
CF = |pico a_w en ventana 1s| / RMS(a_w, 1s)
```

| Columna | Descripción |
|---|---|
| `cf_z`, `cf_h` | Crest Factor en ventana 1s |

**Regla de decisión:** si CF > 9 → el RMS **no es representativo** → usar `vdv_z`/`vdv_h` como métrica principal.

### Jerk (tirón)

```
jerk = Δa / Δt   [m/s³]
```

| Columna | Descripción |
|---|---|
| `jerk_z_ms3` | Brusquedad vertical: baches, badenes |
| `jerk_h_ms3` | Brusquedad horizontal: frenadas, virajes |

No es un indicador ISO 2631 directo, pero estudios de confort en conductores profesionales lo identifican como mejor predictor de fatiga acumulada que el RMS, especialmente en rutas con muchos semáforos y badenes (perfil típico EMT).

### GPS y velocidad

| Columna | Uso |
|---|---|
| `gps_lat`, `gps_lon` | Geolocalizacion del evento de vibración |
| `velocidad` | Correlacionar vibración con velocidad del vehículo |

---

## Datos que NO están en el CSV — cómo obtenerlos

### Amplitudes en frecuencia (espectro FFT y tercios de octava)

Las amplitudes espectrales **no se calculan en tiempo real** porque requieren ventanas de tiempo completas. Se calculan en post-proceso con `analisis_vibraciones.py` (v2):

```bash
python analisis_vibraciones.py \
  --csv1 asiento.csv \
  --csv2 suelo.csv \
  --axis z --fmax 80 \
  --out resultados/
```

**Ficheros de espectro de frecuencia generados:**

| Fichero | Contenido |
|---|---|
| `psd_{eje}.png` | Densidad Espectral de Potencia (Welch) en m²/s⁴/Hz |
| `fft_amplitudes_{eje}.png` | Espectro de amplitudes en m/s² con picos marcados |
| `fft_amplitudes_{eje}.csv` | Tabla: `freq_hz`, `amplitude_csv1_ms2`, `amplitude_csv2_ms2` |
| `spectral_peaks_csv1_{eje}.csv` | Tabla de picos dominantes de CSV1: frecuencia exacta + amplitud + prominencia |
| `spectral_peaks_csv2_{eje}.csv` | Tabla de picos dominantes de CSV2: ídem |
| `third_octave_{eje}.png` | RMS por bandas de 1/3 de octava (0.5–80 Hz) |
| `ratio_{eje}.png` | Ratio CSV2/CSV1 por banda (transmisibilidad / SEAT) |

> Para ajustar el número de picos espectrales reportados: `--n_peaks 15`

### SEAT — Seat Effective Amplitude Transmissibility

```
SEAT = aw(asiento_conductor) / aw(suelo_cabina) × 100%
```

Requiere **dos ESP32 simultáneas**: una en el asiento del conductor y otra en el suelo de la cabina. Permite evaluar si el asiento amortigua o amplifica las vibraciones. Un valor SEAT < 100% indica que el asiento atenúa; > 100% que amplifica.

Se calcula en post-proceso comparando los dos CSVs. `correlacion.py` hace la alineación temporal previa.

### Exposición diaria A(8)

```
A(8) = aw × √(T / 8h)
```

Donde `T` es la duración de exposición real de la jornada. Se calcula en post-proceso a partir del `aw_z_rms1s` medio de la sesión.

### MTVV — Maximum Transient Vibration Value

```
MTVV = max(aw_z_rms1s)   durante toda la sesión
```

Es simplemente el valor máximo de la columna `aw_z_rms1s` en el CSV. Se obtiene con:
```python
import pandas as pd
df = pd.read_csv("grabacion.csv")
mtvv = df["aw_z_rms1s"].max()
```

---

## Flujo de análisis completo

```
[logger_plot4.py]  →  CSV con señal temporal + métricas ISO 2631 en tiempo real
        │
        ├─ graficas.py           → visor interactivo offline con sliders
        ├─ ruta.py               → mapa HTML coloreado por velocidad GPS
        └─ analisis_vibraciones.py (v2)   → análisis normativo completo
                │
                ├─ PSD Welch                → espectro de potencia
                ├─ FFT amplitudes           → amplitud pico por frecuencia
                │       └─ picos espectrales → tabla freq. + amplitud + prominencia
                ├─ Tercios de octava        → RMS por banda + ratio CSV2/CSV1
                ├─ ISO 2631-1 (Wk/Wd)
                │       ├─ aw_rms           → incomodidad ponderada (m/s²)
                │       ├─ VDV              → dosis de vibración total
                │       └─ Crest Factor     → ¿usar RMS o VDV?
                ├─ Jerk                     → brusquedad (m/s³)
                └─ summary_{eje}.json       → resumen completo con sección iso2631
```

### Relación logger_plot4.py ↔ analisis_vibraciones.py

`logger_plot4.py` calcula las métricas ISO 2631 **muestra a muestra** y las guarda en el CSV para monitorización en tiempo real. `analisis_vibraciones.py` las recalcula sobre la señal completa usando `sosfiltfilt` (fase cero), que es más preciso para análisis post-proceso de grabaciones largas. Además añade el análisis espectral (FFT, PSD, tercios de octava) que no puede hacerse en tiempo real.
