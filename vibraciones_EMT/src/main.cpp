#include <Arduino.h>
#include <Wire.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ========= CONFIG FIRMWARE =========
#ifndef UBICACION
#define UBICACION "Suelo"
#endif

// Hz objetivo — debe coincidir con el ODR real del sensor.
// Con DLPF activo, la base interna del MPU6050 es 1 kHz.
// SMPLRT_DIV = round(1000/HZ) - 1 → para 83 Hz: SMPLRT_DIV = 11
// (83.3 Hz es el más cercano a 80 que permite el divisor entero)
#ifndef SAMPLE_HZ
#define SAMPLE_HZ 83
#endif

#define SDA_PIN 21
#define SCL_PIN 22
#define INT_PIN 7     // Pin INT del MPU6050
#define MPU_ADDR 0x69 // 0x68 si AD0 a GND

#ifndef I2C_CLOCK_HZ
#define I2C_CLOCK_HZ 400000
#endif

#ifndef SERIAL_BAUD
#define SERIAL_BAUD 460800
#endif
// ===================================

Adafruit_MPU6050 mpu;

// Flag SYNC: activado por handle_serial_commands(), consumido en loop()
static volatile bool pending_sync = false;

// Flag data-ready: activado por la ISR, consumido en loop()
static volatile bool data_ready = false;

// ISR: se ejecuta en IRAM para evitar fallos de caché de flash en ESP32
void IRAM_ATTR isr_data_ready() { data_ready = true; }

static inline void send_ubicacion() {
  Serial.print(F("Ubicacion:"));
  Serial.println(UBICACION);
}

static void handle_serial_commands() {
  static char buf[64];
  static size_t n = 0;

  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r')
      continue;

    if (c == '\n') {
      buf[n] = 0;
      if (strcmp(buf, "WHO") == 0) {
        send_ubicacion();
      } else if (strcmp(buf, "SYNC") == 0) {
        Serial.println(F("SYNC_OK"));
        pending_sync = true;
      }
      n = 0;
    } else {
      if (n < sizeof(buf) - 1)
        buf[n++] = c;
    }
  }
}

// Escribe directamente en un registro del MPU6050 via I2C
static void mpu_write_reg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(200);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(I2C_CLOCK_HZ);

  while (!mpu.begin(MPU_ADDR, &Wire)) {
    delay(200);
  }

  // ISO 2631: ±4g — resolución 0.0012 m/s²/bit, cubre picos de transporte
  // urbano
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);

  // DLPF a 44 Hz: por debajo de Nyquist (83/2 = 41.5 Hz) → anti-aliasing
  // correcto
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  // ---- Configuración del ODR interno del MPU6050 ----
  // Con DLPF activo, la tasa base del giroscopio es 1 kHz.
  // SMPLRT_DIV = (1000 / SAMPLE_HZ) - 1 = (1000/83) - 1 ≈ 11
  // → ODR real = 1000 / (1 + 11) = 83.33 Hz
  // Registro 0x19 = SMPLRT_DIV
  mpu_write_reg(0x19, (uint8_t)(1000 / SAMPLE_HZ - 1));

  // ---- Configuración del pin INT ----
  // Registro 0x37 = INT_PIN_CFG:
  //   bit 7 = INT_LEVEL   → 0 = activo alto (RISING edge en GPIO 13)
  //   bit 5 = LATCH_INT   → 0 = pulso de 50 µs (no hace falta leer INT_STATUS)
  //   bit 4 = INT_RD_CLR  → 1 = la interrupción se borra al leer cualquier
  //   registro
  mpu_write_reg(0x37, 0b00010000);

  // Registro 0x38 = INT_ENABLE:
  //   bit 0 = DATA_RDY_EN → 1 = activa la interrupción de dato listo
  mpu_write_reg(0x38, 0x01);

  // Conectar ISR al GPIO 13 (flanco de subida, activo alto)
  pinMode(INT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(INT_PIN), isr_data_ready, RISING);

  // Tiempo base desde arranque
  send_ubicacion();
}

void loop() {
  static uint32_t t0_us = micros();

  handle_serial_commands();

  // Reset del tiempo base ante un SYNC del host
  if (pending_sync) {
    t0_us = micros();
    data_ready = false; // descartamos la muestra en vuelo
    pending_sync = false;
  }

  // Esperar a que el MPU6050 avise de dato listo (via INT)
  // No bloqueante: si no hay dato, salimos y volvemos a entrar
  if (!data_ready)
    return;
  data_ready = false;

  // --- Lectura del sensor ---
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp); // Leer datos y limpia el flag DATA_RDY interno

  const float RAD2DEG = 57.2957795f;

  float ax_ms2 = a.acceleration.x;
  float ay_ms2 = a.acceleration.y;
  float az_ms2 = a.acceleration.z;

  float gx_dps = g.gyro.x * RAD2DEG;
  float gy_dps = g.gyro.y * RAD2DEG;
  float gz_dps = g.gyro.z * RAD2DEG;

  // Tiempo relativo en ms desde arranque (o último SYNC)
  uint32_t t_ms = (micros() - t0_us) / 1000UL;

  // Una sola llamada a Serial para minimizar jitter
  char line[128];
  snprintf(line, sizeof(line), "%lu,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
           (unsigned long)t_ms, ax_ms2, ay_ms2, az_ms2, gx_dps, gy_dps, gz_dps);
  Serial.print(line);
}