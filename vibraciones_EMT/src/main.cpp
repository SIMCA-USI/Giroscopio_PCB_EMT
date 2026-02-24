#include <Arduino.h>
#include <Wire.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#define UBICACION "Asiento"

#define SDA_PIN 21
#define SCL_PIN 22
#define MPU_ADDR 0x69  // 0x68 si AD0 está a GND

Adafruit_MPU6050 mpu;

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println(F("Iniciando I2C..."));
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
  Serial.println(F("Wire.begin OK"));

  Serial.println(F("Iniciando MPU6050..."));

  while (!mpu.begin(MPU_ADDR, &Wire)) {
    Serial.println(F("Error al iniciar IMU (begin = false)"));
    delay(500);
    Serial.println(F("Reintentando MPU6050..."));
  }
  Serial.println(F("IMU iniciada correctamente"));

  // Mantener tu configuración (±8g, ±250 dps)
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);
}

void loop() {
  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);

  static uint32_t t0 = millis();
  uint32_t t_ms = millis() - t0;

  // Adafruit:
  // - a.acceleration.* está en m/s^2  -> pasamos a g
  // - g.gyro.* está en rad/s         -> pasamos a dps (deg/s)
  const float G = 9.80665f;
  const float RAD2DEG = 57.2957795f;

  float ax_ms2 = a.acceleration.x;
  float ay_ms2 = a.acceleration.y;
  float az_ms2 = a.acceleration.z;

  float gx_dps = g.gyro.x * RAD2DEG;
  float gy_dps = g.gyro.y * RAD2DEG;
  float gz_dps = g.gyro.z * RAD2DEG;

  // ---- Salida para debug ----
  Serial.print(F("Ubicacion:\t"));
  Serial.print(UBICACION);
  Serial.print('\n');

  Serial.print(F("Accel[g]: "));
  Serial.print(ax_ms2); Serial.print('\t');
  Serial.print(ay_ms2); Serial.print('\t');
  Serial.print(az_ms2); Serial.print('\t');

  Serial.print(F("Gyro[dps]: "));
  Serial.print(gx_dps); Serial.print('\t');
  Serial.print(gy_dps); Serial.print('\t');
  Serial.println(gz_dps);

  // ---- Línea CSV para Python ----
  // Formato CSV: t_ms,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps
  Serial.print(t_ms);      Serial.print(",");
  Serial.print(ax_ms2, 6); Serial.print(",");
  Serial.print(ay_ms2, 6); Serial.print(",");
  Serial.print(az_ms2, 6); Serial.print(",");
  Serial.print(gx_dps, 6); Serial.print(",");
  Serial.print(gy_dps, 6); Serial.print(",");
  Serial.println(gz_dps, 6);

  delay(10);  // ~100 Hz
}