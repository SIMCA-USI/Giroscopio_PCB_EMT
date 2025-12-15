#include <Arduino.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050.h"

#define UBICACION "Asiento"

#define SDA_PIN 21
#define SCL_PIN 22

const int mpuAddress = 0x69;  // AD0 en HIGH

MPU6050 mpu(mpuAddress);

// Variables globales de lectura cruda
int16_t ax, ay, az;
int16_t gx, gy, gz;

// Offsets (en cuentas crudas)
float ax_off = 0, ay_off = 0, az_off = 0;
float gx_off = 0, gy_off = 0, gz_off = 0;

void calibrateMPU() {
  const int N = 1000;
  long ax_sum = 0, ay_sum = 0, az_sum = 0;
  long gx_sum = 0, gy_sum = 0, gz_sum = 0;

  Serial.println(F("Calibrando... NO muevas la placa"));

  for (int i = 0; i < N; i++) {
    // Leemos las 6 magnitudes de golpe
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    ax_sum += ax;
    ay_sum += ay;
    az_sum += az;
    gx_sum += gx;
    gy_sum += gy;
    gz_sum += gz;

    delay(2);  // pequeño retardo
  }

  float ax_mean = ax_sum / (float)N;
  float ay_mean = ay_sum / (float)N;
  float az_mean = az_sum / (float)N;
  float gx_mean = gx_sum / (float)N;
  float gy_mean = gy_sum / (float)N;
  float gz_mean = gz_sum / (float)N;

  // Queremos que en reposo todos los ejes den 0 (quitamos también la gravedad)
  ax_off = ax_mean;
  ay_off = ay_mean;
  az_off = az_mean;   // <-- antes era az_mean - ACC_SCALE (dejaba 1g)
  gx_off = gx_mean;
  gy_off = gy_mean;
  gz_off = gz_mean;

  Serial.println(F("Calibración terminada (offsets crudos):"));
  Serial.print(F("ax_off=")); Serial.print(ax_off);
  Serial.print(F(" ay_off=")); Serial.print(ay_off);
  Serial.print(F(" az_off=")); Serial.println(az_off);
  Serial.print(F("gx_off=")); Serial.print(gx_off);
  Serial.print(F(" gy_off=")); Serial.print(gy_off);
  Serial.print(F(" gz_off=")); Serial.println(gz_off);
}

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println(F("Iniciando I2C..."));
  bool ok = Wire.begin(SDA_PIN, SCL_PIN, 100000);  // SDA=21, SCL=22, 100kHz
  if (!ok) {
    Serial.println(F("ERROR: Wire.begin() ha fallado"));
    while (true) delay(1000);
  }
  Serial.println(F("Wire.begin OK"));

  Serial.println(F("Iniciando MPU6050..."));
  mpu.initialize();

  // Acelerómetro ±2g
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);

  // Giroscopio ±250 °/s
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);

  if (mpu.testConnection()) {
    Serial.println(F("IMU iniciada correctamente"));
  } else {
    Serial.println(F("Error al iniciar IMU (testConnection = false)"));
  }

  // Muy importante: calibrar con la placa quieta
  calibrateMPU();
}

void loop() {
  // Leemos las 6 magnitudes crudas
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  static uint32_t t0 = millis();
  uint32_t t_ms = millis() - t0;

  // Aplica offsets (crudos)
  float ax_corr = ax - ax_off;
  float ay_corr = ay - ay_off;
  float az_corr = az - az_off;

  float gx_corr = gx - gx_off;
  float gy_corr = gy - gy_off;
  float gz_corr = gz - gz_off;

  const float ACC_SCALE  = 16384.0f; // LSB/g (±2g)
  const float GYRO_SCALE = 131.0f;   // LSB/(°/s) (±250 dps)

  // Pasa a unidades físicas
  float ax_g   = ax_corr / ACC_SCALE;
  float ay_g   = ay_corr / ACC_SCALE;
  float az_g   = az_corr / ACC_SCALE;

  float gx_dps = gx_corr / GYRO_SCALE;
  float gy_dps = gy_corr / GYRO_SCALE;
  float gz_dps = gz_corr / GYRO_SCALE;

  // ---- Salida para debug ----
  Serial.print(F("Ubicacion:\t"));
  Serial.print(UBICACION);
  Serial.print('\n');

  Serial.print(F("Accel[g]: "));
  Serial.print(ax_g); Serial.print('\t');
  Serial.print(ay_g); Serial.print('\t');
  Serial.print(az_g); Serial.print('\t');

  Serial.print(F("Gyro[dps]: "));
  Serial.print(gx_dps); Serial.print('\t');
  Serial.print(gy_dps); Serial.print('\t');
  Serial.println(gz_dps);

  // ---- Línea CSV cruda para Python ----
  // Formato CSV: t_ms,ax,ay,az,gx,gy,gz
  Serial.print(t_ms); Serial.print(",");
  Serial.print(ax);   Serial.print(",");
  Serial.print(ay);   Serial.print(",");
  Serial.print(az);   Serial.print(",");
  Serial.print(gx);   Serial.print(",");
  Serial.print(gy);   Serial.print(",");
  Serial.println(gz);

  delay(10);  // ~100 Hz
}
