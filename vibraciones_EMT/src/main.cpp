#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050.h"

#define SDA_PIN 21
#define SCL_PIN 22

const int mpuAddress = 0x69;  // PRUEBA PRIMERO 0x69

MPU6050 mpu(mpuAddress);

int16_t ax, ay, az;
int16_t gx, gy, gz;

void printTab() { Serial.print('\t'); }

void printRAW() {
  Serial.print(F("a[x y z] g[x y z]:"));
  printTab();
  Serial.print(ax); printTab();
  Serial.print(ay); printTab();
  Serial.print(az); printTab();
  Serial.print(gx); printTab();
  Serial.print(gy); printTab();
  Serial.println(gz);
}

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println(F("Iniciando I2C..."));
  bool ok = Wire.begin(SDA_PIN, SCL_PIN, 100000);  // BAJA a 100kHz para probar
  if (!ok) {
    Serial.println(F("ERROR: Wire.begin() ha fallado"));
    while (true) delay(1000);
  }
  Serial.println(F("Wire.begin OK"));

  Serial.println(F("Iniciando MPU6050..."));
  mpu.initialize();

  if (mpu.testConnection()) {
    Serial.println(F("IMU iniciada correctamente"));
  } else {
    Serial.println(F("Error al iniciar IMU (testConnection = false)"));
  }
}

void loop() {
  mpu.getAcceleration(&ax, &ay, &az);
  mpu.getRotation(&gx, &gy, &gz);

  printRAW();
  delay(200);
}
