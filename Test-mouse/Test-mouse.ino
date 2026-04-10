#include <BleConnectionStatus.h>
#include <BleMouse.h>
#include <BleMouse.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

BleMouse bleMouse("ESP32 Mouse");
Adafruit_MPU6050 mpu;

void setup() {
  Serial.begin(115200);
  Serial.println("Iniciando Air Mouse...");

  bleMouse.begin();

  // Iniciar I2C
  Wire.begin(21, 22);

  // Iniciar MPU6050
  if (!mpu.begin()) {
    Serial.println("No se detectó el MPU6050");
    while (1);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

  Serial.println("Listo! Esperando conexión BLE...");
}

void loop() {
  if (bleMouse.isConnected()) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Ajustá los factores de sensibilidad
    int moveX = (int)(g.gyro.y * 10);  // mover según rotación en Y
    int moveY = (int)(g.gyro.x * -10); // mover según rotación en X

    // Evitar movimientos pequeños por ruido
    if (abs(moveX) > 1 || abs(moveY) > 1) {
      bleMouse.move(moveX, moveY);
    }

    delay(20); // control de velocidad
  } else {
    delay(500);
  }
}