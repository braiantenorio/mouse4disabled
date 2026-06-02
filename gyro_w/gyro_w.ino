#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <BleMouse.h>

float MIN_SENSITIVITY = 30.0;
float MAX_SENSITIVITY = 40.0;
float ACCEL_THRESHOLD = 1.5;

float SMOOTHING_ALPHA = 0.2;
float GYRO_DEADZONE = 0.05;

bool VERTICAL_MODE = false;
bool INVERT_X = true;
bool INVERT_Y = false;

Adafruit_MPU6050 mpu;
BleMouse bleMouse("ESP32 Air Mouse");

float gyroX_offset = 0, gyroY_offset = 0, gyroZ_offset = 0;
float smoothX = 0, smoothY = 0;
unsigned long freezeEndTime = 0;

// tasks
TaskHandle_t MouseTaskHandle = NULL;


float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void calibrateGyro() {
  float sumX = 0, sumY = 0, sumZ = 0;
  int samples = 500;

  for (int i = 0; i < samples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    sumX += g.gyro.x;
    sumY += g.gyro.y;
    sumZ += g.gyro.z;

    delay(3);
  }

  gyroX_offset = sumX / samples;
  gyroY_offset = sumY / samples;
  gyroZ_offset = sumZ / samples;
}


void mouseTask(void *pvParameters) {

  while (true) {

    if (!bleMouse.isConnected()) {
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    float rawX = g.gyro.x - gyroX_offset;
    float rawY = g.gyro.y - gyroY_offset;
    float rawZ = g.gyro.z - gyroZ_offset;

    if (abs(rawX) < GYRO_DEADZONE) rawX = 0;
    if (abs(rawY) < GYRO_DEADZONE) rawY = 0;
    if (abs(rawZ) < GYRO_DEADZONE) rawZ = 0;

    static unsigned long lastPrint = 0;

    if (millis() - lastPrint > 200) {
      Serial.printf(
        "rawX=%.3f rawY=%.3f rawZ=%.3f\n",
        rawX,
        rawY,
        rawZ);
      lastPrint = millis();
    }

    float xyMagnitude = sqrt(rawX * rawX + rawZ * rawZ);

    static unsigned long lastClickTime = 0;
    const unsigned long CLICK_COOLDOWN = 500;  // ms

    if (abs(rawY) > xyMagnitude * 2.0 && abs(rawY) > 1.5 && millis() - lastClickTime > CLICK_COOLDOWN) {

      // Congelar el cursor un instante
      freezeEndTime = millis() + 200;

      // Click izquierdo
      bleMouse.click(MOUSE_LEFT);

      lastClickTime = millis();
    }

    float activeX = 0;
    float activeY = 0;

    if (VERTICAL_MODE) {
      activeX = rawX;
      activeY = rawZ;
    } else {
      activeX = rawZ;
      activeY = rawX;
    }

    float velocity = sqrt(activeX * activeX + activeY * activeY);

    float sensitivity = MIN_SENSITIVITY;

    if (velocity > 0.1) {

      sensitivity = mapFloat(
        velocity,
        0,
        ACCEL_THRESHOLD,
        MIN_SENSITIVITY,
        MAX_SENSITIVITY);

      if (sensitivity > MAX_SENSITIVITY)
        sensitivity = MAX_SENSITIVITY;
    }

    float targetMoveX = activeX * sensitivity;
    float targetMoveY = activeY * sensitivity;

    smoothX = (SMOOTHING_ALPHA * targetMoveX) + ((1.0 - SMOOTHING_ALPHA) * smoothX);
    smoothY = (SMOOTHING_ALPHA * targetMoveY) + ((1.0 - SMOOTHING_ALPHA) * smoothY);

    int finalX = (int)smoothX;
    int finalY = (int)smoothY;

    if (INVERT_X) finalX = -finalX;
    if (INVERT_Y) finalY = -finalY;

    bool movementAllowed = true;

    if (millis() < freezeEndTime)
      movementAllowed = false;

    if (movementAllowed && (finalX != 0 || finalY != 0)) {
      bleMouse.move(finalX, finalY);
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}


void setup() {

  Serial.begin(115200);
  Wire.begin(21, 22);


  if (!mpu.begin()) {
    Serial.println("MPU6050 not found!");
    while (1) delay(10);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_2000_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("Calibrating... Keep STILL.");
  delay(1000);

  calibrateGyro();

  Serial.println("Go!");

  bleMouse.begin();

  xTaskCreatePinnedToCore(
    mouseTask,
    "MouseTask",
    4096,
    NULL,
    1,
    &MouseTaskHandle,
    1);
}


void loop() {
}