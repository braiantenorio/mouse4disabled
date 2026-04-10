#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <BleMouse.h>

float MIN_SENSITIVITY  = 2.0;
float MAX_SENSITIVITY  = 25.0;
float ACCEL_THRESHOLD  = 1.5;

float SMOOTHING_ALPHA  = 0.2;
float GYRO_DEADZONE    = 0.08;
int CLICK_FREEZE_MS    = 150;

bool VERTICAL_MODE = false;
bool INVERT_X      = true;
bool INVERT_Y      = false;

#define LEFT_BTN  25
#define RIGHT_BTN 26

Adafruit_MPU6050 mpu;
BleMouse bleMouse("ESP32 Air Mouse");

float gyroX_offset = 0, gyroY_offset = 0, gyroZ_offset = 0;
float smoothX = 0, smoothY = 0;
unsigned long freezeEndTime = 0;

bool lastLeftState = HIGH;
bool lastRightState = HIGH;

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x-in_min)*(out_max-out_min)/(in_max-in_min)+out_min;
}

void calibrateGyro() {
  float sumX = 0, sumY = 0, sumZ = 0;
  int samples = 200;

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

void setup() {

  Serial.begin(115200);
  Wire.begin(21,22);

  pinMode(LEFT_BTN, INPUT_PULLUP);
  pinMode(RIGHT_BTN, INPUT_PULLUP);

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
}

void loop() {

  if (!bleMouse.isConnected()) {
    delay(100);
    return;
  }

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float rawX = g.gyro.x - gyroX_offset;
  float rawY = g.gyro.y - gyroY_offset;
  float rawZ = g.gyro.z - gyroZ_offset;

  if (abs(rawX) < GYRO_DEADZONE) rawX = 0;
  if (abs(rawY) < GYRO_DEADZONE) rawY = 0;
  if (abs(rawZ) < GYRO_DEADZONE) rawZ = 0;

  float activeX = 0;
  float activeY = 0;

  if (VERTICAL_MODE) {
    activeX = rawX;
    activeY = rawZ;
  } else {
    activeX = rawZ;
    activeY = rawX;
  }

  float velocity = sqrt(activeX*activeX + activeY*activeY);

  float sensitivity = MIN_SENSITIVITY;

  if (velocity > 0.1) {

    sensitivity = mapFloat(
      velocity,
      0,
      ACCEL_THRESHOLD,
      MIN_SENSITIVITY,
      MAX_SENSITIVITY
    );

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

  int currentLeft = digitalRead(LEFT_BTN);

  if (currentLeft != lastLeftState) {

    if (currentLeft == LOW) {
      bleMouse.press(MOUSE_LEFT);
      freezeEndTime = millis() + CLICK_FREEZE_MS;
      movementAllowed = false;
    }
    else {
      bleMouse.release(MOUSE_LEFT);
    }

    lastLeftState = currentLeft;
  }

  int currentRight = digitalRead(RIGHT_BTN);

  if (currentRight != lastRightState) {

    if (currentRight == LOW) {
      bleMouse.press(MOUSE_RIGHT);
      freezeEndTime = millis() + CLICK_FREEZE_MS;
      movementAllowed = false;
    }
    else {
      bleMouse.release(MOUSE_RIGHT);
    }

    lastRightState = currentRight;
  }

  if (movementAllowed && (finalX != 0 || finalY != 0)) {
    bleMouse.move(finalX, finalY);
  }

  delay(10);
}