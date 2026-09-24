#include <Wire.h>
#include <MPU6500_WE.h>
#include <math.h>

// MPU6050 and MPU6500 use the same I2C address in this configuration.
#define MPU_ADDR 0x68
#define I2C_SDA  19
#define I2C_SCL  18

// Tune this if the sensor still drifts while stationary.
#define YAW_DEADBAND_DPS 0.8f
#define GYRO_CALIBRATION_SAMPLES 500

MPU6500_WE mpu = MPU6500_WE(MPU_ADDR);

float roll = 0.0f;
float pitch = 0.0f;
float yaw = 0.0f;
float gyroZBias = 0.0f;

unsigned long lastUpdate = 0;
unsigned long lastPrint = 0;

void calibrateYawGyro() {
  Serial.println("Keep the sensor completely still: calibrating yaw gyro...");
  delay(500);

  float sumZ = 0.0f;
  for (int i = 0; i < GYRO_CALIBRATION_SAMPLES; i++) {
    xyzFloat gyro = mpu.getGyrValues();
    sumZ += gyro.z;
    delay(3);
  }

  gyroZBias = sumZ / GYRO_CALIBRATION_SAMPLES;
  Serial.print("Yaw gyro bias: ");
  Serial.print(gyroZBias, 4);
  Serial.println(" deg/s");
}

void setup() {
  Serial.begin(115200);
  delay(500);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  if (!mpu.init()) {
    Serial.println("MPU6050 not detected. Check wiring and I2C address.");
    while (true) {
      delay(1000);
    }
  }

  Serial.println("Keep the sensor still for calibration...");
  delay(1000);
  mpu.autoOffsets();

  mpu.setAccRange(MPU9250_ACC_RANGE_2G);
  mpu.setGyrRange(MPU9250_GYRO_RANGE_250);
  mpu.setAccDLPF(MPU9250_DLPF_6);
  mpu.setGyrDLPF(MPU9250_DLPF_6);

  calibrateYawGyro();

  xyzFloat acc = mpu.getGValues();
  roll = atan2f(acc.y, acc.z) * 180.0f / PI;
  pitch = atan2f(-acc.x, sqrtf(acc.y * acc.y + acc.z * acc.z)) * 180.0f / PI;
  yaw = 0.0f;

  lastUpdate = micros();
  lastPrint = millis();

  Serial.println("Roll(deg), Pitch(deg), Yaw(deg)");
}

void loop() {
  const unsigned long nowMicros = micros();
  const float dt = (nowMicros - lastUpdate) * 1.0e-6f;
  lastUpdate = nowMicros;

  xyzFloat acc = mpu.getGValues();
  xyzFloat gyro = mpu.getGyrValues();

  const float accRoll = atan2f(acc.y, acc.z) * 180.0f / PI;
  const float accPitch = atan2f(-acc.x, sqrtf(acc.y * acc.y + acc.z * acc.z)) * 180.0f / PI;

  const float gyroRoll = gyro.x;
  const float gyroPitch = gyro.y;
  const float correctedGyroYaw = gyro.z - gyroZBias;

  const float gyroWeight = 0.98f;
  roll = gyroWeight * (roll + gyroRoll * dt) + (1.0f - gyroWeight) * accRoll;
  pitch = gyroWeight * (pitch + gyroPitch * dt) + (1.0f - gyroWeight) * accPitch;

  if (fabsf(correctedGyroYaw) > YAW_DEADBAND_DPS) {
    yaw += correctedGyroYaw * dt;
  }

  if (yaw > 180.0f) yaw -= 360.0f;
  if (yaw < -180.0f) yaw += 360.0f;

  if (millis() - lastPrint >= 100) {
    lastPrint = millis();

    Serial.print("Roll: ");
    Serial.print(roll, 2);
    Serial.print(" deg, Pitch: ");
    Serial.print(pitch, 2);
    Serial.print(" deg, Yaw: ");
    Serial.print(yaw, 2);
    Serial.println(" deg");
  }

  delay(2);
}
