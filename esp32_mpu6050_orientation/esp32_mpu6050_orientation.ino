#include <Wire.h>
#include <MPU6500_WE.h>
#include <math.h>

// MPU6050 and MPU6500 use the same I2C address in this configuration.
#define MPU_ADDR 0x68
#define I2C_SDA  19
#define I2C_SCL  18

MPU6500_WE mpu = MPU6500_WE(MPU_ADDR);

float roll = 0.0f;
float pitch = 0.0f;
float yaw = 0.0f;

unsigned long lastUpdate = 0;
unsigned long lastPrint = 0;

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

  // Keep the sensor still and level during this calibration.
  Serial.println("Keep the sensor still for calibration...");
  delay(1000);
  mpu.autoOffsets();

  // These settings are supported by MPU6500_WE and are also valid for an MPU6050.
  mpu.setAccRange(MPU9250_ACC_RANGE_2G);
  mpu.setGyrRange(MPU9250_GYRO_RANGE_250);
  mpu.setAccDLPF(MPU9250_DLPF_6);
  mpu.setGyrDLPF(MPU9250_DLPF_6);

  // Initialize roll and pitch from the accelerometer so startup is stable.
  xyzFloat acc = mpu.getGValues();
  roll = atan2f(acc.y, acc.z) * 180.0f / PI;
  pitch = atan2f(-acc.x, sqrtf(acc.y * acc.y + acc.z * acc.z)) * 180.0f / PI;

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

  // Accelerometer angles. The accelerometer cannot determine yaw.
  const float accRoll = atan2f(acc.y, acc.z) * 180.0f / PI;
  const float accPitch = atan2f(-acc.x, sqrtf(acc.y * acc.y + acc.z * acc.z)) * 180.0f / PI;

  // Gyroscope rates are returned in degrees/second by MPU6500_WE.
  // This assumes the sensor X/Y/Z axes are aligned with roll/pitch/yaw.
  const float gyroRoll = gyro.x;
  const float gyroPitch = gyro.y;
  const float gyroYaw = gyro.z;

  // Complementary filter: gyro gives responsive motion, accelerometer removes drift
  // from roll and pitch. Yaw is gyro-integrated and will drift over time.
  const float gyroWeight = 0.98f;
  roll = gyroWeight * (roll + gyroRoll * dt) + (1.0f - gyroWeight) * accRoll;
  pitch = gyroWeight * (pitch + gyroPitch * dt) + (1.0f - gyroWeight) * accPitch;
  yaw += gyroYaw * dt;

  // Keep yaw in the range -180 to +180 degrees.
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

  // Avoid an excessively tight loop while retaining accurate integration timing.
  delay(2);
}
