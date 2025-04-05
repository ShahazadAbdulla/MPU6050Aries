#include "MPU6050Aries.h"
#include <math.h>

// Ensure Wire instance is defined. If not, define it here:
TwoWire Wire = TwoWire(1);

// KalmanFilter constructor
MPU6050Aries::KalmanFilter::KalmanFilter() {
  Q_angle = 0.00045;
  Q_bias  = 0.00008;
  R_measure = 0.03;
  angle = 0;
  bias = 0;
  P[0][0] = 0; P[0][1] = 0;
  P[1][0] = 0; P[1][1] = 0;
}

// KalmanFilter update function
float MPU6050Aries::KalmanFilter::update(float newAngle, float newRate, float dt) {
  // Predict step
  float rate = newRate - bias;
  angle += dt * rate;
  
  // Update error covariance matrix
  P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
  P[0][1] -= dt * P[1][1];
  P[1][0] -= dt * P[1][1];
  P[1][1] += Q_bias * dt;
  
  // Innovation (measurement residual)
  float S = P[0][0] + R_measure;
  // Kalman gain
  float K[2];
  K[0] = P[0][0] / S;
  K[1] = P[1][0] / S;
  
  // Update estimate with measurement
  float y = newAngle - angle;
  angle += K[0] * y;
  bias  += K[1] * y;
  
  // Update error covariance matrix
  float P00_temp = P[0][0];
  float P01_temp = P[0][1];
  
  P[0][0] -= K[0] * P00_temp;
  P[0][1] -= K[0] * P01_temp;
  P[1][0] -= K[1] * P00_temp;
  P[1][1] -= K[1] * P01_temp;
  
  return angle;
}

// MPU6050Aries constructor
MPU6050Aries::MPU6050Aries() {
  lastTime = 0;
  dt = 0;
  roll = 0;
  pitch = 0;
}

// Public begin function: initializes sensor and checks connection
bool MPU6050Aries::begin() {
  Wire.begin();
  delay(2000);
  
  if (!mpu6050_checkConnection()) {
    return false;
  }
  mpu6050_init();
  lastTime = millis();
  return true;
}

// Public update function: reads sensor data and updates Kalman filter estimates
void MPU6050Aries::update() {
  mpu6050_readRaw();
  
  // Convert raw values to physical units with offsets (adjust as needed)
  accelX = (float)rawAx / 16384.0 - 0.04;
  accelY = (float)rawAy / 16384.0;
  accelZ = (float)rawAz / 16384.0 + 0.40;
  
  gyroX = (float)rawGx / 131.0;
  gyroY = (float)rawGy / 131.0;
  gyroZ = (float)rawGz / 131.0;
  
  temperature = ((float)rawT / 340.0) + 36.53;
  
  // Calculate accelerometer angles (in degrees)
  float accelRoll = atan2(accelY, accelZ) * (180.0 / PI);
  float accelPitch = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * (180.0 / PI);
  
  // Calculate dt
  unsigned long currentTime = millis();
  dt = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;
  
  // Update Kalman filter estimates for roll and pitch
  roll  = kalmanRoll.update(accelRoll, gyroX, dt);
  pitch = kalmanPitch.update(accelPitch, gyroY, dt);
}

// Public getters
float MPU6050Aries::getRoll() {
  return roll;
}

float MPU6050Aries::getPitch() {
  return pitch;
}

float MPU6050Aries::getAccelX() {
  return accelX;
}

float MPU6050Aries::getAccelY() {
  return accelY;
}

float MPU6050Aries::getAccelZ() {
  return accelZ;
}

float MPU6050Aries::getTemperature() {
  return temperature;
}

//-------------------------------------------------
// Private functions for sensor communication

void MPU6050Aries::mpu6050_init() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(SMPLRT_DIV);
  Wire.write(0x07);  // 1KHz sample rate
  Wire.endTransmission(true);
  delayMicroseconds(100);
  
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(PWR_MGMT_1);
  Wire.write(0x01);  // Set clock source to X-axis gyro reference
  Wire.endTransmission(true);
  delayMicroseconds(100);
  
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(CONFIG);
  Wire.write(0x00);  // Default DLPF configuration
  Wire.endTransmission(true);
  delayMicroseconds(100);
  
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(GYRO_CONFIG);
  Wire.write(0x18);  // Full scale range +/- 2000 °/s
  Wire.endTransmission(true);
  delayMicroseconds(100);
}

bool MPU6050Aries::mpu6050_checkConnection() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x75);  // WHO_AM_I register address
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 1, true);
  if (Wire.available()) {
    uint8_t whoAmI = Wire.read();
    return (whoAmI == 0x68);
  }
  return false;
}

void MPU6050Aries::mpu6050_readRaw() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(true);
  Wire.requestFrom(MPU_ADDR, 14, true);
  
  rawAx = (Wire.read() << 8) | Wire.read();
  rawAy = (Wire.read() << 8) | Wire.read();
  rawAz = (Wire.read() << 8) | Wire.read();
  rawT  = (Wire.read() << 8) | Wire.read();
  rawGx = (Wire.read() << 8) | Wire.read();
  rawGy = (Wire.read() << 8) | Wire.read();
  rawGz = (Wire.read() << 8) | Wire.read();
}
