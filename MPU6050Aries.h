#ifndef MPU6050ARIES_H
#define MPU6050ARIES_H

#include <Wire.h>
#include "MPU6050_res_define.h"  // Contains register definitions

// Use I2C port 1 (or change as needed)
extern TwoWire Wire;

const int MPU_ADDR = 0x68;  // MPU6050 I2C address

class MPU6050Aries {
public:
  // Constructor
  MPU6050Aries();

  // Initializes the MPU6050 sensor
  bool begin();

  // Reads sensor data and updates filtered roll and pitch
  void update();

  // Get filtered roll and pitch values
  float getRoll();
  float getPitch();

  // Get accelerometer data in g's
  float getAccelX();
  float getAccelY();
  float getAccelZ();

  // Optionally, get temperature if needed
  float getTemperature();
  

private:
  // Raw sensor variables
  signed short rawAx, rawAy, rawAz, rawT, rawGx, rawGy, rawGz;

  // Converted sensor values (physical units)
  float accelX, accelY, accelZ;   // in g's
  float gyroX, gyroY, gyroZ;      // in °/s
  float temperature;              // in °C

  // Filtered orientation (in degrees)
  float roll, pitch;

  // Timing
  unsigned long lastTime;
  float dt;

  // Kalman filter class for one axis (internal to our library)
  class KalmanFilter {
  public:
    float Q_angle;    // Process noise variance for the accelerometer
    float Q_bias;     // Process noise variance for the gyro bias
    float R_measure;  // Measurement noise variance
    float angle;      // The filtered angle estimate
    float bias;       // The estimated gyro bias
    float P[2][2];    // Error covariance matrix

    KalmanFilter();
    float update(float newAngle, float newRate, float dt);
  };

  // Kalman filter instances for roll and pitch
  KalmanFilter kalmanRoll;
  KalmanFilter kalmanPitch;

  // Private functions for sensor communication
  void mpu6050_init();
  bool mpu6050_checkConnection();
  void mpu6050_readRaw();
};

#endif
