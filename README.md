# MPU6050Aries

MPU6050Aries is an Arduino library designed for Aries boards (or other Vega/RISC-V systems) to interface with the MPU6050 sensor. This library initializes the sensor, reads raw data, and uses a Kalman filter to fuse accelerometer and gyroscope measurements for stable roll and pitch estimates. It also provides temperature readings. This library is ideal for applications like self-balancing robots.

## Features

- **Platform Specific:** Tailored for Aries boards (Vega/RISC-V systems).
- **Sensor Fusion:** Uses a Kalman filter to combine accelerometer and gyroscope data for stable orientation estimates.
- **Modular Design:** Easy-to-use functions for initialization, data reading, and angle retrieval.
- **Self-Balancing Applications:** Suitable for robotics projects that require reliable tilt measurements.
- **Extensible:** Easily modified to include additional sensor data (e.g., accelerometer values).

## Installation

1. **Clone or Download the Repository:**
   - **Clone with Git:**
     ```bash
     git clone https://github.com/ShahazadAbdulla/MPU6050Aries.git
     ```
   - **Download as ZIP:**  
     Go to the repository page on GitHub and click the green **Code** button, then **Download ZIP**.

2. **Install in Arduino IDE:**
   - Extract the folder and rename it to `MPU6050Aries` (if it isn’t already).
   - Copy the `MPU6050Aries` folder into your Arduino libraries directory (typically `~/Documents/Arduino/libraries`).
   - Restart the Arduino IDE.

## Usage

Include the library in your sketch with:

```cpp
#include "MPU6050Aries.h"
```

## Library Structure

- **MPU6050Aries.h**  
  Contains the class definition for `MPU6050Aries`, including:
  - `bool begin()` – Initializes the sensor.
  - `void update()` – Reads sensor data and updates the Kalman filter estimates.
  - Getter functions: `getRoll()`, `getPitch()`, `getTemperature()`, `getAccelX()`, `getAccelY()`, and `getAccelZ()`.

- **MPU6050Aries.cpp**  
  Implements all functions declared in the header. This file handles:
  - Sensor initialization (setting sample rate, power management, DLPF, and gyro configuration).
  - Reading raw sensor data via I²C.
  - Converting raw data to physical units.
  - Fusing sensor data using a Kalman filter for roll and pitch estimation.

- **MPU6050_res_define.h**  
  Contains register definitions for the MPU6050 (ensure this file is present).

## Kalman Filter Overview

The Kalman filter in this library fuses the gyroscope and accelerometer data to obtain a reliable estimate of roll and pitch:

- **Prediction Step:**  
  Integrates the gyro rate (after subtracting the estimated bias) over time to predict the new angle.

- **Error Covariance Update:**  
  Dynamically updates the uncertainty (error covariance) of the prediction using process noise parameters (`Q_angle` and `Q_bias`).

- **Measurement Update:**  
  Uses the accelerometer’s angle (computed via `atan2`) as the measurement to correct the prediction. The Kalman gain is calculated based on the measurement noise (`R_measure`).

- **Recursive Update:**  
  The filter continuously corrects the predicted angle, reducing noise and drift over time.

See the code snippet in `MPU6050Aries.cpp` for the complete implementation.

## Tuning and Calibration

- **Kalman Parameters:**  
  - `Q_angle = 0.00045`
  - `Q_bias = 0.00008`
  - `R_measure = 0.03`  
  You may need to adjust these values based on your specific application and update rate.

- **Accelerometer Offsets:**  
  Fixed offsets are applied during conversion (e.g., `-0.04` for X and `+0.40` for Z). For improved accuracy, consider implementing a calibration routine that averages many readings at startup.

- **Update Frequency:**  
  With a delay of 5 ms in your main loop, the update rate is high, though your measured `dt` might be around 0.02 s. Monitor `dt` and retune parameters as necessary.

## Contributing

Contributions, bug fixes, and improvements are welcome!

1. Fork this repository.
2. Create your feature branch:
   ```bash
   git checkout -b feature/YourFeature
   ```
3. Commit your changes:
   ```bash
   git commit -am 'Add new feature'
   ```
4. Push to the branch:
   ```bash
   git push origin feature/YourFeature
   ```
5. Create a new Pull Request.

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

## Acknowledgements

**Inspiration:**  
This library was inspired by the example code provided with the Aries board.

**References:**  
- [Engineers Garage MPU6050 Code](https://www.engineersgarage.com/Wire345-accelerometer-arduino-i2c/)  
- [VEGA Processors Blog](https://vegaprocessors.in/blog/interfacing-adxl345-digital-accelerometer-to-thejas-soc/)

Special thanks to all contributors and community members for their valuable feedback.


