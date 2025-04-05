#include "MPU6050Aries.h"

MPU6050Aries mpu;

void setup() {
  Serial.begin(115200);
  if (!mpu.begin()) {
    Serial.println("MPU6050 not detected! Check wiring.");
    while (1);
  }
  Serial.println("MPU6050 Aries Library initialized.");
}

void loop() {
  mpu.update();
  
  Serial.print("Accel: X=");
  Serial.print(mpu.getAccelX(), 2);
  Serial.print(" Y=");
  Serial.print(mpu.getAccelY(), 2);
  Serial.print(" Z=");
  Serial.print(mpu.getAccelZ(), 2);
  
  Serial.print(" | Roll: ");
  Serial.print(mpu.getRoll(), 2);
  Serial.print(" | Pitch: ");
  Serial.print(mpu.getPitch(), 2);
  Serial.print(" | Temp: ");
  Serial.print(mpu.getTemperature(), 2);
  Serial.println(" °C");
  
  delay(5);  // Adjust as needed
}
