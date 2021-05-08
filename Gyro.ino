#include "Wire.h"
#include <MPU6050.h>

static const float MPU_TIME_STEP_MS = 10;
unsigned long mpuStarted = 0;
float yaw = 0;

MPU6050 mpu;

/*
   setup()
*/
void gyroSetup() {

  //Serial.begin(115200);

  Serial.println("Initializing MPU6050 ...");
  // Initialize MPU6050
  while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }

  // Calibrate gyroscope. The calibration must be at rest.
  // If you don't want calibrate, comment this line.
  mpu.calibrateGyro();

  // Set threshold sensivty. Default 3.
  // If you don't want use threshold, comment this line or set 0.
  mpu.setThreshold(0);

  Serial.println("MPU6050 Setup Complete...");
}

/*
   loop()
*/
void gyroLoop() {
  
    unsigned long currentMillis = millis();
  //if (currentMillis - mpuStarted >= MPU_TIME_STEP_MS) {
    
    // Read normalized values
    Vector norm = mpu.readRawGyro();
    yaw = yaw + norm.ZAxis * ((currentMillis - mpuStarted) / 1000.0);
    mpuStarted = currentMillis;
   // yaw = yaw + norm.ZAxis * (MPU_TIME_STEP_MS / 1000.0);
    //Serial.print("Heading = ");
    //Serial.println(yaw);
 // }
}
int getYaw(){
  return round(yaw);
}
