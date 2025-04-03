#ifndef IMU_H
#define IMU_H

#include "MPU6050_6Axis_MotionApps20.h"

#define X_AXIS_DATA 0
#define Y_AXIS_DATA 1
#define Z_AXIS_DATA 2
#define YAW_DATA 0    // rotation around z axis
#define PITCH_DATA 1  // rotation around y axis
#define ROLL_DATA 2   // rotation around x axis

#define MPU_DEV_ADDR 0x68
#define GRAVITY_F 9.81f // m/s^2

#define RADS_TO_DEG(x) (x * 180/M_PI)

class IMU {
public:

  IMU(); // Constructor
  bool begin(); // Initialize the MPU6050 sensor
  void updateOrientation();

  void serialPrintOrientation();

  Quaternion getQuatData();
  float* getMagData();
  /* TODO: Methods for mps2/dps data, onboard DMP for offset and fused data from both gyro and accel */

private:
  uint8_t fifoBuffer[64];  // Move buffer inside the class
    
  struct IMU_Data {
    /* nested struct for raw data? */

    /* Accel + Gyro Data - MPU6050 DMP */
    Quaternion q;
    VectorFloat gravity;
    float eulerAngles[3];

    /* Magnetometer Data - LIS3MDL */
    float magData[3]; // raw currently, will change to offset values with hard/soft metal factors 
  } IMU_Data;

  void convertQuaternionToEuler();
};

#endif