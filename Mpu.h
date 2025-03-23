#ifndef MPU_H
#define MPU_H

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


struct Mpu6050_Data {
  uint16_t packetSize;
  Quaternion q;           // [w, x, y, z]         quaternion container
  VectorFloat gravity;    // [x, y, z]            gravity vector
  uint8_t fifoBuffer[64];

  /* sensor information */ 
  int16_t accel_Raw[3];
  int16_t gyro_Raw[3];
  float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector measured in rad/s
};

class Mpu {
public:

    Mpu(); // Constructor
    bool begin(); // Initialize the MPU6050 sensor
    
    uint8_t setupDMP();
    /* get Data in different formats */
    void getData_Raw(Mpu6050_Data* data);
    void output_YPR(Mpu6050_Data* data);

    /* TODO: Methods for mps2/dps data, onboard DMP for offset and fused data from both gyro and accel */
};

#endif