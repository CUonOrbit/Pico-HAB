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

struct MpuRawData {
  int16_t accel[3];  // X, Y, Z acceleration raw data
  int16_t gyro[3];   // X, Y, Z gyro raw data

  void print() {
    Serial.print("Accel: ");
    Serial.print(accel[X_AXIS_DATA]); Serial.print(", ");
    Serial.print(accel[Y_AXIS_DATA]); Serial.print(", ");
    Serial.println(accel[Z_AXIS_DATA]);

    Serial.print("Gyro: ");
    Serial.print(gyro[X_AXIS_DATA]); Serial.print(", ");
    Serial.print(gyro[Y_AXIS_DATA]); Serial.print(", ");
    Serial.println(gyro[Z_AXIS_DATA]);
  }
};

/* Current YPR method experiences 'Gimbal Lock' 
    Basically, if we pitch or roll the sensor +- 90 deg, the pitch or roll axis will allign with the yaw
    axis. and so they become indistinguishable. This is easy to imagine with an actual gyro with 3 gimbals, 
    but a similar thing happens when usnig Euler angles, even when the IC doesnt have a gimbal (its MEMS). 
    The euler representations of 3d space can still allign like the gimbal. 

    Solution is to update to a quaternion calculation method using DMP
*/ 
struct MpuDMPData {
  Quaternion q;
  VectorFloat gravity;
  float ypr[3];  // Yaw, Pitch, Roll in radians

  void print() {
    Serial.print("YPR: ");
    Serial.print(RADS_TO_DEG(ypr[YAW_DATA])); Serial.print("°, ");
    Serial.print(RADS_TO_DEG(ypr[PITCH_DATA])); Serial.print("°, ");
    Serial.print(RADS_TO_DEG(ypr[ROLL_DATA])); Serial.println("°, ");
  }
};

class Mpu {
public:

    Mpu(); // Constructor
    bool begin(); // Initialize the MPU6050 sensor
    
    uint8_t setupDMP();
    /* get Data in different formats */
    void getDataRaw(MpuRawData* data);
    void getDataDMP(MpuDMPData* data);

    /* TODO: Methods for mps2/dps data, onboard DMP for offset and fused data from both gyro and accel */

private:
    uint8_t fifoBuffer[64];  // Move buffer inside the class
};

#endif