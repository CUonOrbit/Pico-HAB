#ifndef MPU_H
#define MPU_H

#include <MPU6050.h>

#define X_AXIS_DATA 0
#define Y_AXIS_DATA 1
#define Z_AXIS_DATA 2

#define MPU_DEV_ADDR 0x68
#define SENSITIVITY_2_G 16.384f // LSB/g
#define GRAVITY_F 9.81f // m/s^2


struct Mpu6050_Data {
  int16_t accel_Raw[3];
  int16_t gyro_Raw[3];

  int16_t accel_mps2[3];
  int16_t gyro_dps[3]; 
};

class Mpu {
public:

    Mpu(); // Constructor
    bool begin(); // Initialize the MPU6050 sensor
    
    /* get Data in different formats */
    void getData_Raw(Mpu6050_Data* data);

    /* TODO: Methods for mps2/dps data, onboard DMP for offset and fused data from both gyro and accel */

private:
    MPU6050 mpu; // MPU6050 sensor object

};

#endif