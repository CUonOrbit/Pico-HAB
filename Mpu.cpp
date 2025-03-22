#include "Mpu.h"
#include <MPU6050.h>

Mpu::Mpu() : mpu(MPU_DEV_ADDR) {}

bool Mpu::begin() {;
  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println("Connection to MPU6050 failed");
    return false;
  } 
  return true; 

}

void Mpu::getData_Raw(Mpu6050_Data* data) {
  int16_t accelRaw[3];
  int16_t gyroRaw[3];

  mpu.getMotion6(&accelRaw[X_AXIS_DATA], &accelRaw[Y_AXIS_DATA], &accelRaw[Z_AXIS_DATA], 
                  &gyroRaw[X_AXIS_DATA], &gyroRaw[Y_AXIS_DATA], &gyroRaw[Z_AXIS_DATA]);


    data->accel_Raw[X_AXIS_DATA] = accelRaw[X_AXIS_DATA];
    data->accel_Raw[Y_AXIS_DATA] = accelRaw[Y_AXIS_DATA];
    data->accel_Raw[Z_AXIS_DATA] = accelRaw[Z_AXIS_DATA];

    data->gyro_Raw[X_AXIS_DATA] = gyroRaw[X_AXIS_DATA];
    data->gyro_Raw[Y_AXIS_DATA] = gyroRaw[Y_AXIS_DATA];
    data->gyro_Raw[Z_AXIS_DATA] = gyroRaw[Z_AXIS_DATA];

}