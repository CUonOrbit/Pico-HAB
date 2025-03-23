#include <sys/_stdint.h>
#include "Mpu.h"
#include "MPU6050_6Axis_MotionApps20.h"

static MPU6050 mpu(MPU_DEV_ADDR);



Mpu::Mpu() {
  // Empty Constructor
}

bool Mpu::begin() {;
  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println("Connection to MPU6050 failed");
    return false;
  } 
  return true; 
}

uint8_t Mpu::setupDMP() {
  uint8_t devStatus = mpu.dmpInitialize();

  if(devStatus == 0) {
    /* Set offsets */ 
    mpu.setXGyroOffset(0);
    mpu.setYGyroOffset(0);
    mpu.setZGyroOffset(0);
    mpu.setXAccelOffset(0);
    mpu.setYAccelOffset(0);
    mpu.setZAccelOffset(0);

    /* Generate Offsets */
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);

    Serial.print("\nActive offsets: ");
    mpu.PrintActiveOffsets();

    /* Enable Sensor */ 
    mpu.setDMPEnabled(true);

  }
  return devStatus;
}

void Mpu::output_YPR(Mpu6050_Data *data) {  
  data->packetSize = mpu.dmpGetFIFOPacketSize(); //Get expected DMP packet size for later comparison
  mpu.dmpGetCurrentFIFOPacket(data->fifoBuffer);

  mpu.dmpGetQuaternion(&data->q, data->fifoBuffer);
  mpu.dmpGetGravity(&data->gravity, &data->q);
  mpu.dmpGetYawPitchRoll(data->ypr, &data->q, &data->gravity);
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