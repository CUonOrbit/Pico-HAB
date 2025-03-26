#include <sys/_stdint.h>
#include "Mpu.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <math.h>

static MPU6050 mpu(MPU_DEV_ADDR);

void quaternionToEuler(MpuDMPData* data) {
  Quaternion q = data->q;

  // Roll (x axis)
  float sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
  float cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
  data->eulerAngles[ROLL_DATA] = atan2f(sinr_cosp, cosr_cosp);

  // Pitch (y axis)
  float sinp = sqrt(1 + 2 * (q.w * q.y - q.x * q.z));
  float cosp = sqrt(1 - 2 * (q.w * q.y - q.x * q.z));
  data->eulerAngles[PITCH_DATA] = 2 * atan2f(sinp, cosp) - M_PI / 2;

  // Yaw (z axis)
  float siny_cosp = 2 * (q.w * q.z + q.x * q.y);
  float cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
  data->eulerAngles[YAW_DATA] = atan2f(siny_cosp, cosy_cosp);
}

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

void Mpu::getDataDMP(MpuDMPData* data) {  
  // packetSize = mpu.dmpGetFIFOPacketSize(); // Get expected DMP packet size for later comparison
  mpu.dmpGetCurrentFIFOPacket(fifoBuffer);

  mpu.dmpGetQuaternion(&data->q, fifoBuffer);
  mpu.dmpGetGravity(&data->gravity, &data->q);

  quaternionToEuler(data);
}

void Mpu::getDataRaw(MpuRawData* data) {
  int16_t accelRaw[3];
  int16_t gyroRaw[3];

  mpu.getMotion6(&data->accel[0], &data->accel[1], &data->accel[2], 
                  &data->gyro[0], &data->gyro[1], &data->gyro[2]);
}