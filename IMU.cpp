#include <sys/_stdint.h>
#include "IMU.h"
#include <Adafruit_LIS3MDL.h>
#include <math.h>

static MPU6050 mpu(MPU_DEV_ADDR);
static Adafruit_LIS3MDL lis3mdl;

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

void LIS3MDL_Initialize() {
  lis3mdl.setPerformanceMode(LIS3MDL_ULTRAHIGHMODE); // least noise, but most power
  lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE); // constantly update data
  lis3mdl.setDataRate(LIS3MDL_DATARATE_40_HZ);
  lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);

  // HardMetal Offsets: (-206.01, -78.57, -214.82) 
}

uint8_t setupDMP() {
  uint8_t devStatus = mpu.dmpInitialize();

  if (devStatus == 0) {
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

IMU::IMU() {
  // Empty Constructor
}

bool IMU::begin() {
  if (!mpu.testConnection()) {
    Serial.println("Connection to MPU6050 failed");
    return false;
  }
  mpu.initialize();

  if(setupDMP() != 0) {
    Serial.println("Error setting up MPU6050 DMP");
    return false;
  }

  if(!lis3mdl.begin_I2C()) {
    Serial.println("Connection to LIS3MDL failed");
    return false;
  }
  LIS3MDL_Initialize();
  
  return true;
}

void IMU::updateOrientation() {
  /* read DMP data */
  mpu.dmpGetCurrentFIFOPacket(fifoBuffer);

  mpu.dmpGetQuaternion(&IMU_Data->q, fifoBuffer);
  mpu.dmpGetGravity(&IMU_Data->gravity, &IMU_Data->q);
  quaternionToEuler(IMU_Data);


}

void IMU::getDataDMP(MpuDMPData * data) {
  // packetSize = mpu.dmpGetFIFOPacketSize(); // Get expected DMP packet size for later comparison
  mpu.dmpGetCurrentFIFOPacket(fifoBuffer);

  mpu.dmpGetQuaternion(&data->q, fifoBuffer);
  mpu.dmpGetGravity(&data->gravity, &data->q);

  quaternionToEuler(data);
}

void IMU::getDataRaw(MpuRawData * data) {
  int16_t accelRaw[3];
  int16_t gyroRaw[3];

  mpu.getMotion6(&data->accel[0], &data->accel[1], &data->accel[2],
                  &data->gyro[0], &data->gyro[1], &data->gyro[2]);
}

void IMU::getMagDataRaw(MagData* data) {
  if(!lis3mdl.magneticFieldAvailable()) {
    return;
  }

  lis3mdl.read();
  data->magDataRaw[0] = lis3mdl.x;
  data->magDataRaw[1] = lis3mdl.y;
  data->magDataRaw[2] = lis3mdl.z;
  data->print();
}

bool IMU::getDMPStatus() {
  return mpu.getDMPEnabled();
}