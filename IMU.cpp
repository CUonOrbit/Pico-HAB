#include <sys/_stdint.h>
#include "IMU.h"
#include <Adafruit_LIS3MDL.h>
#include <math.h>

static MPU6050 mpu(MPU_DEV_ADDR);
static Adafruit_LIS3MDL lis3mdl;

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

  if(!lis3mdl.begin_I2C()) {
    Serial.println("Connection to LIS3MDL failed");
    return false;
  }
  LIS3MDL_Initialize();
  
  if(setupDMP() != 0) {
    Serial.println("Error setting up MPU6050 DMP");
    return false;
  }
  return true;
}

void IMU::updateOrientation() {
  /* Read DMP data */
  mpu.dmpGetCurrentFIFOPacket(fifoBuffer);

  mpu.dmpGetQuaternion(&IMU_Data.q, fifoBuffer);
  mpu.dmpGetGravity(&IMU_Data.gravity, &IMU_Data.q);
  convertQuaternionToEuler();

  /* Read Mag Data*/
  if(!lis3mdl.magneticFieldAvailable()) {
    return; // data not ready
  }

  lis3mdl.read();
  IMU_Data.magData[0] = lis3mdl.x;
  IMU_Data.magData[1] = lis3mdl.y;
  IMU_Data.magData[2] = lis3mdl.z;
}

void IMU::serialPrintOrientation() {

    Serial.print("quat\t");
    Serial.print(IMU_Data.q.w); Serial.print(", ");
    Serial.print(IMU_Data.q.x); Serial.print(", ");
    Serial.print(IMU_Data.q.y); Serial.print(", ");
    Serial.print(IMU_Data.q.z);
    

    Serial.print("\t\tMag Data (X, Y, Z): ");
    Serial.print(IMU_Data.magData[0]); Serial.print(", ");
    Serial.print(IMU_Data.magData[1]); Serial.print(", ");
    Serial.print(IMU_Data.magData[2]); Serial.println(", ");
}

Quaternion IMU::getQuatData() {
  return IMU_Data.q;
}

float* IMU::getMagData() {
  return IMU_Data.magData;
}

/* Private Methods */

void IMU::convertQuaternionToEuler() {
  Quaternion q = IMU_Data.q;

  // Roll (x axis)
  float sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
  float cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
  IMU_Data.eulerAngles[ROLL_DATA] = atan2f(sinr_cosp, cosr_cosp);

  // Pitch (y axis)
  float sinp = sqrt(1 + 2 * (q.w * q.y - q.x * q.z));
  float cosp = sqrt(1 - 2 * (q.w * q.y - q.x * q.z));
  IMU_Data.eulerAngles[PITCH_DATA] = 2 * atan2f(sinp, cosp) - M_PI / 2;

  // Yaw (z axis)
  float siny_cosp = 2 * (q.w * q.z + q.x * q.y);
  float cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
  IMU_Data.eulerAngles[YAW_DATA] = atan2f(siny_cosp, cosy_cosp);
}