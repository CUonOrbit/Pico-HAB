#include <Arduino.h>
#include <Wire.h>
#include "pico/cyw43_arch.h"

#include "Pico-Hab.h"
#include "Bmp.h"
#include "IMU.h"

Bmp bmp; // Create a Bmp object
IMU imu; // Create a mpu object

volatile bool MPUInterrupt = false;
volatile bool LISInterrupt = false;

void setup() {
  Wire.setSDA(0);  // Pico SDA on GP0
  Wire.setSCL(1);  // Pico SCL on GP1
  Wire.begin();

  Serial.begin(115200);  // Start serial communication
  while (!Serial); // wait for serial port

  rp2040.wdt_begin(8000); // Timeout after 8 seconds - reboot

  /* Init onboard LED */ 
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  /* Init Interrupts */ 
  pinMode(MPU_DMP_INT_PIN, INPUT_PULLUP);
  pinMode(LIS3MDL_DRDY_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(MPU_DMP_INT_PIN), DMPDataReady, RISING);
  attachInterrupt(digitalPinToInterrupt(LIS3MDL_DRDY_PIN), MagDataReady, RISING);
  
  // Initialize BMP180 sensor
  if (!bmp.begin()) {
    Serial.println("Failed to initialize BMP180!");
    while (1); // Halt if initialization fails
  }

  Serial.println("BMP180 initialized!");

  if(!imu.begin()) {
      while (1); // Halt if initialization fails
  }
  Serial.println("MPU6050 and LIS3MDL initialized!");
}

void read_Bmp_Sensor() {
  // Read data from BMP180 sensor
  float temperature = bmp.getTemperature();
  float pressure = bmp.getPressure();
  float altitude = bmp.getAltitude();

  // Print sensor data to Serial Monitor
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" °C");

  Serial.print("Pressure: ");
  Serial.print(pressure);
  Serial.println(" Pa");

  Serial.print("Altitude: ");
  Serial.print(altitude);
  Serial.println(" meters");
}

void loop() {
  // Reset the watchdog timer
  rp2040.wdt_reset();
  // Serial.println(mpu.getDMPStatus());

  // read_Bmp_Sensor();

  if(MPUInterrupt) {
    MPUInterrupt = false;
    imu.updateOrientation();
    imu.serialPrintOrientation();
    // Mpu_DMP.print_Quaternion();
  }
  
  sleep_ms(50);
}

void DMPDataReady() {
  MPUInterrupt = true;
}

void MagDataReady() {
  LISInterrupt = true;
}
