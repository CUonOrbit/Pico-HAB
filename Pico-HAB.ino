#include <Arduino.h>
#include <Wire.h>
#include "pico/cyw43_arch.h"

#include "Pico-Hab.h"
#include "Bmp.h"
#include "Mpu.h"

Bmp bmp; // Create a Bmp object
Mpu mpu; // Create a mpu object
MpuDMPData Mpu_DMP; // Create mpu data object

uint8_t MPUIntStatus;
volatile bool MPUInterrupt = false;

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

  attachInterrupt(digitalPinToInterrupt(MPU_DMP_INT_PIN), DMPDataReady, RISING);
  
  // Initialize BMP180 sensor
  if (!bmp.begin()) {
      Serial.println("Failed to initialize BMP180!");
      while (1); // Halt if initialization fails
  }

  Serial.println("BMP180 initialized!");

  if(!mpu.begin()) {
      Serial.println("Failed to initialize MPU6050!");
      while (1); // Halt if initialization fails
  }
  Serial.println("MPU6050 initialized!");
  uint8_t status = mpu.setupDMP();
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

  // read_Bmp_Sensor();

  if(MPUInterrupt) {
    mpu.getDataDMP(&Mpu_DMP);
    Mpu_DMP.print();
    MPUInterrupt = false;
  }
}

void DMPDataReady() {
  MPUInterrupt = true;
}
