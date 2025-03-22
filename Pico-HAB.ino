#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
// #include "RP2040.h"
#include "pico/cyw43_arch.h"

#include "Pico-Hab.h"
#include "Bmp.h"
#include "Mpu.h"

Bmp bmp; // Create a Bmp object
Mpu mpu; // Create a mpu object
Mpu6050_Data mpu_data; // Create mpu data object

void setup() {
    Wire.setSDA(0);  // Pico SDA on GP0
    Wire.setSCL(1);  // Pico SCL on GP1
    Wire.begin();

    Serial.begin(115200);  // Start serial communication
    delay(1000);           

    rp2040.wdt_begin(8000); // Timeout after 8 seconds - reboot

    pinMode(LED_PIN, OUTPUT);
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

void read_Mpu_Sensor() {
  // Read data from BMP180 sensor
  mpu.getData_Raw(&mpu_data);

  Serial.print("Accel: ");
  Serial.print("X="); Serial.print(mpu_data.accel_Raw[0]);
  Serial.print(" Y="); Serial.print(mpu_data.accel_Raw[1]);
  Serial.print(" Z="); Serial.print(mpu_data.accel_Raw[2]);

  Serial.print(" | Gyro: ");
  Serial.print("X="); Serial.print(mpu_data.gyro_Raw[0]);
  Serial.print(" Y="); Serial.print(mpu_data.gyro_Raw[1]);
  Serial.print(" Z="); Serial.println(mpu_data.gyro_Raw[2]);
}

void loop() {
  // Reset the watchdog timer
  rp2040.wdt_reset();

  read_Bmp_Sensor();
  read_Mpu_Sensor();

  delay(500);
  Serial.println("Done Loop!");
}

void RxI2C(int num_bytes) {
  digitalWrite(LED_PIN, HIGH);
}

void TxI2C() {
  digitalWrite(LED_PIN, HIGH);
}