#include <Arduino.h>
#include "RP2040.h"
#include "pico/cyw43_arch.h"
#include "Bmp.h"

Bmp bmp; // Create a Bmp object

void setup() {
    Serial.begin(115200);  // Start serial communication
    delay(1000);           

    rp2040.wdt_begin(8000); // Timeout after 8 seconds - reboot

    // Initialize BMP180 sensor
    if (!bmp.begin()) {
        Serial.println("Failed to initialize BMP180!");
        while (1); // Halt if initialization fails
    }
    
    Serial.println("BMP180 initialized!");
}

void loop() {
    // Reset the watchdog timer
    rp2040.wdt_reset();

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

    // Delay before next reading
    delay(5000); 
    Serial.println("Done Loop!");
}