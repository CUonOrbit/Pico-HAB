#include "Bmp.h"

// Constructor
Bmp::Bmp() {
    // Nothing to do here
}

// Initialize the BMP180 sensor
bool Bmp::begin() {
    if (!bmp.begin()) {
        Serial.println("BMP180 not detected!");
        return false;
    }
    return true;
}

// Get temperature in °C
float Bmp::getTemperature() {
    return bmp.readTemperature();
}

// Get pressure in Pa
float Bmp::getPressure() {
    return bmp.readPressure();
}

// Get altitude in meters
float Bmp::getAltitude() {
    return bmp.readAltitude();
}