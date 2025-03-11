#ifndef BMP_H
#define BMP_H

#include <Wire.h>
#include <Adafruit_BMP085.h>

class Bmp {
public:
    Bmp(); // Constructor
    bool begin(); // Initialize the BMP180 sensor
    float getTemperature(); // Get temperature in °C
    float getPressure(); // Get pressure in Pa
    float getAltitude(); // Get altitude in meters

private:
    Adafruit_BMP085 bmp; // BMP180 sensor object
};

#endif