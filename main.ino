#include <Arduino.h>
#include "RP2040.h"

void setup() {
    Serial.begin(115200);  // Start serial communication
    delay(1000);           

    rp2040.wdt_begin(8000); //timeout after 8 seconds - reboot
}

void loop() {
    // Reset the watchdog timer
    rp2040.wdt_reset();

    //read from sensors

    
    //write data to storage


    //LoRa interaction

    delay(1000); 
    Serial.println("Done Loop!");
}
