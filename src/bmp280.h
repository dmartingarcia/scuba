#ifndef BMP280_H
#define BMP280_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>

class BMP280 {
public:
    BMP280();
    bool init();
    void readSensor();

    float temperature;  // Temperatura em °C
    float pressure;     // Pressão em hPa

private:
    Adafruit_BMP280 bmp;
    bool initialized;
};

#endif