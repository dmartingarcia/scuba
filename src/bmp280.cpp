#include <Wire.h>
#include "bmp280.h"
#include "config.h"
#include <Adafruit_BMP280.h>
#define BMP280_ADDR 0x76

BMP280::BMP280() : initialized(false), temperature(0), pressure(0) {}

bool BMP280::init() {
    if (!bmp.begin(BMP280_ADDR)) {
        return false;
    }

    // Configuração do sensor BMP280
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     // Modo de operação
                    Adafruit_BMP280::SAMPLING_X2,     // Sobreamostragem de temperatura
                    Adafruit_BMP280::SAMPLING_X16,    // Sobreamostragem de pressão
                    Adafruit_BMP280::FILTER_X16,      // Filtragem
                    Adafruit_BMP280::STANDBY_MS_500); // Tempo de espera

    initialized = true;
    return true;
}

void BMP280::readSensor() {
    if (!initialized) return;

    temperature = bmp.readTemperature();
    pressure = bmp.readPressure() / 100.0F; // Convertendo Pa para hPa
}