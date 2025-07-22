#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

class Motor {
public:
    Motor(uint8_t rpwmPin, uint8_t lpwmPin, uint8_t rEnablePin, uint8_t lEnablePin)
        : initialized(false), speed(0), rpwmPin(rpwmPin), lpwmPin(lpwmPin),
          rEnablePin(rEnablePin), lEnablePin(lEnablePin) {};
    bool init();
    void setSpeed(int newSpeed);
    int getSpeed() const;

private:
    bool initialized;
    int speed;
    uint8_t rpwmPin;
    uint8_t lpwmPin;
    uint8_t rEnablePin;
    uint8_t lEnablePin;
    uint8_t pwmPin;
    int checkSpeed(int speed);
};

#endif // MOTOR_H