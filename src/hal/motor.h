#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

class Motor {
public:
    Motor(uint8_t rpwmPin, uint8_t lpwmPin, uint8_t rEnablePin, uint8_t lEnablePin)
        : initialized(false), speed(0), fakeDisabled(false), rpwmPin(rpwmPin), lpwmPin(lpwmPin),
          rEnablePin(rEnablePin), lEnablePin(lEnablePin) {};
    bool init();
    void setSpeed(int newSpeed);
    int getSpeed() const;

    // Diagnostic override: while true, setSpeed() still tracks the
    // requested speed (getSpeed() and every caller's own logging keep
    // reporting as normal) but never touches the physical pins - lets a
    // motor be tested "as if" running without actually powering it.
    void setFakeDisabled(bool disabled);
    bool isFakeDisabled() const;

private:
    bool initialized;
    int speed;
    bool fakeDisabled;
    uint8_t rpwmPin;
    uint8_t lpwmPin;
    uint8_t rEnablePin;
    uint8_t lEnablePin;
    int checkSpeed(int speed);
};

#endif // MOTOR_H