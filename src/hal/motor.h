#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include "../config.h"
#include "../logic/motor_direction_guard.h"

class Motor {
public:
    // Bidirectional H-bridge motor (e.g. IBT-2): RPWM/LPWM pick direction,
    // R_EN/L_EN optionally gate the outputs (see MOTOR_HAS_ENABLE_PINS).
    Motor(uint8_t rpwmPin, uint8_t lpwmPin, uint8_t rEnablePin, uint8_t lEnablePin)
        : initialized(false), speed(0), rpwmPin(rpwmPin), lpwmPin(lpwmPin),
          rEnablePin(rEnablePin), lEnablePin(lEnablePin), singleDirection(false) {};

    // Single-direction motor driven by one PWM pin (e.g. the water pump) -
    // no reverse, no enable pins, negative speed requests are floored at 0.
    explicit Motor(uint8_t pwmPin)
        : initialized(false), speed(0), rpwmPin(pwmPin), lpwmPin(0),
          rEnablePin(0), lEnablePin(0), singleDirection(true) {};

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
    bool singleDirection;
    MotorDirectionGuard directionGuard;
    int checkSpeed(int speed);
};

#endif // MOTOR_H