#ifndef LED_H
#define LED_H

#include <Arduino.h>

class Led {
public:
    Led(uint8_t pin);
    void init();
    void set(bool on);
    void toggle();
    bool getState() const;
    void handleBlink();

private:
    uint8_t pin;
    bool state;
    unsigned long nextLedBlinkMillis; // For blinking functionality
};

#endif