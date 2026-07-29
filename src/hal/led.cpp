#include "led.h"

Led::Led(uint8_t pin) : pin(pin), state(false), nextLedBlinkMillis(0) {}

void Led::init() {
    pinMode(pin, OUTPUT);
    set(HIGH); // Start with LED on
}

void Led::set(bool newState) {
    state = newState;
    digitalWrite(pin, state ? HIGH : LOW);
}

void Led::toggle() {
    set(!state);
}

void Led::handleBlink() {
    if (millis() > nextLedBlinkMillis) {
        nextLedBlinkMillis = millis() + 1000;
        toggle();
    }
}

bool Led::getState() const {
    return state;
}