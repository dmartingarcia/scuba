#ifndef SPEED_UTILS_H
#define SPEED_UTILS_H

// Clamp a motor speed to the PWM range the H-bridge drivers accept.
inline int clampSpeed(int speed) {
    if (speed > 255) return 255;
    if (speed < -255) return -255;
    return speed;
}

#endif // SPEED_UTILS_H
