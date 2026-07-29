#ifndef TURN_MATH_H
#define TURN_MATH_H

struct YawRange {
    float min;
    float max;
};

inline float normalizeAngle360(float angle) {
    while (angle < 0) angle += 360;
    while (angle >= 360) angle -= 360;
    return angle;
}

// Yaw window to turn into: currentYaw +/- targetDegrees, wrapped to [0, 360).
inline YawRange computeTurnRange(float currentYaw, float targetDegrees) {
    float minYaw = normalizeAngle360(currentYaw - targetDegrees);
    float maxYaw = normalizeAngle360(currentYaw + targetDegrees);
    return {minYaw, maxYaw};
}

inline bool isYawOutsideRange(float yaw, const YawRange& range) {
    float lo = range.min < range.max ? range.min : range.max;
    float hi = range.min < range.max ? range.max : range.min;
    return yaw < lo || yaw > hi;
}

#endif // TURN_MATH_H
