#include "turn_controller.h"
#include "../globals.h"
#include "sensors.h"
#include "../logic/turn_math.h"
#include "../logic/kalman_filter.h"

// Raw gyro-yaw tracking - the original behavior, unchanged.
class LegacyGyroTurn : public TurnController {
public:
    bool turn(int targetDegrees) override {
        YawRange targetRange = computeTurnRange(yaw, targetDegrees);
        logBuffer.println("Turning (gyro) to target yaw: " + String(targetRange.min) + "- " + String(targetRange.max) + " from current yaw: " + String(yaw));

        while (maxTurningMillis > (long)millis() && isYawOutsideRange(yaw, targetRange)) {
            updateYaw(); // Update yaw angle based on gyro data
            motorAgua.setSpeed(AGUA_TURN_SPEED); // Keep turning
            logBuffer.println("Turning... Yaw: " + String(yaw) + " Target -> min:" + String(targetRange.min) + " - max:" + String(targetRange.max));
            delay(500); // Small delay to prevent excessive CPU usage
            motorAgua.setSpeed(AGUA_IDLE_SPEED); // Keep turning
            delay(500); // Small delay to prevent excessive CPU usage
        }

        return !isYawOutsideRange(yaw, targetRange);
    }

    const char* name() const override { return "legacy"; }
};

// Fallback for IMUs whose gyro isn't trustworthy (MPU9250, see
// ImuSensor::hasReliableGyro()): just spin for a fixed duration.
class FixedDurationTurn : public TurnController {
public:
    bool turn(int targetDegrees) override {
        (void)targetDegrees; // Not yaw-aware - pulses for a flat calibrated duration
        logBuffer.println("Turning (fixed duration - gyro not reliable on this IMU)");
        unsigned long turnStart = millis();

        while (maxTurningMillis > (long)millis() && !isTimeElapsed(millis(), turnStart, TURN_DURATION_MS)) {
            motorAgua.setSpeed(AGUA_TURN_SPEED); // Keep turning
            delay(500); // Small delay to prevent excessive CPU usage
            motorAgua.setSpeed(AGUA_IDLE_SPEED); // Keep turning
            delay(500); // Small delay to prevent excessive CPU usage
        }

        return isTimeElapsed(millis(), turnStart, TURN_DURATION_MS);
    }

    const char* name() const override { return "duration"; }
};

// Gyro-yaw tracking with a Kalman-smoothed rate instead of the raw/naive
// double-read average LegacyGyroTurn (via updateYaw()) uses. Trades noise
// for latency - doesn't fix long-term drift, since there's no absolute yaw
// reference on this hardware to correct against (see logic/kalman_filter.h).
class KalmanGyroTurn : public TurnController {
public:
    bool turn(int targetDegrees) override {
        YawRange targetRange = computeTurnRange(yaw, targetDegrees);
        logBuffer.println("Turning (Kalman gyro) to target yaw: " + String(targetRange.min) + "- " + String(targetRange.max) + " from current yaw: " + String(yaw));

        KalmanState kalman = kalmanInit(0.0f);
        unsigned long lastSample = millis();

        while (maxTurningMillis > (long)millis() && isYawOutsideRange(yaw, targetRange)) {
            motorAgua.setSpeed(AGUA_TURN_SPEED); // Keep turning

            float gxRaw, gyRaw, gzRaw;
            if (imu->readGyro(gxRaw, gyRaw, gzRaw)) {
                unsigned long now = millis();
                float dt = (now - lastSample) / 1000.0f;
                lastSample = now;

                kalman = kalmanUpdate(kalman, gxRaw, KALMAN_PROCESS_NOISE, KALMAN_MEASUREMENT_NOISE);
                if (abs(kalman.rate) > 5.0) { // Same noise threshold as updateYaw()
                    yaw += kalman.rate * dt;
                }
                while (yaw < 0) yaw += 360;
                while (yaw >= 360) yaw -= 360;
            }

            logBuffer.println("Turning... Yaw: " + String(yaw) + " (filtered rate " + String(kalman.rate) + ") Target -> min:" + String(targetRange.min) + " - max:" + String(targetRange.max));
            delay(500); // Small delay to prevent excessive CPU usage
            motorAgua.setSpeed(AGUA_IDLE_SPEED); // Keep turning
            delay(500); // Small delay to prevent excessive CPU usage
        }

        return !isYawOutsideRange(yaw, targetRange);
    }

    const char* name() const override { return "kalman"; }
};

TurnController& turnControllerFor(TurnStrategy strategy) {
    static LegacyGyroTurn legacy;
    static FixedDurationTurn fixedDuration;
    static KalmanGyroTurn kalmanFusion;

    switch (strategy) {
        case TurnStrategy::FixedDuration: return fixedDuration;
        case TurnStrategy::KalmanFusion: return kalmanFusion;
        case TurnStrategy::Legacy:
        default: return legacy;
    }
}
