#include <esp_task_wdt.h>
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

        while (maxTurningMillis > (long)millis() && isYawOutsideRange(yaw, targetRange)) {
            updateYaw(); // Update yaw angle based on gyro data
            motorAgua.setSpeed(tuning.aguaTurnSpeed); // Keep turning
            delay(500); // Small delay to prevent excessive CPU usage
            motorAgua.setSpeed(tuning.aguaIdleSpeed); // Keep turning
            delay(500); // Small delay to prevent excessive CPU usage
            esp_task_wdt_reset(); // this loop can legitimately run for seconds - keep feeding the watchdog
        }

        bool completed = !isYawOutsideRange(yaw, targetRange);
        logBuffer.println("turn(gyro) y" + String((int)yaw) + " tgt" + String(targetRange.min) + "-" + String(targetRange.max) + (completed ? " ok" : " FAIL"));
        return completed;
    }

    const char* name() const override { return "legacy"; }
};

// Fallback for IMUs whose gyro isn't trustworthy (MPU9250, see
// ImuSensor::hasReliableGyro()): just spin for a fixed duration.
class FixedDurationTurn : public TurnController {
public:
    bool turn(int targetDegrees) override {
        (void)targetDegrees; // Not yaw-aware - pulses for a flat calibrated duration
        unsigned long turnStart = millis();

        while (maxTurningMillis > (long)millis() && !isTimeElapsed(millis(), turnStart, tuning.turnDurationMs)) {
            motorAgua.setSpeed(tuning.aguaTurnSpeed); // Keep turning
            delay(500); // Small delay to prevent excessive CPU usage
            motorAgua.setSpeed(tuning.aguaIdleSpeed); // Keep turning
            delay(500); // Small delay to prevent excessive CPU usage
            esp_task_wdt_reset(); // this loop can legitimately run for seconds - keep feeding the watchdog
        }

        bool completed = isTimeElapsed(millis(), turnStart, tuning.turnDurationMs);
        logBuffer.println(String("turn(duration)") + (completed ? " ok" : " FAIL"));
        return completed;
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

        KalmanState kalman = kalmanInit(0.0f);
        unsigned long lastSample = millis();

        while (maxTurningMillis > (long)millis() && isYawOutsideRange(yaw, targetRange)) {
            motorAgua.setSpeed(tuning.aguaTurnSpeed); // Keep turning

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

            delay(500); // Small delay to prevent excessive CPU usage
            motorAgua.setSpeed(tuning.aguaIdleSpeed); // Keep turning
            delay(500); // Small delay to prevent excessive CPU usage
            esp_task_wdt_reset(); // this loop can legitimately run for seconds - keep feeding the watchdog
        }

        bool completed = !isYawOutsideRange(yaw, targetRange);
        logBuffer.println("turn(kalman) y" + String((int)yaw) + " tgt" + String(targetRange.min) + "-" + String(targetRange.max) + (completed ? " ok" : " FAIL"));
        return completed;
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
