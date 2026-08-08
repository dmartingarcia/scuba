#include <WiFi.h>
#include <esp_task_wdt.h>
#include "robot_logic.h"
#include "../globals.h"
#include "sensors.h"
#include "error_reporter.h"
#include "turn_controller.h"
#include "../logic/session_timer.h"

static void recoverFromWall() {
  while (maxTurningMillis > (long)millis() && abs(angle()) > tuning.wallAngleRecoverThreshold) {
    if (previousState == MOVING_FORWARD) {
      motorMovimiento.setSpeed(-tuning.movimientoIdleSpeed);
    } else {
      motorMovimiento.setSpeed(tuning.movimientoIdleSpeed);
    }

    delay(100); // Small delay to prevent excessive CPU usage
    esp_task_wdt_reset(); // this loop can legitimately run for seconds - keep feeding the watchdog
  }
  motorMovimiento.setSpeed(0);
  logBuffer.println("rec a" + String(angle()));
}

static void turnToDirection(int targetDegrees) {
  motorAgua.setSpeed(tuning.aguaIdleSpeed); // Start turning
  recoverFromWall();
  motorAgua.setSpeed(tuning.aguaIdleSpeed); // Keep turning

  bool completed = turnControllerFor(turnStrategy).turn(targetDegrees);

  motorAgua.setSpeed(tuning.aguaIdleSpeed); // Stop water motor

  if (completed) {
    clearErrorCode(ErrorCode::TurnTimeout);
  } else {
    logError(ErrorCode::TurnTimeout);
  }

  if (previousState == MOVING_FORWARD) {
    currentState = MOVING_BACKWARD; // Autonomous wall-avoidance: reverse away from it
  } else if (previousState == MOVING_BACKWARD) {
    currentState = MOVING_FORWARD; // Autonomous wall-avoidance: reverse away from it
  } else {
    currentState = previousState; // Manual turn from a non-moving state (e.g. STOPPED) - just go back to it
  }
}

static void handleWallDetection() {
  if (angle() > tuning.wallAngleThreshold || angle() < -tuning.wallAngleThreshold) {
    // Wall detected, stop and turn. This safety response always wins over a
    // manual forward/backward pulse in progress - drop it rather than let
    // its revert timer fight the turn afterwards.
    motorMovimiento.setSpeed(0);
    delay(500);
    previousState = currentState;
    currentState = TURNING;
    maxTurningMillis = millis() + tuning.maxTimeTurningMs;
    timeout = millis();
    manualControlActive = false;
  }
}

void robotLogic() {
  if (nextTimeLogic > (long)millis()) return; // Prevent logic from running too frequently
  nextTimeLogic += 500; // Run logic every 500ms

  if (nextTimeLogUpdate <= (long)millis()) { // Heartbeat status line - kept short, buffer is all we have once submerged
    nextTimeLogUpdate += 5000; // Run every 5 seconds
    logBuffer.println(String(millis()) + " a" + String(angle()) + " y" + String((int)yaw) + " " + resolveState(currentState));
  }

  // Manual forward/backward pulse (/control?action=forward|backward): one
  // request drives the robot for tuning.manualActionDurationMs, then this
  // restores manualRevertState (whatever currentState was right before the
  // pulse, captured in web_server.cpp) - entirely server-side.
  if (manualControlActive && (long)millis() > manualActionDeadlineMillis) {
    currentState = manualRevertState;
    manualControlActive = false;
    logBuffer.println("manual pulse done -> " + resolveState(currentState));
  }

  if (!manualControlActive && (millis() - timeout) > tuning.movingTimeoutMs && currentState != STOPPED) { // Set timeout for movement if not stopped
    if (currentState == MOVING_FORWARD) {
      currentState = MOVING_BACKWARD;
    } else {
      currentState = MOVING_FORWARD;
    }
    timeout = millis();
    logBuffer.println("timeout -> " + resolveState(currentState));
  }

  if (!manualControlActive && currentState != STOPPED && isSessionTimeUp(millis(), sessionStartMillis, sessionDurationMs)) {
    logBuffer.println("session done, stopping");
    currentState = STOPPED;
    sessionCompletedByTimer = true;
  }

  switch (currentState) {
    case STARTING:
      // read mpu and check if robot is moving, and wait until it is upright
      if (angle() > tuning.floorInclinationPrecision) {
        timeToAutostart = millis() + tuning.delayAutostartMs;
        motorAgua.setSpeed(tuning.aguaIdleSpeed / 2);
        return;
      }

      if (millis() < (unsigned long)timeToAutostart) {
        motorAgua.setSpeed(tuning.aguaIdleSpeed / 2);
        return;
      }

      motorAgua.setSpeed(tuning.aguaTurnSpeed); // Stop water motor
      motorMovimiento.setSpeed(10); // Start moving forward slowly
      delay(1000); // Allow some time to start moving
      motorAgua.setSpeed(tuning.aguaMoveSpeed); // Set water motor speed
      delay(1000); // Allow some time to start moving
      motorAgua.setSpeed(tuning.aguaIdleSpeed); // Stop water motor
      delay(1000); // Allow some time to start moving

      currentState = MOVING_FORWARD;
      sessionStartMillis = millis();
      sessionCompletedByTimer = false;
      logBuffer.println("started");
      break;

    case MOVING_FORWARD:
      motorMovimiento.setSpeed(tuning.movimientoMoveSpeed);
      motorAgua.setSpeed(tuning.aguaMoveSpeed);
      handleWallDetection();
      break;

    case MOVING_BACKWARD:
      motorMovimiento.setSpeed(-tuning.movimientoMoveBackwardsSpeed);
      motorAgua.setSpeed(tuning.aguaMoveSpeed);
      handleWallDetection();
      break;

    case TURNING:
      turnToDirection(tuning.turnAngleDeg);
      break;

    case STOPPED:
      // Terminal state: only /control?action=... (a manual command) may leave
      // STOPPED. Don't add angle/leveling/timeout checks here that resume
      // movement on their own.
      motorMovimiento.setSpeed(0);
      motorAgua.setSpeed(0);
      break;
  }
}
