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

  if (currentState == STOPPED || currentState == MAINTENANCE) {
    // A stop was pressed (or the robot got flipped) while this blocking turn
    // was in progress - honor it instead of clobbering it with the resume
    // logic below, which would otherwise silently overrule the stop and keep
    // the robot moving.
    return;
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
    // Free heap/PSRAM ride along here so LOG_BUFFER_SIZE (config.h) can be
    // tuned from real numbers instead of guessing.
    logBuffer.println(String(millis()) + " a" + String(angle()) + " y" + String((int)yaw) + " " + resolveState(currentState) +
                       " h" + String(ESP.getFreeHeap()) + " p" + String(ESP.getFreePsram()));
  }

  // Flipped onto its back: force MAINTENANCE ahead of everything else below.
  // Terminal like STOPPED - only /control?action=start or a physical reboot
  // leaves it (enforced in web_server.cpp), never automatically here.
  // Skipped entirely while STOPPED - a manual stop means "hold here", and
  // motors are already off in that state, so there's nothing unsafe about
  // ignoring the angle too (see /control?action=stop in web_server.cpp).
  if (currentState != STOPPED) {
    if (isUpsideDown()) {
      logError(ErrorCode::UpsideDown);
      if (currentState != MAINTENANCE) {
        currentState = MAINTENANCE;
        manualControlActive = false;
        logBuffer.println("upside down -> maintenance");
      }
    } else {
      clearErrorCode(ErrorCode::UpsideDown);
    }
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

  if (!manualControlActive && (millis() - timeout) > tuning.movingTimeoutMs && currentState != STOPPED && currentState != MAINTENANCE) { // Set timeout for movement if not stopped
    if (currentState == MOVING_FORWARD) {
      currentState = MOVING_BACKWARD;
    } else {
      currentState = MOVING_FORWARD;
    }
    timeout = millis();
    logBuffer.println("timeout -> " + resolveState(currentState));
  }

  if (!manualControlActive && currentState != STOPPED && currentState != MAINTENANCE && isSessionTimeUp(millis(), sessionStartMillis, sessionDurationMs)) {
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

    case MAINTENANCE: {
      // Terminal state: only /control?action=start leaves MAINTENANCE (see
      // web_server.cpp) - staying upright again on its own doesn't resume.

      // Movement-motor ramp test (/maintenance?action=rampMovimientoForward|
      // rampMovimientoBackward): drives 0->255 (or 0->-255) over
      // MAINTENANCE_RAMP_DURATION_MS entirely within this case.
      bool movimientoTestRunning = false;
      if (movimientoRampActive) {
        unsigned long elapsed = millis() - movimientoRampStartMillis;
        if (elapsed >= MAINTENANCE_RAMP_DURATION_MS) {
          movimientoRampActive = false;
          motorMovimiento.setSpeed(0);
          logBuffer.println("movimiento ramp test done");
        } else {
          motorMovimiento.setSpeed(movimientoRampDirection * (int)(elapsed * 255 / MAINTENANCE_RAMP_DURATION_MS));
          movimientoTestRunning = true;
        }
      } else if (movimientoRunActive) {
        // Run test (/maintenance?action=runMovimientoForward|Backward): same
        // idea as the ramp above but held at a constant MAINTENANCE_TEST_SPEED
        // until ?action=stopMotorTest - see MotorTest handling in web_server.cpp.
        motorMovimiento.setSpeed(movimientoRunDirection * MAINTENANCE_TEST_SPEED);
        movimientoTestRunning = true;
      } else {
        motorMovimiento.setSpeed(0);
      }

      // Water-motor ramp test (/maintenance?action=rampAgua): drives 0->255
      // over MAINTENANCE_RAMP_DURATION_MS entirely within this case -
      // currentState never leaves MAINTENANCE for it.
      if (aguaRampActive) {
        unsigned long elapsed = millis() - aguaRampStartMillis;
        if (elapsed >= MAINTENANCE_RAMP_DURATION_MS) {
          aguaRampActive = false;
          motorAgua.setSpeed(0);
          logBuffer.println("agua ramp test done");
        } else {
          motorAgua.setSpeed((int)(elapsed * 255 / MAINTENANCE_RAMP_DURATION_MS));
        }
      } else if (aguaRunActive) {
        // Run test (/maintenance?action=runAgua): constant speed until
        // ?action=stopMotorTest.
        motorAgua.setSpeed(MAINTENANCE_TEST_SPEED);
      } else if (movimientoTestRunning) {
        // The movement motor has never been driven alone anywhere else in
        // this codebase - MOVING_FORWARD/MOVING_BACKWARD always run
        // motorAgua alongside it - and doesn't turn on its own. Match that
        // here so the forward/backward ramp/run test actually moves the robot.
        motorAgua.setSpeed(tuning.aguaMoveSpeed);
      } else {
        motorAgua.setSpeed(0);
      }
      break;
    }
  }
}
