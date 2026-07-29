#include <WiFi.h>
#include "robot_logic.h"
#include "../globals.h"
#include "sensors.h"
#include "error_reporter.h"
#include "turn_controller.h"
#include "../logic/session_timer.h"

static void recoverFromWall() {
  while (maxTurningMillis > (long)millis() && abs(angle()) > WALL_ANGLE_RECOVER_THRESHOLD) {
    if (previousState == MOVING_FORWARD) {
      motorMovimiento.setSpeed(-MOVIMIENTO_IDLE_SPEED);
    } else {
      motorMovimiento.setSpeed(MOVIMIENTO_IDLE_SPEED);
    }

    delay(100); // Small delay to prevent excessive CPU usage
    logBuffer.println("Coming back to angle: " + String(angle()));
  }
  motorMovimiento.setSpeed(0);
}

static void turnToDirection(int targetDegrees) {
  motorAgua.setSpeed(AGUA_IDLE_SPEED); // Start turning
  recoverFromWall();
  motorAgua.setSpeed(AGUA_IDLE_SPEED); // Keep turning

  bool completed = turnControllerFor(turnStrategy).turn(targetDegrees);

  motorAgua.setSpeed(AGUA_IDLE_SPEED); // Stop water motor

  if (completed) {
    clearErrorCode(ErrorCode::TurnTimeout);
  } else {
    logError(ErrorCode::TurnTimeout);
  }

  if (previousState == MOVING_FORWARD) {
    currentState = MOVING_BACKWARD; // Change state to moving backward after turning
  } else {
    currentState = MOVING_FORWARD; // Change state to moving forward after turning
  }
}

static void handleWallDetection() {
  if (angle() > WALL_ANGLE_THRESHOLD || angle() < -WALL_ANGLE_THRESHOLD) {
    // Wall detected, stop and turn
    motorMovimiento.setSpeed(0);
    delay(500);
    previousState = currentState;
    currentState = TURNING;
    maxTurningMillis = millis() + MAX_TIME_TURNING;
    timeout = millis();
  }
}

void robotLogic() {
  if (nextTimeLogic > (long)millis()) return; // Prevent logic from running too frequently
  nextTimeLogic += 500; // Run logic every 500ms
  logBuffer.println("\n\n\n\n\n\n");
  logBuffer.println("------ Millis: " + String(millis()) + " IP address: " + WiFi.localIP().toString());
  logBuffer.println("Inclination: " + String(angle()) + " Yaw: " + String(yaw));
  logBuffer.println("Temperature: " + String(temp) + " C" + " Pressure: " + String(pressure) + " Pa");
  logBuffer.println("Previous Movement: " + resolveState(previousState) + " Current State: " + resolveState(currentState));

  if ((millis() - timeout) > MOVING_TIMEOUT) { // Set timeout for movement in 100 seconds
    if (currentState == MOVING_FORWARD) {
      currentState = MOVING_BACKWARD;
    } else {
      currentState = MOVING_FORWARD;
    }
    timeout = millis();
    logBuffer.println("Movement timeout reached, changing state to: " + resolveState(currentState));
  }

  if (currentState != STOPPED && isSessionTimeUp(millis(), sessionStartMillis, sessionDurationMs)) {
    logBuffer.println("Configured session duration reached, stopping.");
    currentState = STOPPED;
  }

  switch (currentState) {
    case STARTING:
      logBuffer.println("Starting robot...");
      // read mpu and check if robot is moving, and wait until it is upright
      if (angle() > FLOOR_INCLINATION_PRECISION) {
        timeToAutostart = millis() + DELAY_AUTOSTART;
        logBuffer.println("Robot is not upright, waiting to start...");
        return;
      }

      if (millis() < (unsigned long)timeToAutostart) {
        logBuffer.println("Robot is upright, starting in " + String((timeToAutostart - millis()) / 1000) + " seconds");
        motorAgua.setSpeed(AGUA_IDLE_SPEED / 2);
        return;
      }

      motorAgua.setSpeed(AGUA_TURN_SPEED); // Stop water motor
      motorMovimiento.setSpeed(10); // Start moving forward slowly
      delay(1000); // Allow some time to start moving
      motorAgua.setSpeed(AGUA_MOVE_SPEED); // Set water motor speed
      delay(1000); // Allow some time to start moving
      motorAgua.setSpeed(AGUA_IDLE_SPEED); // Stop water motor
      delay(1000); // Allow some time to start moving

      currentState = MOVING_FORWARD;
      sessionStartMillis = millis();
      logBuffer.println("Robot started and ready to move.");
      break;

    case MOVING_FORWARD:
      motorMovimiento.setSpeed(MOVIMIENTO_MOVE_SPEED);
      motorAgua.setSpeed(AGUA_MOVE_SPEED);
      handleWallDetection();
      break;

    case MOVING_BACKWARD:
      motorMovimiento.setSpeed(-MOVIMIENTO_MOVE_SPEED);
      motorAgua.setSpeed(AGUA_MOVE_SPEED);
      handleWallDetection();
      break;

    case TURNING:
      turnToDirection(TURN_ANGLE);
      break;

    case STOPPED:
      motorMovimiento.setSpeed(0);
      motorAgua.setSpeed(0);
      break;
  }
}
