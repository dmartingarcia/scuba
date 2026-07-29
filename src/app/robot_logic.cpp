#include <WiFi.h>
#include "robot_logic.h"
#include "../globals.h"
#include "sensors.h"
#include "error_reporter.h"
#include "../logic/turn_math.h"
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

// Precise turn: track yaw via the gyro until it leaves the target window.
static bool turnByGyro(int targetDegrees) {
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

  return !isYawOutsideRange(yaw, targetRange); // true = reached target, false = timed out
}

// Fallback for IMUs whose gyro isn't trustworthy (MPU9250, see
// ImuSensor::hasReliableGyro()): just spin for a fixed duration.
static bool turnByDuration() {
  logBuffer.println("Turning (fixed duration - gyro not reliable on this IMU)");
  unsigned long turnStart = millis();

  while (maxTurningMillis > (long)millis() && !isTimeElapsed(millis(), turnStart, TURN_DURATION_MS)) {
    motorAgua.setSpeed(AGUA_TURN_SPEED); // Keep turning
    delay(500); // Small delay to prevent excessive CPU usage
    motorAgua.setSpeed(AGUA_IDLE_SPEED); // Keep turning
    delay(500); // Small delay to prevent excessive CPU usage
  }

  return isTimeElapsed(millis(), turnStart, TURN_DURATION_MS); // true = completed, false = timed out
}

static void turnToDirection(int targetDegrees) {
  motorAgua.setSpeed(AGUA_IDLE_SPEED); // Start turning
  recoverFromWall();
  motorAgua.setSpeed(AGUA_IDLE_SPEED); // Keep turning

  bool completed = imu->hasReliableGyro() ? turnByGyro(targetDegrees) : turnByDuration();

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
