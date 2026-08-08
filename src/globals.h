#ifndef GLOBALS_H
#define GLOBALS_H

#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include "Adafruit_BMP280.h"
#include "config.h"
#include "hal/motor.h"
#include "hal/led.h"
#include "hal/imu.h"
#include "log_buffer.h"
#include "app/robot_state.h"
#include "logic/maintenance_stats.h"
#include "logic/error_log.h"
#include "logic/turn_strategy.h"
#include "logic/accel_calibration.h"
#include "logic/tuning_params.h"
#include "app/mqtt_config.h"

// Shared robot state, defined once in globals.cpp. Every module that reads
// or drives the robot (sensors, robot_logic, position_tracker, web_server,
// wifi_manager) includes this instead of duplicating state locally.

extern AsyncWebServer server;

extern Motor motorMovimiento;
extern Motor motorAgua;
extern Led led;
extern Adafruit_BMP280 bmp;
extern ImuSensor* imu; // set once detectAndBeginImu() runs in setup()
extern LogBuffer logBuffer;

extern RobotState currentState;
extern RobotState previousState;

extern bool cleanedArea[GRID_SIZE][GRID_SIZE];
extern int currentX;
extern int currentY;

extern long nextTimeLogic;
extern long nextTimeLogUpdate;
extern long nextUpdate;
extern long timeToAutostart;
extern long nextPositionUpdate;
extern long maxTurningMillis;
extern long timeout;
extern long timeToConnectWifi;

// Manual one-shot pulse (see /control?action=forward|backward in
// web_server.cpp): a single request drives the robot for
// tuning.manualActionDurationMs, then robotLogic() automatically restores
// manualRevertState (whatever currentState was right before the pulse) -
// entirely server-side, no further requests needed from the caller.
// action=turn uses the same previousState mechanism the autonomous
// wall-avoidance turn already has (see turnToDirection()), not this pulse.
extern bool manualControlActive;
extern long manualActionDeadlineMillis;
extern RobotState manualRevertState;

extern float aX, aY, aZ, aSqrt, gX, gY, gZ, temp, pressure;
extern float yaw;
extern unsigned long lastYawUpdate;

// Configurable cleaning session: how long a run should last (0 = unlimited)
// and when the current run started. Set via the /control web API.
extern unsigned long sessionDurationMs;
extern unsigned long sessionStartMillis;

// How often maintenance stats get persisted to EEPROM (0 = only on session
// end). Configurable via the /config web API.
extern unsigned long statsSaveIntervalMs;
extern MaintenanceStats maintenanceStats;

// ECU-style fault log: persisted, deduped while a fault stays active.
extern ErrorLog errorLog;

// Which turn strategy to use - defaulted per detected IMU in setup(),
// overridable at runtime via /config?turnStrategy=legacy|duration|kalman
extern TurnStrategy turnStrategy;

// Accelerometer zero-offset (corrects a not-perfectly-level IMU mount),
// persisted to LittleFS. See app/accel_calibration_store.h.
extern AccelCalibration accelCalibration;

// Live motor/angle/timing tuning, persisted to LittleFS. config.h's
// DEFAULT_* constants are only the seed values - see logic/tuning_params.h
// and app/tuning_store.h. Overridable at runtime via /tuning.
extern TuningParams tuning;

// MQTT / Home Assistant integration
extern MqttConfig mqttConfig;
// True if the current STOPPED state was reached because the configured
// session duration ran out (vs. a manual stop) - lets MQTT report
// "finished" instead of "paused". See logic/ha_state.h.
extern bool sessionCompletedByTimer;

#endif // GLOBALS_H
