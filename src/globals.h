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
extern long nextUpdate;
extern long timeToAutostart;
extern long nextPositionUpdate;
extern long maxTurningMillis;
extern long timeout;
extern long timeToConnectWifi;

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

#endif // GLOBALS_H
