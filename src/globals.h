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
#include "robot_state.h"

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

#endif // GLOBALS_H
