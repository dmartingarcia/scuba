#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <Wire.h>
#include <EEPROM.h>
#include <ArduinoOTA.h>
#include "motor.h"
#include "led.h"
#include "secrets.h"
#include "index.h" // HTML content for the web interface
#include "log_buffer.h" // Custom log buffer class for managing logs
#include "Adafruit_BMP280.h" // BMP280 sensor library
#include "MPU9250_asukiaaa.h" // MPU9250 sensor library

// Pin definitions for the motors and sensors
#define AGUA_RPWM_Output 13 // Arduino PWM output pin 5; connect to IBT-2 pin 1 (RPWM)
#define AGUA_LPWM_Output 12 // Arduino PWM output pin 6; connect to IBT-2 pin 2 (LPWM)
#define AGUA_R_ENABLE 10 // Not used in this example, but can be connected to Arduino pin 8 if needed
#define AGUA_L_ENABLE 11 // Not used in this example, but can be connected to Arduino pin 7 if needed
#define MOVIMIENTO_RPWM_Output 3 // Arduino PWM output pin 5; connect to IBT-2 pin 1 (RPWM)
#define MOVIMIENTO_LPWM_Output 2 // Arduino PWM output pin 6; connect to IBT-2 pin 2 (LPWM)
#define MOVIMIENTO_R_ENABLE 5 // Not used in this example, but can be connected to Arduino pin 8 if needed
#define MOVIMIENTO_L_ENABLE 4 // Not used in this example, but can be connected to Arduino pin 7 if needed
#define SDA_PIN 7
#define SCL_PIN 6
#define MAX_TIME_TURNING 10000
#define DELAY_UPDATING_SENSORS 100
#define DELAY_AUTOSTART 30000
#define DELAY_UPDATING_POSITION 2000 // Delay for updating position in milliseconds
#define TURN_ANGLE 15 // Degrees to turn when changing direction
#define MOVING_TIMEOUT 100000 // Timeout for movement in milliseconds
float aX, aY, aZ, aSqrt, gX, gY, gZ, mDirection, mX, mY, mZ, temp, pressure;

// Web server
AsyncWebServer server(80);

Motor motorMovimiento(MOVIMIENTO_RPWM_Output, MOVIMIENTO_LPWM_Output, MOVIMIENTO_R_ENABLE, MOVIMIENTO_L_ENABLE);
Motor motorAgua(AGUA_RPWM_Output, AGUA_LPWM_Output, AGUA_R_ENABLE, AGUA_L_ENABLE);
Led led(LED_BUILTIN);
Adafruit_BMP280 bmp; // BMP280 sensor object
MPU9250_asukiaaa mpu; // MPU9250 sensor object
LogBuffer logBuffer;

// Robot state
enum RobotState {
  MOVING_FORWARD,
  MOVING_BACKWARD,
  TURNING,
  STOPPED,
  STARTING
};
RobotState currentState = STARTING; // TODO: Initial state while debugging, should be STARTING
RobotState previousState = STARTING; // Default movement state

// Cleaning area tracking
const int GRID_SIZE = 30; // 200x200 grid
bool cleanedArea[GRID_SIZE][GRID_SIZE] = {false};
int currentX = GRID_SIZE / 2;
int currentY = GRID_SIZE / 2;
long nextTimeLogic = 1000; // For controlling logic execution frequency, startup delay of 1 second
long nextUpdate = 0; // for preventing updating sensor al the times
long timeToAutostart = DELAY_AUTOSTART; // Time to wait before starting the robot after setup
long nextPositionUpdate = 0; // For updating position every 2 seconds
long maxTurningMillis = 0;
long timeout = 0; // Timeout for turning

// Movement parameters
const float WALL_ANGLE_THRESHOLD = 45.0; // Degrees for wall detection
const float WALL_ANGLE_RECOVER_THRESHOLD = 10.0; // Degrees to consider the robot upright
const float FLOOR_INCLINATION_PRECISION = 10; // Minimum inclination to consider the robot upright
const float MOVIMIENTO_MOVE_SPEED = 100; // Speed for movement
const float MOVIMIENTO_IDLE_SPEED = 50; // Speed for turning
const float AGUA_TURN_SPEED = 255;
const float AGUA_MOVE_SPEED = 240;
const float AGUA_IDLE_SPEED = 180; // Speed when not moving

float yaw = 0;
unsigned long lastYawUpdate = 0;

String resolveState(RobotState state) {
  switch (state) {
    case MOVING_FORWARD: return "MOVING_FORWARD";
    case MOVING_BACKWARD: return "MOVING_BACKWARD";
    case TURNING: return "TURNING";
    case STOPPED: return "STOPPED";
    case STARTING: return "STARTING";
    default: return "UNKNOWN";
  }
}

void update(){
  if(nextUpdate > millis()){
    return;
  }
  nextUpdate += DELAY_UPDATING_SENSORS;

  uint8_t sensorId;
  int result;

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_OFF,
                  Adafruit_BMP280::STANDBY_MS_1);
  temp = bmp.readTemperature();
  pressure = bmp.readPressure();
  //logBuffer.println("Temperature: " + String(temp) + " C - Pressure: " + String(pressure) + " Pa");

  result = mpu.readId(&sensorId);
  if (result == 0) {
    //logBuffer.println("sensorId: " + String(sensorId));
  } else {
    logBuffer.println("Cannot read sensorId " + String(result));
  }

  result = mpu.accelUpdate();
  if (result == 0) {
    aX = mpu.accelX();
    aY = mpu.accelY();
    aZ = mpu.accelZ();
    aSqrt = mpu.accelSqrt();
    //logBuffer.println("accel - X:" + String(aX) + " Y:" + String(aY) +" Z:" + String(aZ) + " Sqrt:" + String(aSqrt));
  } else {
    logBuffer.println("Cannot read accel values " + String(result));
  }
}

void updateYaw() {
  unsigned long now = millis();
  float dt = (now - lastYawUpdate) / 1000.0; // en segundos
  lastYawUpdate = now;
  int result;

  result = mpu.gyroUpdate();
  if (result == 0) {
    gX = mpu.gyroX();
    gY = mpu.gyroY();
    gZ = mpu.gyroZ();
    //logBuffer.println("gyroX: " + String(gX) + " gyroY: " + String(gY) + " gyroZ: " + String(gZ));
  } else {
    logBuffer.println("Cannot read gyro values " + String(result));
    return; // No gyro data, cannot update yaw
  }

  result = mpu.gyroUpdate();
  if (result == 0) {
    gX = (mpu.gyroX() + gZ) / 2;
    gY = (mpu.gyroY() + gY) / 2;
    gZ = (mpu.gyroZ() + gZ) / 2;
    //logBuffer.println("gyroX: " + String(gX) + " gyroY: " + String(gY) + " gyroZ: " + String(gZ));
  } else {
    logBuffer.println("Cannot read gyro values " + String(result));
    return; // No gyro data, cannot update yaw
  }

  if (abs(gX) > 5.0) { // Threshold to avoid noise
    yaw += gX * dt; // Integración simple
  }

  // Normaliza el ángulo entre 0 y 360
  while (yaw < 0) yaw += 360;
  while (yaw >= 360) yaw -= 360;
}

float angle(){
  update();
  //if (result != 0) {
  //  mpu.beginAccel();
  //  result = mpu.accelUpdate();
  //}
  // accel - X:-1.01 Y: 0.07 Z:-0.01 Sqrt:1.02 reposo
  // accel - X:-0.05 Y:-0.04 Z:-1.04 Sqrt:1.04 subiendo de un lado
  // accel - X: 0.02 Y: 0.06 Z: 0.97 Sqrt:0.97 subiendo del otro lado
  // accel - X: 0.98 Y:-0.11 Z:-0.09 Sqrt:0.99  boca arriba
  // accel - X: 0.04 Y: 1.01 Z:-0.09 Sqrt:1.01 de canto 1
  // accel - X:-0.08 Y:-1.00 Z:-0.00 Sqrt:1.00 de canto 2

  return 90.0f * aZ ;
}

float direction(){
  return 0; // Fake MPU9250 , it doesnt have magnetometer
}

long timeToConnectWifi = 0; // Time to connect to WiFi

void setup_wifi() {
  if (timeToConnectWifi > millis()) {
    return;
  }

  logBuffer.println("Connecting to WiFi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  int retries = 40;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    logBuffer.println(".");

    retries--;
    if (retries <= 0) {
      logBuffer.println("Failed to connect to WiFi. Please check your credentials.");
      return;
    }
    timeToConnectWifi = millis() + 60000; // 60 seconds to connect
  }

  logBuffer.println("");
  logBuffer.println("WiFi connected");
  logBuffer.println("IP address: ");
  logBuffer.println(WiFi.localIP().toString());
}

void setup_ota() {
  ArduinoOTA.onStart([]() {
    motorMovimiento.setSpeed(0);
    motorAgua.setSpeed(0);
    currentState = STOPPED; // Stop motors during OTA
  });

  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      logBuffer.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      logBuffer.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      logBuffer.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      logBuffer.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      logBuffer.println("End Failed");
    }
  });

  ArduinoOTA.begin();
}

void setup_web_server() {
  // Serve main page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/html", INDEX_HTML);
  });

  // Serve logs
  server.on("/logs", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", logBuffer.get());
  });

  // Control endpoint
  server.on("/control", HTTP_GET, [](AsyncWebServerRequest *request){
    String action;
    if (request->hasParam("action")) {
      action = request->getParam("action")->value();

      if (action == "start") {
        currentState = MOVING_FORWARD;
      } else if (action == "stop") {
        currentState = STOPPED;
      } else if (action == "turn") {
        currentState = TURNING;
      }
    }
    request->send(200, "text/plain", "OK");
  });

  // Status endpoint
  server.on("/status", HTTP_GET, [](AsyncWebServerRequest *request){
    String response = "{";
    response += "\"state\":\"" + resolveState(currentState) + "\"";
    response += ",\"angle\":" + String(angle());
    response += ",\"yaw\": " + String(yaw);
    response += ",\"map\":[";

    for (int y = 0; y < GRID_SIZE; y++) {
      response += "[";
      for (int x = 0; x < GRID_SIZE; x++) {
        response += cleanedArea[y][x] ? "true" : "false";
        if (x < GRID_SIZE - 1) response += ",";
      }
      response += "]";
      if (y < GRID_SIZE - 1) response += ",";
    }

    response += "],\"x\":" + String(currentX) + ",\"y\":" + String(currentY);
    response += "}";

    request->send(200, "application/json", response);
  });

  server.begin();
}

void updatePosition() {
  if(nextPositionUpdate < millis()) return;

  nextPositionUpdate += DELAY_UPDATING_POSITION;
  int direction = 0; // Default direction is 0 (no movement)

  if(currentState == MOVING_FORWARD) {
    direction = 1;
  } else {
    direction = -1; // Moving backward
  }

  // Mark current position as cleaned
  cleanedArea[currentY][currentX] = true;

  // Update position based on orientation
  if (yaw >= 315 || yaw < 45) { // North
    currentY = std::max(0, currentY - direction);
  } else if (yaw >= 45 && yaw < 135) { // East
    currentX = std::min(GRID_SIZE - direction, currentX + direction);
  } else if (yaw >= 135 && yaw < 225) { // South
    currentY = std::min(GRID_SIZE - direction, currentY + direction);
  } else { // West
    currentX = std::max(0, currentX - direction);
  }
}

void turnToDirection(int targetDegrees) {
    float currentYaw = yaw;
    float minTargetYaw = currentYaw - targetDegrees;
    float maxTargetYaw = currentYaw + targetDegrees;

    while(maxTargetYaw > 360) {
      maxTargetYaw -= 360; // Normalize range
    }

    while(minTargetYaw < 0) {
      minTargetYaw += 360; // Normalize range
    }

    logBuffer.println("Turning to target yaw: " + String(minTargetYaw) + "- " + String(maxTargetYaw) + " from current yaw: " + String(yaw));
    motorAgua.setSpeed(AGUA_IDLE_SPEED); // Start turning

    while(maxTurningMillis > millis() && abs(angle()) > WALL_ANGLE_RECOVER_THRESHOLD) {
      if (previousState == MOVING_FORWARD) {
        motorMovimiento.setSpeed(-MOVIMIENTO_IDLE_SPEED);
      } else {
        motorMovimiento.setSpeed(MOVIMIENTO_IDLE_SPEED);
      }

      delay(100); // Small delay to prevent excessive CPU usage
      logBuffer.println("Coming back to angle: " + String(angle()));
    }

    // Stop movement motor while turning
    motorMovimiento.setSpeed(0);
    motorAgua.setSpeed(AGUA_IDLE_SPEED); // Keep turning

    while(maxTurningMillis > millis() && (std::max(minTargetYaw, maxTargetYaw) < yaw || std::min(minTargetYaw, maxTargetYaw) > yaw)) {
      updateYaw(); // Update yaw angle based on gyro data
      motorAgua.setSpeed(AGUA_TURN_SPEED); // Keep turning
      logBuffer.println("Turning... Yaw: " + String(yaw) + " Target -> min:" + String(minTargetYaw) + " - max:" + String(maxTargetYaw));
      delay(500); // Small delay to prevent excessive CPU usage
      motorAgua.setSpeed(AGUA_IDLE_SPEED); // Keep turning
      delay(500); // Small delay to prevent excessive CPU usage
    }

    motorAgua.setSpeed(AGUA_IDLE_SPEED); // Stop water motor

    if(previousState == MOVING_FORWARD) {
        currentState = MOVING_BACKWARD; // Change state to moving backward after turning
    } else {
        currentState = MOVING_FORWARD; // Change state to moving forward after turning
    }

  return ;
}

void handleWallDetection() {
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
  if (nextTimeLogic > millis()) return; // Prevent logic from running too frequently
  nextTimeLogic += 500; // Run logic every 500ms
  logBuffer.println("\n\n\n\n\n\n");
  logBuffer.println("------ Millis: " + String(millis()) + " IP address: " + WiFi.localIP().toString());
  logBuffer.println("Inclination: " + String(angle()) + " Yaw: " + String(yaw));
  logBuffer.println("Temperature: " + String(temp) + " C" + " Pressure: " + String(pressure) + " Pa");
  logBuffer.println("Previous Movement: " + resolveState(previousState) + " Current State: " + resolveState(currentState));

  if ((millis() - timeout) > MOVING_TIMEOUT){ // Set timeout for movement in 100 seconds
    if (currentState == MOVING_FORWARD) {
      // Handle timeout for moving forward
      currentState = MOVING_BACKWARD;
    } else {
      currentState = MOVING_FORWARD;
    }
    timeout = millis();
    logBuffer.println("Movement timeout reached, changing state to: " + resolveState(currentState
  }

  switch (currentState) {
    case STARTING:
      logBuffer.println("Starting robot...");
      // read mpu and check if robot is moving, and wait until it is upright
      if(angle() > FLOOR_INCLINATION_PRECISION) {
        timeToAutostart = millis() + DELAY_AUTOSTART;
        logBuffer.println("Robot is not upright, waiting to start...");
        return;
      }

      if (millis() < timeToAutostart) {
        logBuffer.println("Robot is upright, starting in " + String((timeToAutostart - millis()) / 1000) + " seconds");
        motorAgua.setSpeed(AGUA_IDLE_SPEED/2);
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
      turnToDirection(TURN_ANGLE); // turn 5 degrees
      break;

    case STOPPED:
      motorMovimiento.setSpeed(0);
      motorAgua.setSpeed(0);
      break;
  }
}

void setup()
{
  Serial.begin(9600); // Initialize serial communication for debugging
  setup_wifi(); // Connect to WiFi
  setup_ota(); // Setup OTA updates
  setup_web_server(); // Setup web server

  led.init();
  motorMovimiento.init(); // Initialize movement motor
  motorAgua.init(); // Initialize water motor

  Wire.begin(SDA_PIN, SCL_PIN);
  mpu.setWire(&Wire);

  if (!bmp.begin(0x76)) {
    logBuffer.println("Could not find a valid BMP280 sensor, check wiring!");
  }

  mpu.beginAccel();
  mpu.beginGyro();
}

void loop() {
  // Handle OTA updates and robot logic
  ArduinoOTA.handle();
  robotLogic();
  updatePosition();
  updateYaw(); // Update yaw angle based on gyro data
  led.handleBlink();
}