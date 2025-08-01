#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <Wire.h>
#include <EEPROM.h>
#include <ArduinoOTA.h>
#include "motor.h"
#include "led.h"
#include "mpu9250.h"
#include "bmp280.h"
#include "secrets.h"
#include "index.h" // HTML content for the web interface
#include "log_buffer.h" // Custom log buffer class for managing logs

// Pin definitions for the motors and sensors
#define AGUA_RPWM_Output 13 // Arduino PWM output pin 5; connect to IBT-2 pin 1 (RPWM)
#define AGUA_LPWM_Output 12 // Arduino PWM output pin 6; connect to IBT-2 pin 2 (LPWM)
#define AGUA_R_ENABLE 10 // Not used in this example, but can be connected to Arduino pin 8 if needed
#define AGUA_L_ENABLE 11 // Not used in this example, but can be connected to Arduino pin 7 if needed
#define MOVIMIENTO_RPWM_Output 3 // Arduino PWM output pin 5; connect to IBT-2 pin 1 (RPWM)
#define MOVIMIENTO_LPWM_Output 2 // Arduino PWM output pin 6; connect to IBT-2 pin 2 (LPWM)
#define MOVIMIENTO_R_ENABLE 5 // Not used in this example, but can be connected to Arduino pin 8 if needed
#define MOVIMIENTO_L_ENABLE 4 // Not used in this example, but can be connected to Arduino pin 7 if needed
#define I2C_SDA 7
#define I2C_SCL 4

// Web server
AsyncWebServer server(80);

Motor motorMovimiento(MOVIMIENTO_RPWM_Output, MOVIMIENTO_LPWM_Output, MOVIMIENTO_R_ENABLE, MOVIMIENTO_L_ENABLE);
Motor motorAgua(AGUA_RPWM_Output, AGUA_LPWM_Output, AGUA_R_ENABLE, AGUA_L_ENABLE);
Led led(LED_BUILTIN);
MPU9250 mpu(I2C_SDA, I2C_SCL);
BMP280 bmp;
LogBuffer logBuffer;

// Robot state
enum RobotState {
  MOVING_FORWARD,
  MOVING_BACKWARD,
  TURNING,
  STOPPED,
  STARTING
};
RobotState currentState = STOPPED;

// Cleaning area tracking
const int GRID_SIZE = 20; // 20x20 grid
bool cleanedArea[GRID_SIZE][GRID_SIZE] = {false};
int currentX = GRID_SIZE / 2;
int currentY = GRID_SIZE / 2;
int targetDirection = 0; // 0 = North, 90 = East, 180 = South, 270 = West
long nextTimeLogic = 1000; // For controlling logic execution frequency, startup delay of 1 second

// Movement parameters
const float WALL_ANGLE_THRESHOLD = 30.0; // Degrees for wall detection
const float FLOOR_INCLINATION_PRECISION = 10; // Minimum inclination to consider the robot upright

const float MOVIMIENTO_MOVE_SPEED = 200; // Speed for movement
const float MOVIMIENTO_TURN_SPEED = 150; // Speed for turning
const float AGUA_TURN_SPEED = 250;
const float AGUA_MOVE_SPEED = 150;
const float AGUA_IDLE_SPEED = 50; // Speed when not moving
const float TURN_PRECISION = 5.0; // Degrees

void setup_wifi() {
  logBuffer.println("Connecting to WiFi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  int retries = 40;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");

    retries--;
    if (retries <= 0) {
      logBuffer.println("Failed to connect to WiFi. Please check your credentials.");
      return;
    }
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
    request->send_P(200, "text/html", INDEX_HTML);
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
      }
    }
    request->send(200, "text/plain", "OK");
  });

  // Status endpoint
  server.on("/status", HTTP_GET, [](AsyncWebServerRequest *request){
    String response = "{";
    response += "\"state\":\"";

    switch (currentState) {
      case MOVING_FORWARD:
        response += "Moving Forward";
        break;
      case MOVING_BACKWARD:
        response += "Moving Backward";
        break;
      case TURNING:
        response += "Turning";
        break;
      case STOPPED:
        response += "Stopped";
        break;
    }

    response += "\",\"yaw\":" + String(mpu.getInclination()) + ",";
    response += "\"map\":[";

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
  // Mark current position as cleaned
  cleanedArea[currentY][currentX] = true;

  // Update position based on orientation
  float direction = mpu.getMagDirection();

  if (direction >= 315 || direction < 45) { // North
    currentY = max(0, currentY - 1);
  } else if (direction >= 45 && direction < 135) { // East
    currentX = min(GRID_SIZE - 1, currentX + 1);
  } else if (direction >= 135 && direction < 225) { // South
    currentY = min(GRID_SIZE - 1, currentY + 1);
  } else { // West
    currentX = max(0, currentX - 1);
  }
}

void turnToDirection(int targetDegrees) {
  currentState = TURNING;

  // Normalize target to 0-360
  while (targetDegrees < 0) targetDegrees += 360;
  while (targetDegrees >= 360) targetDegrees -= 360;

  // Calculate shortest turn direction
  float diff = targetDegrees - mpu.getMagDirection();
  if (diff > 180) diff -= 360;
  if (diff < -180) diff += 360;

  if (abs(diff) <= TURN_PRECISION) {
    // Already at target
    motorAgua.setSpeed(0);
    return;
  }

  if (diff > 0) {
    // Turn clockwise
    motorAgua.setSpeed(AGUA_TURN_SPEED);
  } else {
    // Turn counter-clockwise
    motorAgua.setSpeed(-AGUA_TURN_SPEED);
  }
}

void handleWallDetection() {
  if (mpu.getInclination() > WALL_ANGLE_THRESHOLD) {
    // Wall detected, stop and turn
    motorMovimiento.setSpeed(0);
    delay(500);

    // Choose a random new direction
    int newDirection = random(0, 4) * 90; // 0, 90, 180, or 270

    // Turn to the new direction
    while (abs(mpu.getMagZ() - newDirection) > TURN_PRECISION) {
      mpu.update();
      turnToDirection(newDirection);
      delay(50);
    }

    // Stop turning
    motorAgua.setSpeed(0);

    // Resume forward movement
    currentState = MOVING_FORWARD;
  }
}

void robotLogic() {
  if (nextTimeLogic > millis()) return; // Prevent logic from running too frequently
  nextTimeLogic += 500; // Run logic every 500ms

  mpu.update();
  bmp.readSensor();
  logBuffer.println("------ Millis: " + String(millis()));
  logBuffer.println("Inclination: " + String(mpu.getInclination()));
  logBuffer.println("Magnetic Direction: " + String(mpu.getMagDirection()));
  logBuffer.println("accelX: " + String(mpu.getAccelX()));
  logBuffer.println("accelY: " + String(mpu.getAccelY()));
  logBuffer.println("accelZ: " + String(mpu.getAccelZ()));
  logBuffer.println("Temperature: " + String(bmp.temperature));
  logBuffer.println("Pressure: " + String(bmp.pressure));
  logBuffer.println("IP address: " + WiFi.localIP().toString());

  switch (currentState) {
    case STARTING:
      logBuffer.println("Starting robot...");
      // read mpu and check if robot is moving, and wait until it is upright
      while (mpu.getInclination() < FLOOR_INCLINATION_PRECISION) {
        mpu.update();
        logBuffer.println("Waiting for robot to be upright...");
        delay(50); // Wait until the robot is upright
      }

      motorAgua.setSpeed(AGUA_IDLE_SPEED);
      motorMovimiento.setSpeed(10); // Start moving forward slowly

      currentState = MOVING_FORWARD;
      logBuffer.println("Robot started and ready to move.");

      break;
    case MOVING_FORWARD:
      motorMovimiento.setSpeed(MOVIMIENTO_MOVE_SPEED);
      motorAgua.setSpeed(AGUA_MOVE_SPEED);
      handleWallDetection();
      updatePosition();
      break;

    case MOVING_BACKWARD:
      motorAgua.setSpeed(AGUA_IDLE_SPEED);
      motorMovimiento.setSpeed(-MOVIMIENTO_MOVE_SPEED);
      while(mpu.getInclination() < FLOOR_INCLINATION_PRECISION) {
        delay(50); // Wait until the robot is upright
      }
      motorMovimiento.setSpeed(0);

      // Choose a new direction to turn
      targetDirection = random(0, 4) * 90;
      currentState = TURNING;
      break;

    case TURNING:
      turnToDirection(targetDirection);

      // Check if we've reached the target direction
      if (abs(mpu.getMagDirection() - targetDirection) <= TURN_PRECISION) {
        motorAgua.setSpeed(0);
        currentState = MOVING_FORWARD;
      }
      break;

    case STOPPED:
      motorMovimiento.setSpeed(0);
      motorAgua.setSpeed(0);
      break;
  }
}

void setup()
{
  setup_wifi(); // Connect to WiFi
  setup_ota(); // Setup OTA updates
  setup_web_server(); // Setup web server

  led.init();
  motorMovimiento.init(); // Initialize movement motor
  motorAgua.init(); // Initialize water motor
  mpu.init(); // Initialize MPU9250 sensor

  Serial.begin(9600); // Initialize serial communication for debugging
}

void loop() {
  // Handle OTA updates and robot logic
  ArduinoOTA.handle();
  robotLogic();

  led.handleBlink();

  delay(500); // Small delay for stability
}