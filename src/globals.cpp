#include "globals.h"

AsyncWebServer server(80);

Motor motorMovimiento(MOVIMIENTO_RPWM_Output, MOVIMIENTO_LPWM_Output, MOVIMIENTO_R_ENABLE, MOVIMIENTO_L_ENABLE);
Motor motorAgua(AGUA_RPWM_Output, AGUA_LPWM_Output, AGUA_R_ENABLE, AGUA_L_ENABLE);
Led led(LED_BUILTIN);
Adafruit_BMP280 bmp;
ImuSensor* imu = nullptr;
LogBuffer logBuffer;

RobotState currentState = STARTING;
RobotState previousState = STARTING;

bool cleanedArea[GRID_SIZE][GRID_SIZE] = {false};
int currentX = GRID_SIZE / 2;
int currentY = GRID_SIZE / 2;

long nextTimeLogic = 1000; // Startup delay of 1 second before robot logic runs
long nextUpdate = 0;
long timeToAutostart = DELAY_AUTOSTART;
long nextPositionUpdate = 0;
long maxTurningMillis = 0;
long timeout = 0;
long timeToConnectWifi = 0;

float aX, aY, aZ, aSqrt, gX, gY, gZ, temp, pressure;
float yaw = 0;
unsigned long lastYawUpdate = 0;

unsigned long sessionDurationMs = 0; // 0 = unlimited by default
unsigned long sessionStartMillis = 0;

unsigned long statsSaveIntervalMs = 600000; // default: every 10 minutes
MaintenanceStats maintenanceStats = {0, 0};

ErrorLog errorLog = {};

TurnStrategy turnStrategy = TurnStrategy::Legacy; // overwritten in setup() per detected IMU

AccelCalibration accelCalibration = defaultAccelCalibration();

MqttConfig mqttConfig;
bool sessionCompletedByTimer = false;
