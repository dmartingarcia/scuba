#include "globals.h"

AsyncWebServer server(80);

Motor motorMovimiento(MOVIMIENTO_RPWM_Output, MOVIMIENTO_LPWM_Output, 0, 0); // no enable pins - see MOTOR_HAS_ENABLE_PINS
Motor motorAgua(AGUA_PWM_Output);
Led led(LED_BUILTIN);
Adafruit_BMP280 bmp;
ImuSensor* imu = nullptr;
LogBuffer logBuffer;

RobotState currentState = STARTING;
RobotState previousState = STARTING;

bool cleanedArea[GRID_SIZE][GRID_SIZE] = {false};
int currentX = GRID_SIZE / 2;
int currentY = GRID_SIZE / 2;

TuningParams tuning; // uses its own compiled-in defaults until tuningInit() loads LittleFS

long nextTimeLogic = 1000; // Startup delay of 1 second before robot logic runs
long nextTimeLogUpdate = 1000;
long nextUpdate = 0;
long timeToAutostart = tuning.delayAutostartMs;
long nextPositionUpdate = 0;
long maxTurningMillis = 0;
long timeout = 0;
long timeToConnectWifi = 0;

bool manualControlActive = false;
long manualActionDeadlineMillis = 0;
RobotState manualRevertState = STOPPED;

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
