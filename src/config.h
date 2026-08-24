#ifndef CONFIG_H
#define CONFIG_H

#ifndef FIRMWARE_COMMIT
#define FIRMWARE_COMMIT "unknown" // Set by get_git_version.py at build time
#endif

// Pin definitions for the motors and sensors

// Movement (drive) motor: IBT-2 H-bridge, RPWM/LPWM only. R_EN/L_EN are
// physically tied to VCC on the board now, not wired to a GPIO - see
// MOTOR_HAS_ENABLE_PINS below. This freed the old R_EN pin (GPIO5), now
// reused below for the water pump.
#define MOVIMIENTO_RPWM_Output 9
#define MOVIMIENTO_LPWM_Output 11

// Water pump: single PWM pin, one direction only - no RPWM/LPWM pair, no
// enable pin (see the single-pin Motor constructor). GPIO5 was movimiento's
// R_EN before the enable wiring was removed.
#define AGUA_PWM_Output 5

// Set to 0 once the IBT-2 R_EN/L_EN wiring is physically removed (tied
// permanently to VCC on the board instead of a GPIO). When 1, Motor drives
// R_EN/L_EN from GPIO on init/setSpeed; when 0, it never touches them, and
// the reversal dead time in motor_direction_guard.h becomes the only thing
// standing between a direction flip and shoot-through.
#define MOTOR_HAS_ENABLE_PINS 0

#define SDA_PIN 7
#define SCL_PIN 3

// Task watchdog: if loop() (or a helper it calls) stops feeding this for
// this many seconds - most likely an I2C lockup talking to the IMU/BMP280,
// which real hardware can hit from motor EMI or moisture - the chip reboots
// itself instead of sitting frozen underwater until someone pulls it out.
// Must stay comfortably above the worst legitimate blocking stretch in
// loop() (a turn maneuver, bounded by tuning.maxTimeTurningMs) - bump this
// too if you tune that much higher.
#define WATCHDOG_TIMEOUT_SECONDS 12

#define DEFAULT_MAX_TIME_TURNING 10000
#define DELAY_UPDATING_SENSORS 100
#define DELAY_UPDATING_YAW 100 // Throttle gyro polling in the main loop, ms
#define DELAY_AUTOSTART 30000
#define DEFAULT_DELAY_AUTOSTART 30000
#define DELAY_UPDATING_POSITION 2000 // Delay for updating position in milliseconds
#define DEFAULT_TURN_ANGLE 15 // Degrees to turn when changing direction
#define DEFAULT_MOVING_TIMEOUT 120000 // Timeout for movement in milliseconds

// Fallback turn duration when the IMU's gyro isn't trustworthy (see
// ImuSensor::hasReliableGyro()) - rough default, tune on real hardware.
#define DEFAULT_TURN_DURATION_MS 3000

// Kalman gyro-rate filter tuning (see logic/kalman_filter.h). Higher process
// noise trusts new readings more; higher measurement noise trusts the
// running estimate more (smooths a noisier gyro harder).
#define KALMAN_PROCESS_NOISE 0.01f
#define KALMAN_MEASUREMENT_NOISE 4.0f

#define GRID_SIZE 30 // Cleaning-progress tracking grid, 30x30 cells

#define LSM6DS3_WHO_AM_I_ADDR 0x6A
#define LSM6DS3_WHO_AM_I_REG 0x0F

// Persisted across reboots on LittleFS (wear-leveled flash filesystem).
#define MAINTENANCE_STATS_PATH "/maintenance.json"
#define ERROR_LOG_PATH "/errors.json"
#define ACCEL_CALIBRATION_PATH "/accel_calibration.json"
#define ACCEL_CALIBRATION_SAMPLES 10
#define MQTT_CONFIG_PATH "/mqtt_config.json"
#define MQTT_DEVICE_NAME "Pool Robot"
#define MQTT_DEFAULT_TOPIC_PREFIX "scuba"
#define MQTT_RECONNECT_INTERVAL_MS 15000
#define MQTT_PUBLISH_INTERVAL_MS 10000
#define TUNING_CONFIG_PATH "/tuning.json"

// Movement parameters - these are only the *defaults*. The live values are
// in globals.h/.cpp as `tuning.*` (see logic/tuning_params.h), editable at
// runtime via /tuning and persisted to LittleFS.
const float DEFAULT_WALL_ANGLE_THRESHOLD = 45.0; // Degrees for wall detection
const float DEFAULT_WALL_ANGLE_RECOVER_THRESHOLD = 10.0; // Degrees to consider the robot upright
const float DEFAULT_FLOOR_INCLINATION_PRECISION = 10; // Minimum inclination to consider the robot upright
const float DEFAULT_MOVIMIENTO_MOVE_SPEED = 50; // Speed for movement
const float DEFAULT_MOVIMIENTO_MOVE_BACKWARDS_SPEED = 70; // Speed for moving backwards
const float DEFAULT_MOVIMIENTO_IDLE_SPEED = 25; // Speed for turning
const float DEFAULT_AGUA_TURN_SPEED = 256;
const float DEFAULT_AGUA_MOVE_SPEED = 245;
const float DEFAULT_AGUA_IDLE_SPEED = 180; // Speed when not moving
const float DEFAULT_ATTITUDE_SMOOTHING_ALPHA = 0.2; // EMA blend for UI pitch/roll display
const long DEFAULT_MANUAL_ACTION_DURATION_MS = 2000; // Manual forward/backward pulse length before auto-reverting

// Maintenance tests (/maintenance?action=rampAgua|rampMovimientoForward|
// rampMovimientoBackward): ramps a motor linearly from 0 to 255 (or 0 to
// -255) over this long, only while parked in MAINTENANCE.
const unsigned long MAINTENANCE_RAMP_DURATION_MS = 10000;

// Calibrated accel X reads ~-1 resting normally, ~+1 flipped onto its back
// (see the readings logged in sensors.cpp) - a threshold well above what
// normal operation (wall climbs, turns) ever reaches on that axis.
const float DEFAULT_UPSIDE_DOWN_THRESHOLD = 0.6;

const long TIME_TO_CONNECT_WIFI = 60000; // 60 seconds to connect to WiFi

// Retries * 500ms delay = total budget for the FIRST connect attempt in
// setupWifi(). This matters a lot more right after an OTA flash than it
// looks: if it runs out before the AP associates, the code treats it as a
// bad new image and triggers an immediate OTA rollback to whatever was in
// the other partition (see rollbackIfPendingVerify() in wifi_manager.cpp) -
// 10 (5s) was too tight for a fresh post-reset association.
const int RETRIES_WIFI_CONNECT = 40; // 20 seconds

// Recovery access point, started whenever the home WiFi (WIFI_SSID in
// secrets.h) can't be reached - keeps the robot reachable for OTA/control
// without ever needing physical/USB access to the sealed enclosure.
#define AP_FALLBACK_SSID "ScubaRobot-Recovery"
#define AP_FALLBACK_PASSWORD "scuba1234"

// Lives in PSRAM (2MB available, otherwise unused by this app) so it can be
// this large without competing with WiFi/AsyncWebServer for internal SRAM -
// the whole point is to survive as much of a submerged, unreachable run as
// possible for later review over /logs. Falls back to a much smaller
// internal-heap buffer if PSRAM isn't available (see log_buffer.h).
const size_t LOG_BUFFER_SIZE = 512000; // Maximum log buffer size (PSRAM)
const size_t LOG_BUFFER_TRIM_SIZE = 500000; // Size to trim the log down to
const size_t LOG_BUFFER_FALLBACK_SIZE = 30000; // Used if PSRAM allocation fails
const size_t LOG_BUFFER_FALLBACK_TRIM_SIZE = 29800;

#endif // CONFIG_H
