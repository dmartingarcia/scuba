#ifndef CONFIG_H
#define CONFIG_H

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

#define GRID_SIZE 30 // Cleaning-progress tracking grid, 30x30 cells

#define LSM6DS3_WHO_AM_I_ADDR 0x6A
#define LSM6DS3_WHO_AM_I_REG 0x0F

// Movement parameters
const float WALL_ANGLE_THRESHOLD = 45.0; // Degrees for wall detection
const float WALL_ANGLE_RECOVER_THRESHOLD = 10.0; // Degrees to consider the robot upright
const float FLOOR_INCLINATION_PRECISION = 10; // Minimum inclination to consider the robot upright
const float MOVIMIENTO_MOVE_SPEED = 100; // Speed for movement
const float MOVIMIENTO_IDLE_SPEED = 50; // Speed for turning
const float AGUA_TURN_SPEED = 255;
const float AGUA_MOVE_SPEED = 240;
const float AGUA_IDLE_SPEED = 180; // Speed when not moving

#endif // CONFIG_H
