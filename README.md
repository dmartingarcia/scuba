# Dolphin Sprite Pool Robot Hack

![Dolphin Sprite](https://www.piscinasferromar.com/media/catalog/product/cache/e84f338105fc4db69a602e3e8c33beec/s/p/sprite_b.jpg)

## Overview

This project documents the restoration and enhancement of a Dolphin Sprite pool cleaning robot. After the original control board failed, I decided to replace it with custom hardware based on an ESP32-S2 Mini microcontroller, MPU9250 IMU sensor, and IBT-2 H-bridge motor drivers.
**Power is supplied by the robot’s main 30V supply, stepped down to 3.3V using a voltage regulator.**

---

## Hardware Components

- **ESP32-S2 Mini** (main controller)
- **MPU9250** 9-axis IMU (for orientation and navigation)
- **2× IBT-2 H-bridge boards** (motor control)
- **Voltage regulator** (30V to 3.3V for logic supply)
- **Original Dolphin Sprite motors and chassis**

---

## Pinout Configuration

### ESP32-S2 Mini Connections

| Pin | Connection                | Function                        |
|-----|---------------------------|---------------------------------|
|  3  | MOVIMIENTO_RPWM_Output    | Movement motor RPWM (IBT-2 #1)  |
|  2  | MOVIMIENTO_LPWM_Output    | Movement motor LPWM (IBT-2 #1)  |
|  5  | MOVIMIENTO_R_ENABLE       | Movement motor R_EN (IBT-2 #1)  |
|  4  | MOVIMIENTO_L_ENABLE       | Movement motor L_EN (IBT-2 #1)  |
| 13  | AGUA_RPWM_Output          | Water motor RPWM (IBT-2 #2)     |
| 12  | AGUA_LPWM_Output          | Water motor LPWM (IBT-2 #2)     |
| 10  | AGUA_R_ENABLE             | Water motor R_EN (IBT-2 #2)     |
| 11  | AGUA_L_ENABLE             | Water motor L_EN (IBT-2 #2)     |
|  7  | SDA_PIN                   | I2C SDA (MPU9250/BMP280)        |
|  6  | SCL_PIN                   | I2C SCL (MPU9250/BMP280)        |
| LED_BUILTIN | LED               | Status indicator                |

### MPU9250 (I2C Connection)
- VCC: 3.3V (from voltage regulator)
- GND: Ground
- SDA: GPIO7
- SCL: GPIO6

### IBT-2 H-Bridge Boards
- VCC: 3.3V (logic, from voltage regulator)
- GND: Ground
- R_EN and L_EN: Connected to 5V (always enabled, or via ESP32 pins if you want to control enable)
- RPWM: PWM input for forward direction (see table above)
- LPWM: PWM input for reverse direction (see table above)

### Power Supply
- **Main supply:** 30V (robot’s original battery)
- **Voltage regulator:** Steps down 30V to 3.3V for ESP32, IMU, and logic

---

## Software Architecture

The codebase is organized into several key components:

1. **Main Control Loop** - Handles the robot's operation modes and main program flow
2. **Motor Control** - PWM generation and H-bridge interfacing
3. **IMU Interface** - Communication with the MPU9250 and motion processing
4. **Navigation System** - Processes sensor data to control robot movement
5. **Power Management** - Monitors battery levels and handles power-saving features

---

## Setup and Operation

1. Install [PlatformIO](https://platformio.org/)
2. Clone this repository
3. Connect hardware according to the pinout table and ensure the voltage regulator is properly wired between the 30V supply and the 3.3V logic components
4. Build and flash the firmware:
    ```
    pio run -t upload
    ```
5. Place the robot in the pool and power on

---

## Implementation Details

The control system uses the MPU9250's accelerometer and gyroscope to determine the robot's orientation in the pool. This information drives the robot's navigation algorithm, allowing it to:

- Detect walls and obstacles
- Track cleaning progress
- Maintain efficient coverage patterns
- Return to the surface when cleaning is complete or battery is low

The dual H-bridge configuration provides precise control over the two drive motors, enabling forward/reverse operations and turning maneuvers.

---

## Future Improvements

- [x] Add remote control via WiFi
- [ ] Implement autonomous pool mapping
- [x] Add status LED indicators
- [ ] Optimize cleaning patterns based on pool shape

---

## License

This project is open source under the MIT license.