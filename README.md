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
3. Copy `src/secrets.h.example` to `src/secrets.h` and fill in your real WiFi SSID/password (this file is gitignored — it never gets committed)
4. Connect hardware according to the pinout table and ensure the voltage regulator is properly wired between the 30V supply and the 3.3V logic components
5. Build and flash the firmware:
    ```sh
    pio run -t upload
    ```
6. Place the robot in the pool and power on

---

## Web API

Served by the ESP32 on port 80 once connected to WiFi. None of these endpoints require authentication — anyone on the same network can hit them.

| Endpoint          | Method | Description                                                                 |
|-------------------|--------|-------------------------------------------------------------------------------|
| `/`               | GET    | Serves the control UI (`src/index.html`, baked into firmware as `src/index.h`) |
| `/status`         | GET    | JSON: current state, tilt angle, yaw, cleaned-area grid, robot x/y position    |
| `/control?action=`| GET    | Drive the state machine: `action=start` / `stop` / `turn`                     |
| `/logs`           | GET    | Plain-text tail of the in-memory log buffer (last ~1500 chars)                |

OTA updates (`ArduinoOTA`) are also enabled with no password set — same caveat applies.

### Regenerating the UI

`src/index.h` (the HTML baked into firmware as a `PROGMEM` string) is generated from `src/index.html`. After editing the HTML, regenerate it manually:

```sh
python3 gen_html_header.py
```

There's no build hook wiring this in yet, so it's easy to edit `index.html` and forget to regenerate — check `src/index.h` changed before flashing.

### Local UI development

`flask_mock_server.py` serves `src/index.html` against fake `/status`, `/control`, `/logs` responses, so UI work can happen on a laptop without flashing the robot:

```sh
pip install -r requirements.txt
python3 flask_mock_server.py
```

---

## Testing

Pure logic (no hardware, no WiFi) is extracted into header-only modules — `src/speed_utils.h`, `src/turn_math.h`, `src/position_math.h` — and covered by native Unity tests that run on your machine, not the ESP32:

```sh
pio test -e native
```

This is the only test coverage that exists today. The state machine, sensor reads, and web server in `main.cpp` are still untested — they're tightly coupled to Arduino/WiFi globals. New pure logic extracted from `main.cpp` should get a native test alongside it rather than being tested by hand on hardware.

---

## Security notes

- **WiFi credentials**: kept in `src/secrets.h`, gitignored. If you ever find real credentials in a commit, rotate the WiFi password immediately and scrub git history (`git filter-repo` or BFG) before anyone else clones it.
- **No auth on the web API or OTA**: anyone on the same WiFi network can start/stop/turn the robot or push new firmware. Fine for a home network you trust, not fine if the SSID is shared/public.
- **Single hardcoded SSID**: there's no fallback AP mode — if the configured network is unreachable, the robot has no way to be reconfigured over WiFi.

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

Done:
- [x] Add remote control via WiFi
- [x] Add status LED indicators

Backlog:
- [ ] Optimize cleaning patterns based on pool shape

Planned — tackled one at a time, incrementally, so we never break a working robot:

### Stats & history
- Persist (EEPROM or flash FS) run stats across reboots: number of boots, total cleaning time, last session duration.
- Persist the `cleanedArea` coverage grid (or a compressed/rasterized version) so the "where has it been" map survives a restart, not just RAM.

### Dual IMU support (MPU9250 + new LSM6DS3)
- Auto-detect which IMU is physically wired at boot: read the I2C `WHO_AM_I` register (or just probe both known I2C addresses) and pick MPU9250 vs LSM6DS3 accordingly — no manual `#define`/rebuild needed to swap hardware.
- Wrap sensor access behind a small interface (`readAccel()`, `readGyro()`) so `main.cpp` logic doesn't care which chip answered.

### Configurable cleaning timer
- Let the UI set how long a cleaning run should last (a max-duration timer), instead of running forever until manually stopped.

### Home Assistant integration
- Push robot state (paused / cleaning / finished) to Home Assistant so it shows up like any other HA entity — likely via HA's MQTT discovery or a simple REST sensor call from the ESP32.

### Local UI development against a mock backend
- Build out `flask_mock_server.py` (already a start) into a proper local dev loop: run the real `index.html` UI against a mock server that fakes `/status`, `/control`, `/logs` — so UI changes can be tested on a laptop without flashing the robot each time.

### Manual control
- Dedicated manual-drive mode in the UI (vs. the current start/stop/turn buttons) — direct motor control while overriding the autonomous state machine.

### Fully UI-configurable settings
- Expose the constants that currently only live as `#define`s (autostart delay, turn angle, movement timeout, speeds, etc.) as runtime settings adjustable from the web UI, not just compile-time constants. Autostart delay is the first candidate.

### Wall-climbing detection (experimental)
- Investigate whether the chassis/motors can climb the pool wall like some commercial pool robots do. Once submerged there's no WiFi to lean on, so this needs an on-device check (IMU-based) with no connectivity assumed — pure exploration, not guaranteed to work with this hardware.

---

## License

This project is open source under the MIT license.