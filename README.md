# Dolphin Sprite Pool Robot Hack

![Dolphin Sprite](https://www.piscinasferromar.com/media/catalog/product/cache/e84f338105fc4db69a602e3e8c33beec/s/p/sprite_b.jpg)

## Overview

This project documents the restoration and enhancement of a Dolphin Sprite pool cleaning robot. After the original control board failed, I decided to replace it with custom hardware based on an ESP32-S2 Mini microcontroller, MPU9250 IMU sensor, and IBT-2 H-bridge motor drivers.
**Power is supplied by the robot’s main 30V supply, stepped down to 3.3V using a voltage regulator, feeding the ESP32-S2 Mini's `3V3` pin directly (bypassing its onboard 5V→3.3V LDO) along with the IMU and both IBT-2 boards' logic VCC.**

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

| Pin | Connection                | Function                                          |
|-----|---------------------------|----------------------------------------------------|
|  9  | MOVIMIENTO_RPWM_Output    | Movement motor RPWM (IBT-2)                       |
| 11  | MOVIMIENTO_LPWM_Output    | Movement motor LPWM (IBT-2)                       |
|  5  | AGUA_PWM_Output           | Water pump PWM (single direction, no RPWM/LPWM pair) |
|  7  | SDA_PIN                   | I2C SDA (MPU9250/BMP280)                          |
|  6  | SCL_PIN                   | I2C SCL (MPU9250/BMP280)                          |
| LED_BUILTIN | LED               | Status indicator                                  |

Movement motor's R_EN/L_EN are no longer wired to GPIO at all — see `MOTOR_HAS_ENABLE_PINS` in `src/config.h` and the note below. That freed GPIO5 (old R_EN), now reused for the water pump.

### MPU9250 (I2C Connection)
- VCC: 3.3V (from voltage regulator, same rail as everything else)
- GND: Ground
- SDA: GPIO7
- SCL: GPIO6

### IBT-2 H-Bridge Board (movement motor)
- VCC: 3.3V (logic supply for the onboard SN74AHC244 buffer, same rail as ESP32/IMU)
- VS (motor power terminal): main 30V supply, separate from VCC — not through the regulator
- GND: Ground
- R_EN and L_EN: **tied directly to VCC on the board** — no GPIO, always enabled. Firmware never touches them (`MOTOR_HAS_ENABLE_PINS 0` in `src/config.h`); the reversal dead time in `src/logic/motor_direction_guard.h` is what keeps forward/reverse from overlapping instead.
- RPWM: PWM input for forward direction, GPIO9 (see table above)
- LPWM: PWM input for reverse direction, GPIO11 (see table above)

> **Why VCC=3.3V, not 5V:** each IBT-2 board has an `SN74AHC244` line buffer between the header pins and the BTN7970/BTS7960 chips. Its VIH threshold is ratiometric to its own VCC pin (~3.5V min at VCC=5V per TI datasheet) — a 3.3V-logic GPIO (ESP32) doesn't reliably clear that, causing intermittent/flaky motor start. At VCC=3.3V the threshold drops to ~2.3V, which 3.3V GPIO clears comfortably. The BTN7970/BTS7960 chips themselves are fine with 3.3V logic (VIH max ~2.15V per Infineon datasheet) — the buffer was the bottleneck.

### Water Pump (single direction)
- VCC / driver supply: 3.3V, same rail
- PWM: GPIO5 (see table above) — one pin only, no RPWM/LPWM pair, no enable pin
- GND: Ground

### Power Supply
- **Main supply:** 30V (robot's original battery) — feeds the IBT-2 VS (motor power) terminals directly
- **Voltage regulator:** Steps down 30V to 3.3V, single rail feeding the ESP32-S2 Mini's `3V3` pin directly (skips the onboard LDO/5V pin), the MPU9250, and both IBT-2 boards' VCC (logic)
- Size the regulator for peak current: ESP32 WiFi bursts ~400-500mA, plus IMU + 2× buffer chips (low mA) on top

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
| `/status`         | GET    | JSON: state, tilt angle, yaw, cleaned-area grid, robot x/y, session progress, maintenance stats |
| `/control?action=`| GET    | Drive the state machine: `action=start` (optional `&duration=<minutes>`, 0/omitted = unlimited) / `stop` / `turn` |
| `/config`         | GET    | Runtime settings: `?sessionDuration=<min>` and/or `?statsSaveInterval=<min>` (0 = only save on session end). Returns current values as JSON. |
| `/logs`           | GET    | Plain-text tail of the in-memory log buffer (last ~1500 chars)                |

OTA updates (`ArduinoOTA`) are also enabled with no password set — same caveat applies.

### Maintenance stats

Boot count and total active-cleaning hours are persisted to LittleFS (`src/app/maintenance.cpp`, `/maintenance.json`) across reboots, for wear/maintenance tracking. They're saved on `statsSaveInterval` (default 10 minutes) and always right when a cleaning session ends, so a power loss mid-run only risks losing the last few seconds. Read them from `/status` → `maintenance.bootCount` / `maintenance.totalRuntimeHours`.

### Fault log

`src/app/error_reporter.cpp` keeps an ECU-style fault log, persisted to LittleFS (`/errors.json`): each fault code logs once when it first occurs and stays "active" (no duplicate entries) until the condition clears - e.g. a failed sensor read only logs once per failure streak, not every loop iteration. Current codes: `ImuInitFailed`, `ImuReadFailed`, `BmpInitFailed`, `TurnTimeout`, `WifiConnectFailed`.

- `GET /errors` - JSON list of logged faults (code, name, timestamp, whether still active)
- `GET /errors?action=clear` - wipes the log, like clearing codes on a scan tool

### Turn strategies

Turning has always been finicky on this robot: it turns by thrust against water (not wheels), so each attempt rotates a variable amount, and the MPU9250's gyro is unreliable in practice. Rather than commit to one fix, turning is pluggable (`src/app/turn_controller.h` - same interface shape as `ImuSensor`) with three interchangeable strategies:

| Strategy   | Class               | How it works                                                                 |
|------------|----------------------|-------------------------------------------------------------------------------|
| `legacy`   | `LegacyGyroTurn`     | Original behavior: track yaw via raw gyro integration until in range           |
| `duration` | `FixedDurationTurn`  | No yaw tracking - pulse the motor for a calibrated fixed time (`TURN_DURATION_MS` in `config.h`, rough default, tune on hardware) |
| `kalman`   | `KalmanGyroTurn`     | Track yaw via a Kalman-smoothed gyro rate (`logic/kalman_filter.h`) - reduces noise, does **not** correct long-term drift (no magnetometer/absolute yaw reference exists near the motors) |

Defaults per detected IMU at boot (`ImuSensor::hasReliableGyro()`): `legacy` for LSM6DS3, `duration` for MPU9250. Override anytime via `/config?turnStrategy=legacy|duration|kalman` to experiment on real hardware. A turn that runs out the safety timeout without completing logs `TurnTimeout` regardless of strategy.

If a turn doesn't fully rotate the robot, that's expected and self-corrects: `handleWallDetection()` re-triggers `TURNING` on the next cycle if the wall/tilt condition still holds.

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

Pure logic (no hardware, no WiFi) is extracted into header-only modules under `src/logic/` — speed clamping, turn-yaw math, grid position math, wifi reconnect timing, IMU chip detection, session timer, maintenance stats — and covered by native Unity tests that run on your machine, not the ESP32:

```sh
pio test -e native
```

Everything under `src/hal/` (motor/led/IMU drivers) and `src/app/` + `src/net/` (state machine, sensor reads, web server, wifi/OTA) is still untested by design — it's the hardware-facing glue and can't run without real hardware. New logic should be split so the decision-making part lands in `src/logic/` with a native test, and the hardware-touching part stays a thin wrapper around it (see `src/app/imu_setup.cpp` calling into `src/logic/imu_detect.h` for the pattern).

### Coverage

`env:native_coverage` (in `platformio.ini`) instruments the same tests for coverage. On macOS the native toolchain is clang, so it uses clang's source-based coverage rather than gcov (if your native toolchain is gcc/Linux, plain `--coverage` + `gcov`/`lcov` works the usual way instead):

```sh
LLVM_PROFILE_FILE="coverage.profraw" pio test -e native_coverage -f test_logic
xcrun llvm-profdata merge -sparse coverage.profraw -o coverage.profdata
xcrun llvm-cov report .pio/build/native_coverage/program -instr-profile=coverage.profdata src/logic/position_math.h
```

Swap `-f test_logic` and the last path for whichever suite/header you're checking (each suite builds its own binary at `.pio/build/native_coverage/program`, so one binary only carries coverage for the headers that suite includes - run/inspect one suite at a time rather than expecting a single combined report). Drop `xcrun` if you're not on macOS.

Only `src/logic/` files have meaningful coverage to check — the rest isn't compiled into the native test binary at all (see `build_src_filter` in `platformio.ini`).

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
- [x] Dual IMU support (MPU9250 + LSM6DS3), auto-detected via I2C `WHO_AM_I` at boot — see `src/app/imu_setup.cpp` + `src/logic/imu_detect.h`
- [x] Configurable cleaning timer — `/control?action=start&duration=<minutes>`, enforced in `robot_logic.cpp`
- [x] Maintenance stats (boot count, total cleaning hours) persisted to LittleFS — `src/app/maintenance.cpp`
- [x] Native unit test harness + CI (`.github/workflows/ci.yml`)
- [x] ECU-style fault log (`/errors`), persisted, deduped while a fault stays active — `src/app/error_reporter.cpp`
- [x] Per-IMU turn strategy: gyro-precise on LSM6DS3, fixed-duration fallback on MPU9250 (unreliable gyro on that hardware) — `ImuSensor::hasReliableGyro()`

Backlog:
- [ ] Optimize cleaning patterns based on pool shape
- [ ] Persist the `cleanedArea` coverage grid itself (currently RAM-only, resets on reboot — only the boot count/runtime stats are persisted so far)

Planned — tackled one at a time, incrementally, so we never break a working robot:

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
- Needs the movement motor at full power (`MOVIMIENTO_MOVE_SPEED`, currently a modest 100/255) to have any chance of climbing - not the idle/cleaning speed.

---

## License

This project is open source under the MIT license.