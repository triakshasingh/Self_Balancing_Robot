# Self-Balancing Robot

A two-wheeled inverted-pendulum robot built around a custom ESP32-S3 PCB. The
board carries the MCU, an IMU, a dual H-bridge motor driver and the power rail;
firmware fuses accelerometer and gyroscope readings into a tilt angle and runs a
PID loop that drives the wheels to keep the chassis upright.

![Completed robot](images/completed_robot.png)

## Contents

| Path | What's in it |
| --- | --- |
| [PCB/](PCB/) | KiCad 10 project — schematic, board file, project settings |
| [self_balancing_robot_code/self_balancing_robot_code/](self_balancing_robot_code/self_balancing_robot_code/) | Main balancing firmware (Arduino sketch) |
| [self_balancing_robot_code/mpu6050_calibration_only/](self_balancing_robot_code/mpu6050_calibration_only/) | Standalone sketch for finding the upright angle offset |
| [images/](images/) | Build photos and a schematic render |

## Hardware

| Part | Role |
| --- | --- |
| ESP32-S3-WROOM-1 | Main controller, USB programming |
| MPU-6500 (MPU-6050 footprint) | 6-axis IMU over I²C at `0x68` |
| DRV8833PW | Dual H-bridge, drives both gearmotors |
| 2× geared DC motors with quadrature encoders | Drive and wheel odometry |
| Passives | 4.7 kΩ I²C pull-ups, 10 kΩ pull-ups, 0.1 µF / 1 µF / 0.01 µF decoupling |
| 3× push buttons | Boot, reset, user |

The MPU-6500 is pin- and register-compatible with the MPU-6050 footprint used in
the schematic, so the same land pattern works for either part.

### Pin map

Both values below are the ESP32-S3 GPIO numbers used in the firmware.

| Signal | GPIO |
| --- | --- |
| I²C SDA | 8 |
| I²C SCL | 9 |
| DRV8833 `AIN1` / `AIN2` | 4 / 5 |
| DRV8833 `BIN1` / `BIN2` | 6 / 7 |
| DRV8833 `STBY` | 15 |
| Left encoder `C1` / `C2` | 16 / 17 |
| Right encoder `C1` / `C2` | 18 / 21 |

## PCB

![Schematic](images/schematic.png)

Open [PCB/PCB.kicad_pro](PCB/PCB.kicad_pro) in KiCad 10 or newer. The schematic
is complete; **the board layout (`PCB.kicad_pcb`) is still empty** — routing is
the next step on the hardware side.

![Completed design](images/completed_design.png)

## Firmware

Built with the Arduino core for ESP32. Only the bundled `Wire` library is
needed — the IMU is talked to over raw register reads rather than a vendor
driver.

**Arduino IDE setup**

1. Install the ESP32 board package (Boards Manager → *esp32* by Espressif).
2. Select board **ESP32S3 Dev Module**.
3. Set the serial monitor to **115200 baud**.

### How it works

- The IMU is configured for ±4 g and ±500 °/s with the on-chip DLPF enabled.
- `atan2(ax, az)` gives an absolute but noisy tilt angle from the accelerometer;
  the gyro gives a clean but drifting rate.
- A complementary filter blends them at 98 % gyro / 2 % accelerometer each cycle.
- Tilt error feeds a PID controller whose output is written to both motors as a
  signed PWM value.
- If tilt exceeds `FALL_LIMIT` (45°), the motors are cut and the integrator is
  reset so the robot doesn't lurch when picked up.

### Calibration

The robot's "upright" reading is mechanical, not zero, so it has to be measured
once per build.

1. Flash [mpu6050_calibration_only.ino](self_balancing_robot_code/mpu6050_calibration_only/mpu6050_calibration_only.ino),
   hold the robot exactly upright and still, and read the printed `angleOffset`.
   (The main sketch can do the same thing — set `CALIBRATE_ONLY` to `1`.)
2. Put that number into `HARDCODED_OFFSET` in
   [self_balancing_robot_code.ino](self_balancing_robot_code/self_balancing_robot_code/self_balancing_robot_code.ino#L31).
3. Set `CALIBRATE_ONLY` back to `0` and re-flash.

Gyro bias is measured automatically at every boot — leave the robot still for
the couple of seconds after reset while it averages 1000 samples.

### Tuning

Gains live at the top of the main sketch:

```cpp
float Kp = 45;
float Ki = 0.0;
float Kd = 0.0;
```

Start with `Kp` alone and raise it until the robot oscillates around upright,
then add `Kd` to damp the oscillation, and only then a small `Ki` to remove
steady-state lean. The loop prints tilt, gyro rate, output and both encoder
counts every 100 ms for tuning.

## Status

Working: schematic, firmware, balancing loop, encoder counting.
Not done yet: PCB layout and routing; encoder counts are read but not used by
the controller (no position or velocity hold).

![During the build](images/during_build.png)

## License

[MIT](LICENSE)
