# Autonomous Robot Navigation System

An Arduino-based autonomous navigation controller for a differential-drive Tektite robot. The program follows a configurable sequence of grid coordinates using encoder odometry, gyro-based heading estimation, closed-loop turning, and trapezoidal speed profiles.

## Highlights

- Converts grid coordinates into robot-frame distances
- Tracks position from left/right wheel encoders
- Integrates yaw-rate measurements for heading estimation
- Uses PD control for point turns and heading correction
- Uses acceleration- and deceleration-limited straight-line motion
- Supports forward and reverse route steps
- Applies static-friction compensation and motor-voltage limits
- Includes movement timeouts, a physical stop-button check, and stop-duration budgeting
- Streams pose and controller telemetry over Serial for tuning

## Tech stack

- Arduino / C++
- Tektite `TektiteRotEv` hardware library
- Differential-drive encoder odometry
- Gyroscope-based heading estimation
- PD feedback control and trapezoidal motion profiling

## Architecture

The single sketch, `RotoCode.ino`, is organized into four main layers:

1. **Hardware interface** — reads battery voltage, encoders, yaw rate, and buttons through `TektiteRotEv`; writes normalized motor duty cycles.
2. **State estimation** — unwraps encoder angles, filters wheel velocity, integrates yaw rate, and updates the robot pose in centimeters.
3. **Motion control** — turns to a target heading, drives a requested distance while holding heading, and applies voltage caps and breakaway compensation.
4. **Route execution** — converts each `MoveStep` grid coordinate into a target point, chooses forward or reverse travel, and runs the route sequentially.

## Hardware assumptions

The checked-in tuning values are specific to the robot used for development:

- 6.03 cm drive-wheel diameter
- Two independently driven wheels with encoders
- Yaw-rate sensor exposed by `TektiteRotEv`
- Go and stop buttons
- 50 cm default grid spacing on both axes

Retune the constants near the top of the sketch before using different motors, wheels, gearing, mass, or field dimensions.

## Setup

1. Install the Arduino IDE or an equivalent Arduino-compatible build environment.
2. Install the hardware support and `TektiteRotEv` library supplied for the Tektite platform.
3. Clone this repository and open `RotoCode.ino`.
4. Confirm the motor direction, encoder signs, wheel diameter, grid spacing, and controller gains.
5. Connect over Serial at the baud rate configured in `setup()`, compile, and upload to the robot.
6. Test with the drive wheels safely raised before running on the floor.

## Configure a route

Routes use grid coordinates and an optional reverse flag:

```cpp
struct MoveStep {
  float x;
  float y;
  bool reverse;
};

#define STEP(x, y, reverse) {x, y, reverse}
```

Edit the route array in `RotoCode.ino` to describe the desired tour. Positive grid `x` points right and positive grid `y` points forward. Each unit is converted using `X_STEP_CM` and `Y_STEP_CM`.

## Tuning and diagnostics

Key constants are grouped near the top of the sketch:

- `MAX_CRUISE_CMPS`, `MAX_ACCEL_CMPS2`, `MAX_DECEL_CMPS2`
- `kP_speed` and `kStaticFwd`
- `kP_heading`, `kD_heading`
- `kP_turn`, `kD_turn`, and turn breakaway limits
- distance, velocity, angle, and timeout thresholds

With `DEBUG_SERIAL` enabled, the controller prints pose, drive, turn, wait-cap, and gyro-recalibration telemetry. Use those measurements to tune one subsystem at a time.

## Safety

This is physical-robot control software. Keep the robot on blocks for initial tests, maintain access to the stop button, begin with conservative voltage limits, and verify every route in an open area.
