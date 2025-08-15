# 3D printed 3-DOF Robot Arm

![Demo of the 3DOF robot arm](docs/arm_demo_slow.gif)

Single file Arduino project to control a **3-DOF anthropomorphic arm** plus a **servo gripper**.  
It receives **Cartesian targets** over Serial, solves **analytical inverse kinematics** onboard and commands three stepper motors via `AccelStepper`.

## Overview

- **Inverse kinematics** (IK) for a 3-link arm (shoulder-up, elbow-down branch).
- **3 stepper motors** controlled with `AccelStepper`.
- **Servo gripper** that can be only opened or closed.
- **Simple Serial interface** for homing and Cartesian commands.

---

## Hardware

- **Arduino Mega**.
- **3× stepper motors** + A4988 drivers.
- **1× servo** for the gripper.
- **Power supply** sized for motors and servo.
- [CAD design of the 3D printed planetary gearboxes, base and link connectors available through OnShape](https://cad.onshape.com/documents/7ce69c395a1409a2cfc972d9/w/4e6c29745636ca8a07db6948/e/61d0a1b9c1a23eb44e8f73c4).

---

## Pinout

| Function          | Pins              |
|-------------------|-------------------|
| Step (motors 0–2) | `2`, `3`, `4`     |
| Dir  (motors 0–2) | `11`, `12`, `13`  |
| Microstep select* | `5`, `6`, `7`     |
| Gripper servo     | `9`               |

\* The sketch drives pins `5/6/7` HIGH to select **1/8 microstepping**.

---

## Software Setup

- **Arduino IDE** (or PlatformIO).
- Libraries:
  - [`AccelStepper`](https://www.airspayce.com/mikem/arduino/AccelStepper/) (install via Library Manager).
  - `Servo` (built-in).
  - `math.h` (built-in).
- **Serial**: 9600 baud.

---

## Build & Upload

1. Open the `.ino` file in Arduino IDE.
2. Select your board/port.
3. Upload.
4. Open **Serial Monitor** at **9600 baud**.

---

## Serial Protocol

Send one command per line:

| Command      | Meaning                                        |
|--------------|------------------------------------------------|
| `0`          | **Home** all joints to `0°` (no endstops).     |
| `grip open`  | Open the gripper (servo to `GRIPPER_OPEN`).    |
| `grip close` | Close the gripper (servo to `GRIPPER_CLOSED`). |
| `x y z`      | Move end-effector to Cartesian `(m, m, m)`.    |

Responses:
- `Unreachable` if IK target is outside the workspace.
- Debug lines like: `id q motor_angle steps` when targets are set.

**Units:** `x y z` are **meters**.

**Line endings:** Make sure your terminal sends a **newline `\n`**.

---

## Examples

Home the arm:
```
0
```

Open / close gripper:
```
grip open
grip close
```

Move to 15 cm, 10 cm, 5 cm:
```
0.15 0.10 0.05
```

---

## Motion Model

- Links:
  - `a2 = 0.18 m`
  - `a3 = 0.25 m`
- IK branch: **shoulder-up, elbow-down**.
- Joint angles (degrees) → motor steps use:
  - Motor **step angle**: `1.8°`
  - **Microstepping**: `1/8`
  - **Gear reduction**: `18.666666:1`

The sketch converts `q0, q1, q2` (deg) to steps, then calls `AccelStepper::moveTo()` and drives motion with `run()` in the loop.

---

## Configuration

Adjust these constants at the top of the file to match your hardware:

```cpp
// Actuation properties
#define ANGLE_PER_STEP 1.8
#define MICROSTEP_FRACTION 8
#define REDUCTION_RATIO 18.666666

// Gripper positions (servo angles)
#define GRIPPER_OPEN 50
#define GRIPPER_CLOSED 10
```

And link lengths in the IK function:

```cpp
double a2 = 0.18;
double a3 = 0.25;
```

Stepper dynamics (per-axis limits) are set in `setup()`:
```cpp
steppers[i].setMaxSpeed(i != 1 ? 900 : 700);
steppers[i].setAcceleration(i != 1 ? 5000 : 3000);
```
Tune for your mechanics and drivers.

---

## Notes & Safety

- **No endstops**: The `0` homing just sets targets to `0°`; ensure this is safe for your mechanism.
- **Power**: Use a suitable PSU for steppers and the servo.

---
