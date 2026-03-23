#  Multi Motor Control Node (ROS2)

A ROS2 node for **multi-bus, multi-motor unified control** over CAN, supporting both **DM4340 (MIT-control)** and **ODrive** motor drivers.
It provides a consistent interface for command input, trajectory interpolation, safety handling, and state feedback publishing.

---

#  Features

* Multi-CAN bus support (`can0`, `can1`, ...)
* Mixed motor types:

  * DM4340 (MIT control)
  * ODrive (position / velocity / torque modes)
* Unified command interface via ROS2 topic
* Cubic interpolation for smooth motion
* ODrive calibration & state machine handling
* Soft position limits (`q_min / q_max`) for safety
* Real-time state publishing
* Safety shutdown and fault handling

---

#  System Overview

```
CAN Bus Layer
   ├── Dispatcher
   ├── DM Controller
   └── ODrive Driver

Motor Layer
   ├── DM Motors
   └── ODrive Axes

Control Layer
   ├── Command Parsing
   ├── Interpolation
   ├── Limit Enforcement
   └── Control Loop

ROS Interface
   ├── /motor_commands (input)
   └── /motor_states   (output)
```

---

#  Parameters

## Core Parameters

| Parameter                 | Type     | Description                        |
| ------------------------- | -------- | ---------------------------------- |
| `can_interfaces`          | string[] | List of CAN interfaces             |
| `update_rate`             | double   | Control loop frequency (Hz)        |
| `interpolation_enabled`   | bool     | Enable position interpolation      |
| `default_interp_duration` | double   | Default interpolation duration (s) |

---

## Motor Configuration

| Parameter          | Description              |
| ------------------ | ------------------------ |
| `motor_ids_can0`   | Motor IDs on CAN0        |
| `motor_ids_can1`   | Motor IDs on CAN1        |
| `motor_types_canX` | `"DM4340"` or `"ODRIVE"` |

Example:

```
motor_ids_can0: "[1, 2, 5, 6]"
motor_types_can0: "['DM4340', 'DM4340', 'DM4340', 'DM4340']"
```

---

## Joint Limits (Soft Safety Limits)

| Parameter              | Description                   |
| ---------------------- | ----------------------------- |
| `joint_limit_keys`     | Motor identifiers (`canX:id`) |
| `joint_q_mins`         | Minimum positions             |
| `joint_q_maxs`         | Maximum positions             |
| `default_limit_margin` | Safety margin                 |

Example:

```
joint_limit_keys: "['can0:1', 'can0:2']"
joint_q_mins: "[-1.5, -0.8]"
joint_q_maxs: "[1.1, 0.95]"
default_limit_margin: 0.02
```

---

#  Command Interface

## Topic: `/motor_commands`

Type: `sensor_msgs/msg/JointState`

### Naming Convention

```
<can_interface>:<motor_id>
```

Example:

```
can0:1
can1:3
```

---

## Control Modes

| Input Field | Mode                                           |
| ----------- | ---------------------------------------------- |
| `position`  | Position control (with optional interpolation) |
| `velocity`  | Velocity control                               |
| `effort`    | Torque control                                 |

### Priority

```
Position > Velocity > Torque
```

---

## Example Command

```
ros2 topic pub /motor_commands sensor_msgs/msg/JointState "
name: ['can0:1']
position: [1.2]
"
```

---

#  Control Loop

Runs at `update_rate` Hz and follows:

1. Poll CAN feedback
2. Update motor states
3. Process interpolation
4. Apply safety limits
5. Send control commands
6. Publish states

---

#  Interpolation

Position commands can be smoothed using cubic interpolation:

* Smooth start/stop (zero velocity at endpoints)
* Avoids step jumps and mechanical shock

Mathematically:

q = q0 + Δq (3s² - 2s³)

---

#  Safety Mechanisms

## 1. Soft Position Limits

* Clamp position commands within `[q_min, q_max]`
* Applied:

  * At command input
  * During interpolation
  * Before sending commands

## 2. Velocity Direction Protection

* Prevents movement further into unsafe region

## 3. Runtime Boundary Check

* If actual position exceeds limits:

  * DM → disabled
  * ODrive → set to IDLE

## 4. ODrive State Machine

* Calibration required before control
* Automatic error clearing
* Closed-loop activation handled internally

---

#  State Output

## Topic: `/motor_states`

Each message represents one motor:

```
can_if: "can0"
motor_id: 1
motor_type: "ODRIVE"
position: 1.57
velocity: 0.25
torque: 0.12
active_errors: 0
axis_state: 8
procedure_result: 0
trajectory_done: true
```

---

#  Motor Behavior

## DM4340

* Uses MIT control
* Direct enable/disable
* Position/velocity/torque control

## ODrive

* Requires calibration
* Supports:

  * Position mode
  * Velocity mode
  * Torque mode
* Controlled via internal state machine

---

#  Shutdown Behavior

On node shutdown:

1. Stop control loop
2. ODrive → switch to IDLE
3. DM → disable
4. Final CAN poll

Ensures safe system stop.

---

#  Recommended Workflow

1. Measure physical joint limits manually
2. Apply safety margin
3. Configure limits in YAML
4. Test:

   * Over-limit position command
   * Boundary velocity command
   * Emergency stop behavior

---

#  Important Notes

* Never use raw mechanical limits directly
* Always leave a safety margin
* Interpolation is strongly recommended
* ODrive must complete calibration before control

---

#  Example Launch Config

```
multi_motor_control_node:
  ros__parameters:
    can_interfaces: ["can0", "can1"]
    update_rate: 100.0

    interpolation_enabled: true
    default_interp_duration: 1.0

    motor_ids_can0: "[1, 2]"
    motor_types_can0: "['DM4340', 'DM4340']"

    motor_ids_can1: "[3, 4]"
    motor_types_can1: "['ODRIVE', 'ODRIVE']"

    joint_limit_keys: "['can0:1', 'can1:3']"
    joint_q_mins: "[-1.5, -2.0]"
    joint_q_maxs: "[1.1, 2.0]"
```

---

#  Design Highlights

* Unified abstraction across heterogeneous motor drivers
* Bus-oriented architecture (scalable)
* Safety-first control pipeline
* ROS2-native integration
* Easily extensible

---

#  Future Improvements

* Acceleration limits
* Jerk-limited trajectory (S-curve)
* Dynamic parameter update
* Diagnostics topic
* ROS2 control integration
