<p align="center">
    <img alt="MIRMI" src="./.media/TUM_mirmi.png" height="80">
</p>

# DoDo Alive! Project WS25

DoDo Alive! is a course-based project at TUM MIRMI.  
The Bipedal Robot Locomotion Task is a team consisting of course students and motivated external contributors.  
Thanks for the support and supervision from:  
- Dr. Hoan Quang Le  

Contributor of the code for this semester
Bo Song

Based on the following repos:
[ros_odrive](https://github.com/odriverobotics/ros_odrive)
[dodo_robot_ROS2(Previous semesters)](https://github.com/Thisanwerss/dodo_robot_ROS2)
[Damiao Driver(Python)](https://wiki.seeedstudio.com/damiao_series/)

---

# Dodo Robot Bipedal Robot Locomotion ROS2 Framework

This is a ROS2-based control framework for a bipedal robotic system with 8 degrees of freedom (DoF). 

This system implements a ROS2 multi-motor unified control node capable of simultaneously managing two types of drive devices—DM4340 and ODrive Tmotor two CAN buses. Upon system startup, the module first loads and verifies configurations such as bus settings, motor IDs, motor types, and interpolation parameters. It then establishes communication contexts based on the CAN buses and dynamically instantiates motor objects according to the configuration.

During runtime, the node executes a closed-loop process of “CAN feedback polling—state refresh—control dispatch—state publication” through a periodic control loop. For DM motors, the node primarily uses the MIT/speed control interface; for ODrive, it additionally incorporates state machine logic such as non-blocking calibration, control mode switching, and closed-loop entry.

At the command input layer, the node subscribes to JointState messages on the `/motor_commands` topic, using `can_if:id` as a unified target index. It interprets the `position`, `velocity`, and `effort` fields as position, velocity, and torque control commands, respectively. For position commands, the system can optionally enable cubic polynomial interpolation to achieve a smooth transition from the current position to the target position within a specified time.

Finally, the node uniformly publishes all motor states to `/motor_states`, providing APIs to upper-level systems.


### Currently supported interfaces

- **Hardware Interface**: CAN-based motor control and sensor interface
- **Sensor Fusion**: Time-synchronization of sensor data
- **Trajectory Recording/Replay**

### Key Message Types
- **sensor_msgs/JointState**: Used for motor commands and state

## Getting Started

### Prerequisites

- **ROS2 Humble** (or later)
- **Ubuntu 22.04** (or later)
- **Python 3.10+**
- CAN interface (for hardware control)

## Build/Run Instructions

```bash
# Clone the repo and enter workspace
cd ~/dodo_main

# Source ROS2 Humble setup
source /opt/ros/humble/setup.bash

# Build the workspace
colcon build --packages-select dodo_canbus --event-handlers console_direct+

# Source your overlay workspace after building
source install/setup.bash

# Run:
ros2 launch dodo_canbus canbus_node.launch.py
# Debug:
#ros2 run dodo_canbus canbus_node --ros-args --log-level rcl:=info --log-level multi_motor_control_node:=debug
```

## Before you send topic messages, following need to be done:
- [] Safety limitations! After a forced power cycle,Odrive *loses* its position data and treats the position upon power-up as the 0 position. Therefore, be careful with q max and q min! Alternatively, a solution may be found in the future. For example, try using Odrive’s absolute position mode (I tried this but was unsuccessful).
- [] DM Motors doesn't have the issues above, but STILL please *update* the actual limitations in the following parameters in `canbus_node.launch.py`
  -- `joint_q_mins`
  -- `joint_q_maxs`

### Send/Read Topic status:
Example:
```bash
# Print motor states
  ros2 topic echo /motor_states
# Disable all motors
  ros2 topic pub /motor_commands sensor_msgs/msg/JointState "{
    name:['can0:1','can0:2','can0:5','can0:6','can1:3','can1:4','can1:7','can1:8'],
    position:[],
    velocity:[],
    effort:[]
}"
# position control
ros2 topic pub /motor_commands sensor_msgs/msg/JointState "{
    name:['can0:6','can0:5','can0:2','can0:1','can1:8','can1:7','can1:4','can1:3'],
    position:[-1, 0, 1, 0, 6, -12, -6, 12],
    velocity:[],
    effort:[]
}"
# torque control
ros2 topic pub /motor_commands sensor_msgs/msg/JointState "{
    name:['can0:6','can0:5','can0:2','can0:1','can1:8','can1:7','can1:4','can1:3'],
    position:[],
    velocity:[],
    effort:[-0.01, 0, 0.01, 0, 0.01, 0, -0.01, 0.01]
}"
```

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Submit a pull request

## License

Distributed under the MIT License. See `LICENSE` for more information.