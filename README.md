<p align="center">
    <img alt="MIRMI" src="./media/TUM_mirmi.png" height="80">
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

- **Hardware Interface**: CAN-based motor control and IMU sensor interface
- **Sensor Fusion**: Time-synchronization of sensor data
- **Trajectory Recording/Replay**

## System Architecture and Communication Flow

The system follows a layered approach:
- **Hardware Interface() Layer**: Interacts directly with sensors and actuators
- **Data Processing Layer**: Handles sensor fusion and synchronization
- **Control Layer**: Generates motion commands through RL or manual input
- **System Management Layer**: Handles safety, monitoring, and state transitions

### Node Interactions

1. The **IMU Node** and **CAN Bus Node** interface with physical hardware and publish sensor data.
2. The **Sensor Fusion Node** combines this data into time-synchronized `AlignedSensorData` messages.
3. The **RL Node** uses this synchronized data to generate joint commands based on learned policies.
4. The **USB Command Node** provides manual control through a joystick interface.
5. The **Processing Node** combines, validates, and constraints commands before sending to motors.
6. The **Safety Node** continuously monitors sensors and can trigger emergency stops.
7. The **Monitor Node** provides diagnostics and system health information.
8. The **State Manager Node** coordinates the overall robot state and mode transitions.

### Key Message Types

- **sensor_msgs/JointState**: Used for motor commands and state
- **sensor_msgs/Imu**: Raw IMU data
- **dodo_msgs/AlignedSensorData**: Time-synchronized sensor information
- **std_msgs/Bool**: Used for emergency stop signals
- **std_msgs/String**: Used for robot state information
- **std_msgs/Int32**: Used for directional commands
- **diagnostic_msgs/DiagnosticArray**: System diagnostics and health information

**PlantUML source code for the architecture diagram is available in `architecture.puml`.**

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


## Common Issues & Fixes


## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Submit a pull request

## License

Distributed under the MIT License. See `LICENSE` for more information.