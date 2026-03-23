#!/usr/bin/env python3
import json
import math
import time
from pathlib import Path
from typing import List, Dict, Any, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType


class TrajectoryPlayer(Node):
    def __init__(self):
        super().__init__('trajectory_player_node')

        self.declare_parameter('command_topic', '/motor_commands')
        self.declare_parameter('state_topic', '/motor_states')
        self.declare_parameter('trajectory_file', 'trajectory.jsonl')
        self.declare_parameter('align_duration', 2.0)
        self.declare_parameter('align_with_interpolation', True)
        self.declare_parameter('playback_speed', 1.0)
        self.declare_parameter('position_only', True)
        self.declare_parameter('control_node_name', '/multi_motor_control_node')
        self.declare_parameter('disable_interpolation_during_playback', True)

        self.command_topic = self.get_parameter('command_topic').get_parameter_value().string_value
        self.state_topic = self.get_parameter('state_topic').get_parameter_value().string_value
        self.trajectory_file = self.get_parameter('trajectory_file').get_parameter_value().string_value
        self.align_duration = self.get_parameter('align_duration').get_parameter_value().double_value
        self.align_with_interpolation = self.get_parameter('align_with_interpolation').get_parameter_value().bool_value
        self.playback_speed = self.get_parameter('playback_speed').get_parameter_value().double_value
        self.position_only = self.get_parameter('position_only').get_parameter_value().bool_value
        self.control_node_name = self.get_parameter('control_node_name').get_parameter_value().string_value
        self.disable_interpolation_during_playback = (
            self.get_parameter('disable_interpolation_during_playback').get_parameter_value().bool_value
        )

        self.pub = self.create_publisher(JointState, self.command_topic, 10)
        self.sub = self.create_subscription(JointState, self.state_topic, self.state_callback, 50)

        self.latest_state: Optional[JointState] = None

        self.records = self.load_trajectory(self.trajectory_file)
        if not self.records:
            raise RuntimeError('Trajectory file is empty.')

        self.get_logger().info(f'Loaded {len(self.records)} frames from {self.trajectory_file}')
        self.get_logger().info(f'Publishing commands to {self.command_topic}')
        self.get_logger().info(f'Listening state from {self.state_topic}')

        self.wait_for_initial_state()

        restore_interp = False
        if self.disable_interpolation_during_playback:
            # 对齐阶段需要插值，所以先保证底层插值开启
            ok = self.set_control_node_interpolation(True)
            restore_interp = ok

        self.align_to_start_pose()

        if self.disable_interpolation_during_playback:
            # 正式回放前关闭底层插值，避免二次平滑
            self.set_control_node_interpolation(False)

        self.playback()

        self.align_to_start_pose()

        if restore_interp:
            self.set_control_node_interpolation(True)

    def load_trajectory(self, file_path: str) -> List[Dict[str, Any]]:
        path = Path(file_path).expanduser().resolve()
        if not path.exists():
            raise FileNotFoundError(f'Trajectory file not found: {path}')

        records: List[Dict[str, Any]] = []
        with path.open('r', encoding='utf-8') as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                records.append(json.loads(line))
        return records

    def state_callback(self, msg: JointState):
        self.latest_state = msg

    def wait_for_initial_state(self):
        self.get_logger().info('Waiting for /motor_states ...')
        while rclpy.ok() and self.latest_state is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info('Received initial motor state.')

    def joint_state_to_dict(self, msg: JointState) -> Dict[str, Dict[str, float]]:
        result = {}
        n = len(msg.name)
        for i in range(n):
            result[msg.name[i]] = {
                'position': msg.position[i] if i < len(msg.position) else math.nan,
                'velocity': msg.velocity[i] if i < len(msg.velocity) else math.nan,
                'effort': msg.effort[i] if i < len(msg.effort) else math.nan,
            }
        return result

    def publish_position_command(self, names: List[str], positions: List[float]):
        msg = JointState()
        msg.name = names
        msg.position = positions
        msg.velocity = [math.nan] * len(names)
        msg.effort = [math.nan] * len(names)
        self.pub.publish(msg)

    def align_to_start_pose(self):
        first = self.records[0]
        start_names = first['name']
        start_positions = first['position']

        if not self.align_with_interpolation:
            self.get_logger().info('Aligning to trajectory start pose without interpolation...')
            self.publish_position_command(start_names, start_positions)
            for _ in range(10):
                rclpy.spin_once(self, timeout_sec=0.0)
                time.sleep(0.02)
                self.publish_position_command(start_names, start_positions)
            self.get_logger().info('Start pose command sent without interpolation.')
            return

        current_map = self.joint_state_to_dict(self.latest_state)
        q0 = []
        q1 = []

        for i, name in enumerate(start_names):
            if name in current_map and math.isfinite(current_map[name]['position']):
                q0.append(current_map[name]['position'])
            else:
                q0.append(start_positions[i])
            q1.append(start_positions[i])

        self.get_logger().info('Aligning to trajectory start pose with interpolation...')

        t0 = time.time()
        duration = max(0.01, self.align_duration)
        rate_hz = 100.0
        dt = 1.0 / rate_hz

        while rclpy.ok():
            elapsed = time.time() - t0
            if elapsed >= duration:
                break

            s = elapsed / duration
            alpha = 3.0 * s * s - 2.0 * s * s * s

            q_cmd = [
                q0_i + alpha * (q1_i - q0_i)
                for q0_i, q1_i in zip(q0, q1)
            ]

            self.publish_position_command(start_names, q_cmd)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(dt)

        self.publish_position_command(start_names, q1)
        self.get_logger().info('Start pose alignment done.')

    def playback(self):
        self.get_logger().info('Start trajectory playback...')
        t_wall_start = time.time()
        first_t = self.records[0]['t']

        idx = 0
        rate_hz = 200.0
        dt = 1.0 / rate_hz

        while rclpy.ok() and idx < len(self.records):
            elapsed_wall = (time.time() - t_wall_start) * self.playback_speed
            target_t = first_t + elapsed_wall

            while idx < len(self.records) and self.records[idx]['t'] <= target_t:
                rec = self.records[idx]

                msg = JointState()
                msg.name = rec['name']

                if self.position_only:
                    msg.position = rec['position']
                    msg.velocity = [math.nan] * len(rec['name'])
                    msg.effort = [math.nan] * len(rec['name'])
                else:
                    msg.position = rec['position']
                    msg.velocity = rec['velocity']
                    msg.effort = rec['effort']

                self.pub.publish(msg)
                idx += 1

            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(dt)

        self.get_logger().info('Trajectory playback finished.')

    def set_control_node_interpolation(self, enabled: bool) -> bool:
        self.get_logger().info(
            f'Setting {self.control_node_name} interpolation_enabled={enabled}'
        )

        service_name = f'{self.control_node_name}/set_parameters'
        client = self.create_client(SetParameters, service_name)

        if not client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error(f'Parameter service not available: {service_name}')
            return False

        req = SetParameters.Request()

        p = Parameter()
        p.name = 'interpolation_enabled'
        p.value = ParameterValue(
            type=ParameterType.PARAMETER_BOOL,
            bool_value=enabled
        )

        req.parameters = [p]

        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)

        if not future.done():
            self.get_logger().error('Set parameter request timed out.')
            return False

        result = future.result()
        if result is None:
            self.get_logger().error('Set parameter returned no result.')
            return False

        success = all(r.successful for r in result.results)
        if success:
            self.get_logger().info(f'interpolation_enabled set to {enabled}')
        else:
            reasons = [r.reason for r in result.results if not r.successful]
            self.get_logger().error(
                f'Failed to set interpolation_enabled. Reasons: {reasons}'
            )

        return success


def main():
    rclpy.init()
    node = None
    try:
        node = TrajectoryPlayer()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'[trajectory_player_node] ERROR: {e}')
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
