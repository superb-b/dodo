#!/usr/bin/env python3
import json
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class TrajectoryRecorder(Node):
    def __init__(self):
        super().__init__('trajectory_recorder_node')

        self.declare_parameter('state_topic', '/motor_states')
        self.declare_parameter('output_file', 'trajectory.jsonl')

        state_topic = self.get_parameter('state_topic').get_parameter_value().string_value
        output_file = self.get_parameter('output_file').get_parameter_value().string_value

        self.output_path = Path(output_file).expanduser().resolve()
        self.output_path.parent.mkdir(parents=True, exist_ok=True)

        self.file = self.output_path.open('w', encoding='utf-8')
        self.start_time = None
        self.frame_count = 0

        self.sub = self.create_subscription(
            JointState,
            state_topic,
            self.state_callback,
            50
        )

        self.get_logger().info(f'Recording trajectory from {state_topic}')
        self.get_logger().info(f'Output file: {self.output_path}')

    def state_callback(self, msg: JointState):
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        if self.start_time is None:
            self.start_time = now_sec

        t = now_sec - self.start_time

        record = {
            't': t,
            'name': list(msg.name),
            'position': list(msg.position),
            'velocity': list(msg.velocity),
            'effort': list(msg.effort),
        }

        self.file.write(json.dumps(record, ensure_ascii=False) + '\n')
        self.file.flush()

        self.frame_count += 1
        if self.frame_count % 100 == 0:
            self.get_logger().info(f'Recorded {self.frame_count} frames')

    def destroy_node(self):
        try:
            self.file.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = TrajectoryRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(f'Stopped recording. Total frames: {node.frame_count}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()