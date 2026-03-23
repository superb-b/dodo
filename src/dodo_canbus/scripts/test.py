import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class CmdPub(Node):
    def __init__(self):
        super().__init__('cmd_pub')
        self.pub = self.create_publisher(JointState, '/motor_commands', 10)
        self.timer = self.create_timer(1.0, self.send_once)
        self.sent = False

    def send_once(self):
        if self.sent:
            return

        msg = JointState()
        msg.name = ['can1:3']
        msg.position = [0.5]
        msg.velocity = [math.nan]
        msg.effort = [math.nan]

        self.pub.publish(msg)
        self.get_logger().info('sent cubic interpolated position command')
        self.sent = True

rclpy.init()
node = CmdPub()
rclpy.spin(node)