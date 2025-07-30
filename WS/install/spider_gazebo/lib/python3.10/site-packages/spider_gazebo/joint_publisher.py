import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

class JointPublisher(Node):
    def __init__(self):
        super().__init__('joint_publisher')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.1, self.publish_joint_states)
        self.angle = 0.0

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint1']
        msg.position = [self.angle]
        self.publisher_.publish(msg)
        self.angle += 0.1
        if self.angle > 3.14:
            self.angle = -3.14

def main(args=None):
    rclpy.init(args=args)
    node = JointPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()