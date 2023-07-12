import rclpy
from rclpy.node import Node
import numpy as np
import math

from geometry_msgs.msg import Twist, PoseStamped, PointStamped
from sensor_msgs.msg import LaserScan

class VelocityController(Node):

    def __init__(self):
        super().__init__('velocity_controller')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.forward_distance = 0
        self.goal = None
        self.position = None
        self.previous_position = None
        self.create_subscription(LaserScan, 'scan', self.laser_cb, rclpy.qos.qos_profile_sensor_data)
        self.create_subscription(PoseStamped, 'nav/goal', self.goal_cb, 10)
        self.create_subscription(PointStamped, 'position', self.position_cb, 10)
        self.create_timer(0.1, self.timer_cb)
        self.get_logger().info('controller node started')
        
    def timer_cb(self):
        if self.forward_distance < 0.5:
            msg = Twist()
            msg.linear.x = 0.0
            msg.angular.z = 1.0
            self.publisher.publish(msg)
            return
        msg = Twist()
        msg.linear.x = 0.1
        if self.goal is not None and self.position is not None and self.previous_position is not None:
            a = np.linalg.norm(self.goal - self.position)
            b = np.linalg.norm(self.goal - self.previous_position)
            c = np.linalg.norm(self.position - self.previous_position)
            angle = math.acos((c**2 + b**2 - a**2) / (2 * c * b))
            direction = np.cross(np.append(self.position , np.zeros(1,))- np.append(self.previous_position , np.zeros(1,)), np.append(self.goal , np.zeros(1,)) - np.append(self.previous_position , np.zeros(1,)))
            if direction[-1] < 0:
                angle = -angle
            angle = math.degrees(angle)
            self.get_logger().info(f'angle: {angle}')
            if (abs(angle) > 1):
                msg.angular.z = math.copysign(0.25, angle)
        self.publisher.publish(msg)
    
    def goal_cb(self, msg):
        goal = np.array([msg.pose.position.x, msg.pose.position.y])
        if self.goal is None or (self.goal != goal).all():
            self.get_logger().info(f'received a new goal: (x={goal[0]}, y={goal[1]})')
            self.goal = goal
    
    def laser_cb(self, msg):
        buffer_angle = 15
        right_distances = msg.ranges[len(msg.ranges)-buffer_angle:]
        left_distances = msg.ranges[0:buffer_angle]
        self.forward_distance = min(min(left_distances), min(right_distances))
        
    def position_cb(self, msg):
        self.previous_position = self.position
        self.position = np.array([msg.point.x, msg.point.y])


def main(args=None):
    rclpy.init(args=args)

    node = VelocityController()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()