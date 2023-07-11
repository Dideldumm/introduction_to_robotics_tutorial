import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, PoseStamped, PointStamped
from sensor_msgs.msg import LaserScan

class VelocityController(Node):

    def __init__(self):
        super().__init__('velocity_controller')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.goal = None
        self.position = None
        self.create_subscription(LaserScan, 'scan', self.laser_cb, rclpy.qos.qos_profile_sensor_data)
        self.create_subscription(PoseStamped, 'nav/goal', self.goal_cb, 10)
        self.create_subscription(PointStamped, 'position', self.position_cb, 10)
        self.create_timer(0.1, self.timer_cb)
        self.get_logger().info('controller node started')
        self.forward_distance = 0
        self.left_distance = 0
        self.right_distance = 0
        self.simulation_time = 0
        
    def timer_cb(self):
        msg = Twist()
        x = self.forward_distance - 0.2 - self.simulation_time / (10 * 60 * 2)
        if x < 0.0:
            msg.angular.z = 0.7
            msg.linear.x = 0.0
        else:
            msg.linear.x = 0.2
            msg.angular.z = 0.0

        if self.right_distance > self.left_distance:
            msg.angular.z = msg.angular.z * -1
        
        self.simulation_time += 1
        self.publisher.publish(msg)
    
    def goal_cb(self, msg):
        goal = msg.pose.position.x, msg.pose.position.y
        if self.goal != goal:
            self.get_logger().info(f'received a new goal: (x={goal[0]}, y={goal[1]})')
            self.goal = goal
    
    def laser_cb(self, msg):
        buffer_angle = 20
        right_distances = msg.ranges[len(msg.ranges)-buffer_angle:]
        left_distances = msg.ranges[0:buffer_angle]
        self.left_distance = msg.ranges[90]
        self.right_distance = msg.ranges[180 + 90]
        self.forward_distance = min(min(left_distances), min(right_distances))
        
    def position_cb(self, msg):
        self.position = msg.point.x, msg.point.y


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
