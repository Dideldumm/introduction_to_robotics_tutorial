import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class VelocityController(Node):
    def __init__(self):
        super().__init__("velocity_controller")
        self.publisher = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_subscription(
            LaserScan, "scan", self.laser_cb, rclpy.qos.qos_profile_sensor_data
        )
        self.create_subscription(String, "status", self.score_status_cb, 10)
        self.create_timer(0.1, self.timer_cb)
        self.get_logger().info("controller node started")
        self.active = True
        self.score_status = 0
        self.scan_time = 0.0
        self.forward_distance = 0

    def timer_cb(self):
        msg = Twist()
        x = self.forward_distance - 0.4
        if self.score_status > 15:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
        elif x < 0.0:
            msg.angular.z = 0.7
            msg.linear.x = 0.0
        else:
            msg.linear.x = 0.2
            msg.angular.z = 0.0
        if self.active:
            self.get_logger().info(
                "Score status: "
                + str(self.score_status)
                + " - Distance: "
                + str(self.forward_distance)
                + " - Scan time: "
                + str(self.scan_time)
            )

        self.publisher.publish(msg)

    def laser_cb(self, msg):
        if self.score_status % 10 == 0:
            self.get_logger().info(str(self.forward_distance))
        self.scan_time = msg.scan_time
        self.forward_distance = msg.ranges[0]

    def score_status_cb(self, msg):
        self.score_status = int(msg.data) if msg.data.isdigit() else self.score_status


def main(args=None):
    rclpy.init(args=args)

    node = VelocityController()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
