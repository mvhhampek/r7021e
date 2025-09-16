import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, Twist
from nav_msgs.msg import Odometry
import math
from geometry_msgs.msg import Point

import matplotlib
matplotlib.use('Agg')

import matplotlib.pyplot as plt

"""
Sender Node to send the next waypoint to the velocity / position controller
Tracks current position and if it is close enough to the first waypoint (list of waypoints passed as argument), it publishes the next waypoint to the controller

"""


class WayPointer(Node):
    def __init__(self, waypoints):
        super().__init__('waypointer')
        self.waypoints = waypoints
        self.current_index = 0

        self.odom_sub = self.create_subscription(
            # to track robot position
            Odometry, 
            '/odom', 
            self.odom_callback, 
            10
        )
        self.waypoint_pub = self.create_publisher(Point, '/waypoint', 10)

        self.x = 0
        self.y = 0
        self.theta = 0

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info(f"Running Waypointer")
        self.current_index = 0
        self.waypoint_sent = False
        self.tolerance = 0.1  # e.g., 5 cm

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.theta = math.atan2(siny_cosp, cosy_cosp)



    def control_loop(self):
        if self.current_index >= len(self.waypoints):
            return  # all done

        x, y = self.waypoints[self.current_index]


        wp = Point()
        wp.x = x
        wp.y = y
        self.waypoint_pub.publish(wp)
        self.get_logger().info(f"Published waypoint {self.current_index}: x={x:.2f}, y={y:.2f}")


        # Now check if robot reached the waypoint
        distance = math.hypot(self.x - x, self.y - y)
        if distance < self.tolerance:
            self.get_logger().info(f"Reached waypoint {self.current_index}")
            self.current_index += 1
            self.waypoint_sent = False



def main(args=None):
    rclpy.init(args=args)
    # Example: figure-8 trajectory
    waypoints = [(math.sin(0.1*t), math.sin(0.1*t)*math.cos(0.1*t)) for t in range(100)]
    node = WayPointer(waypoints)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()