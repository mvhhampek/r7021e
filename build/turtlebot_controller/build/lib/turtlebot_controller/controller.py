import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, Twist, Point
from nav_msgs.msg import Odometry
import math

import matplotlib
matplotlib.use('Agg')

import matplotlib.pyplot as plt

"""
Controller that receives goal position and uses PID to control linear and angular velocity
"""


class TurtlebotController(Node):
    def __init__(self):
        super().__init__('controller')

        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.wp_sub = self.create_subscription(Point, '/waypoint', self.wp_callback, 10)

        # Publisher
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Target waypoint
        self.x_star = 0.0
        self.y_star = 0.0

        # Trajectory tracking
        self.traj_x = []  # waypoints received
        self.traj_y = []
        self.path_x = []  # actual robot path
        self.path_y = []

        # PID / control parameters
        self.Kp = 2.0
        self.Ki = 0.025
        self.Kh = 2.0
        self.dt = 0.1
        self.v_max = 1.0
        self.w_max = 2.5
        self.d_star = 0.0

        # moving window integral error (only last N errors)
        self.e_int = 0.0
        self.window_size = 20
        self.e_index = 0
        self.e_window = [0.0] * self.window_size

        # Timer for control loop
        self.timer = self.create_timer(self.dt, self.control_loop)

        self.get_logger().info("Controller node running")

    def odom_callback(self, msg):
        # Update robot pose
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.path_x.append(self.x)
        self.path_y.append(self.y)

        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.theta = math.atan2(siny_cosp, cosy_cosp)

    def wp_callback(self, msg):
        # Receive new waypoint
        self.x_star = msg.x
        self.y_star = msg.y
        self.traj_x.append(msg.x)
        self.traj_y.append(msg.y)
        #self.get_logger().info(f"Received waypoint: x={msg.x:.2f}, y={msg.y:.2f}")

    def control_loop(self):
        # Compute errors
        e = math.hypot(self.x_star - self.x, self.y_star - self.y) - self.d_star
        
        # integral window
        self.e_window[self.e_index] = e * self.dt
        self.e_index = (self.e_index + 1) % self.window_size
        self.e_int = sum(self.e_window)

        # Linear velocity
        v = self.Kp * e + self.Ki * self.e_int
        v = max(min(v, self.v_max), -self.v_max)

        # Angular velocity
        theta_star = math.atan2(self.y_star - self.y, self.x_star - self.x)
        a = self.Kh * (theta_star - self.theta)
        a = (a + math.pi) % (2 * math.pi) - math.pi
        a = max(min(a, self.w_max), -self.w_max)

        # Publish TwistStamped
        twist = Twist()
        twist.linear.x = v
        twist.angular.z = a
        ts = TwistStamped()
        ts.twist = twist
        ts.header.stamp = self.get_clock().now().to_msg()
        self.cmd_pub.publish(ts)

        # Logging
        self.get_logger().info(
            f"pos:({self.x:.2f},{self.y:.2f}), "
            f"ref:({self.x_star:.2f},{self.y_star:.2f}), "
            f"cmd:(v:{v:.2f},w:{a:.2f}), "
            f"e:({e:.3f}, int:{self.e_int:.3f})"
        )

    def plot(self, fname='figure8.png'):
        if not self.traj_x or not self.path_x:
            self.get_logger().warn("No data to plot")
            return

        plt.figure()
        plt.plot(self.traj_x, self.traj_y, linewidth = '5', label="Desired Trajectory")
        plt.plot(self.path_x, self.path_y, label="Actual Robot Path")
        plt.axis('equal')
        plt.grid(True)
        plt.legend()
        plt.title("Figure-8 Trajectory Tracking")
        plt.savefig(fname, dpi=150)
        plt.close()
        self.get_logger().info(f"Trajectory plot saved to {fname}")


def main(args=None):
    rclpy.init(args=args)
    node = TurtlebotController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.plot('figure8.png')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
