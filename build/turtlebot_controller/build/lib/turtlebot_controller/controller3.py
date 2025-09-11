import math
import time
import rclpy
from rclpy.time import Time
from rclpy.clock import Clock
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped, Twist


class TurtlebotController(Node):
    def __init__(self):
        super().__init__('turtlebot_controller')
        
        self.current_pose = None
        self.ranges = None
        self.start_time = self.get_clock().now()
        self.trajectory_type = 'figure8'


        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)



    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        self.follow_trajectory()

    def scan_callback(self, msg):
        self.ranges = msg.ranges

    def follow_trajectory(self):
        if self.current_pose is None:
            return

        t = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        
        ref_x, ref_y = traj_figure8(t)
        dx = ref_x - self.current_pose.position.x
        dy = ref_y - self.current_pose.position.y

        distance = math.hypot(dx, dy)
        desired_yaw = math.atan2(dy, dx)

        # Current yaw from quaternion
        q = self.current_pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        current_yaw = math.atan2(siny_cosp, cosy_cosp)

        yaw_error = desired_yaw - current_yaw
        # Normalize yaw error to [-pi, pi]
        while yaw_error > math.pi:
            yaw_error -= 2 * math.pi
        while yaw_error < -math.pi:
            yaw_error += 2 * math.pi

        # Gains (tune these)
        k_linear = 0.3
        k_angular = 1.0

        twist = Twist()
        if abs(yaw_error) < math.pi / 6:  # only move forward if roughly facing target
            twist.linear.x = min(k_linear * distance, 0.3)
        else:
            twist.linear.x = 0.0

        twist.angular.z = max(-0.5, min(k_angular * yaw_error, 0.5))

        ts = TwistStamped()
        ts.twist = twist
        ts.header.stamp = self.get_clock().now().to_msg()
        ts.header.frame_id = ''

        self.cmd_pub.publish(ts)
        self.get_logger().info(f'Published [{twist.linear}], [{twist.angular}] on /cmd_vel')


def traj_figure8(t):
    r = 3
    x = r * math.cos(t)
    y = r * math.sin(2 * t) / 2
    return x, y
    



def main(args = None):
    rclpy.init(args=args)
    node = TurtlebotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
