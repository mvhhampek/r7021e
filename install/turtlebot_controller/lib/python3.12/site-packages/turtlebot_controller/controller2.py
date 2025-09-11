import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, Twist
from nav_msgs.msg import Odometry
import math

class TurtlebotController(Node):
    def __init__(self):
        super().__init__('turtlebot_controller')
        
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.Kp = 0.5
        self.Ki = 0.01
        self.Kh = 1.5
        self.dt = 0.1
        
        self.d_star = 0.5

        self.e_int = 0 # integral error
        
        self.x = 0
        self.y = 0
        self.theta = 0
        
        self.t = 0
        
    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.position.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1- 2.0*(q.y * q.y + q.z * q.z))
        self.theta = yaw
        
    def control_loop(self):
        self.t += dt
        
        # 8 trajectory (surely)
        x_star = 2.0 * math.sin(0.2*self.t)
        y_star = 2.0 * math.sin(0.2*self.t) * math.cos(0.2 * self.t)
        
        e = math.sqrt((x_star - self.x)**2 + (y_star - self.y)**2) - self.d_star
        self.e_int += e * self.dt
        
        v = self.Kp * e + self.Ki * self.e_int
        
        theta_star = math.atan2(y_star - self.y, x_star - self.x)
        
        a = self.Kh * (theta_star - self.theta)
        a = (a + math.pi) % (2 * math.pi) - math.pi
        
        twist = Twist()
        twist.linear.x = v
        twist.angular.z = a
        ts = TwistStamped()
        ts.twist = twist
        ts.header.stamp = self.get_clock().now().to_msg()
        ts.header.frame_id = ''

        self.cmd_pub.publish(ts)
        
        self.get_logger().info(f'/cmd_vel: Published [{twist.linear}], [{twist.angular}]')
        
        
def main(args = None):
    rclpy.init(args=args)
    node = TurtlebotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
