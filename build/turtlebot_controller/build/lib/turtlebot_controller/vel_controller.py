import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, Twist, Point
from nav_msgs.msg import Odometry
import math

#import matplotlib
#matplotlib.use('Agg')
#import matplotlib.pyplot as plt

class TurtlebotController(Node):
    def __init__(self):
        super().__init__('turtlebot_controller')
        
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        self.waypoint_sub = self.create_subscription(Point, '/waypoint', self.waypoint_callback, 10) 
        
        self.timer = self.create_timer(0.1, self.control_loop)
        self.dt = 0.1
        
        # vel
        self.Kp = 1.2
        self.Ki = 0.00 # integral part is fake anyway
        self.v_max = 0.7
        self.e_int = 0 # integral error
        
        # heading
        self.Kh = 2.0
        self.w_max = 2.5

        self.d_star = 0.1

        self.t = 0 
        self.x = 0
        self.y = 0
        self.theta = 0 
        
        #plt
        self.traj_x, self.traj_y = [], []
        self.path_x, self.path_y = [], []
        
    def waypoint_callback(self.msg):
        self.x_star = msg.x
        self.y_star = msg.y
        
    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # manual quaternion to euler xd
        q = msg.pose.pose.orientation
        n = (q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)**2
        x,y,z,w = q.x/n, q.y/n, q.z/n, q.w/n 
        yaw = math.atan2(2.0 * (w*z + x*y), 1.0 - 2.0*(y*y+z*z))
        self.theta = yaw
        


    def control_loop(self):
        self.t += self.dt
        
        # position
        e = math.hypot(x_star - self.x, y_star - self.y) - self.d_star
        self.e_int += e * self.dt
        self.e_int = 0
        #self.e_int = max(min(self.e_int, 1.0), -1.0) 
        
        v = self.Kp * e + self.Ki * self.e_int
        
        v = max(min(v, self.v_max), -self.v_max)


        # heading
        theta_star = math.atan2(y_star - self.y, x_star - self.x)
        a = self.Kh * (theta_star - self.theta)

        a = (a + math.pi) % (2 * math.pi) - math.pi
        a = max(min(a, self.w_max), -self.w_max)
        

        twist = Twist()
        twist.linear.x = v
        twist.angular.z = a

        ts = TwistStamped()
        ts.twist = twist
        ts.header.stamp = self.get_clock().now().to_msg()
        ts.header.frame_id = ''

        self.cmd_pub.publish(ts)
        
        self.get_logger().info(f'e: {e},,,,,,,,,, e_int: {self.e_int}')
        
def main(args = None):
    rclpy.init(args=args)
    node = TurtlebotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
