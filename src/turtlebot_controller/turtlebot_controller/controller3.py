import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, Twist
from nav_msgs.msg import Odometry
import math

import matplotlib
matplotlib.use('Agg')

import matplotlib.pyplot as plt

class TurtlebotController(Node):
    def __init__(self):
        super().__init__('turtlebot_controller')
        
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
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
        
        # 8 trajectory (surely)
        x_star = 2.5 * math.sin(0.10*self.t)
        y_star = 2.5 * math.sin(0.10*self.t) * math.cos(0.10 * self.t)
        
        # line trajectory
       # x_star = 0.2*self.t
       # y_star = 0.2*self.t

        self.traj_x.append(x_star)
        self.traj_y.append(y_star)
        self.path_x.append(self.x)
        self.path_y.append(self.y)
        
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
        
        self.get_logger().info(f'v: {v},,,,,,,,,, a: {a}')
        
        
    def plot(self, fname='plt.png', show=False):
        if not self.traj_x or not self.path_x:
            return
            
        #self.get_logger().info(f'end time: {self.t}')
        #self.get_logger().info(f'{self.path_x[100:110]}')
        
        plt.figure()
        plt.plot(self.traj_x, self.traj_y, label = "desired traj")
        plt.plot(self.path_x, self.path_y, label = "burger path")
        plt.axis('equal')
        plt.grid(True)
        plt.legend()
        plt.title(f"end time: {self.t}")
        plt.savefig(fname, dpi=150)
        if show:
            plt.show()
        plt.close()
    
def main(args = None):
    rclpy.init(args=args)
    node = TurtlebotController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = 0

        ts = TwistStamped()
        ts.twist = twist
        ts.header.stamp = 0
        ts.header.frame_id = ''

        node.cmd_pub.publish(ts)

    finally:
        node.plot('figure8.png', False)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
