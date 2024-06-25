import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
# Adicione aqui os imports necessários
import numpy as np
import time
from my_package.odom import Odom
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy



class GoTo(Node, Odom): # Mude o nome da classe
    def __init__(self, point: Point = Point()):
        Node.__init__(self, 'quadrado_node') # Mude o nome do nó
        Odom.__init__(self) # Mude o nome do nó
        time.sleep(1)

        # Inicialização de variáveis
        self.twist = Twist()
        self.threshold = np.pi/180
        self.kp_linear = 0.8
        self.kp_angular = 2.
        self.point = point

        self.robot_state = 'center'
        self.state_machine = {
            'center': self.center,
            'goto': self.goto,
            'stop': self.stop
        }

        self.timer = self.create_timer(0.3, self.control)

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        # Subscribers
        transient_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.odom_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose', # '/odom',
            self.odom_callback,
            transient_qos)
        print('Odom initialized at amcl_pose topic - fixed')

    def euler_from_quaternion(self, quaternion : list):
            """
            Converts quaternion (w in last place) to euler roll, pitch, yaw
            quaternion = [x, y, z, w]
            Below should be replaced when porting for ROS2 Python tf_conversions is done.
            """
            x = quaternion[0]
            y = quaternion[1]
            z = quaternion[2]
            w = quaternion[3]

            sinr_cosp = 2 * (w * x + y * z)
            cosr_cosp = 1 - 2 * (x * x + y * y)
            roll = np.arctan2(sinr_cosp, cosr_cosp)

            sinp = 2 * (w * y - z * x)
            pitch = np.arcsin(sinp)

            siny_cosp = 2 * (w * z + x * y)
            cosy_cosp = 1 - 2 * (y * y + z * z)
            yaw = np.arctan2(siny_cosp, cosy_cosp)

            return roll, pitch, yaw

    def odom_callback(self, data: PoseWithCovarianceStamped):
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y

        quaternion = [
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w]
        
        self.roll, self.pitch, self.yaw = self.euler_from_quaternion(quaternion)

        self.yaw_2pi = (self.yaw + 2 * np.pi) % (2 * np.pi)

    def get_angular_error(self):
        if self.x == np.inf:
            self.erro = np.inf
            self.twist.angular.z = 0.
            return

        x = self.point.x - self.x
        y = self.point.y - self.y
        theta = np.arctan2(y , x)

        self.distance = np.sqrt(x**2 + y**2)
        erro = theta - self.yaw
        self.erro = np.arctan2(np.sin(erro), np.cos(erro))

        print('Erro: ', self.erro)
        self.twist.angular.z = self.erro * self.kp_angular

    def center(self):
        self.get_angular_error()

        if abs(self.erro) < np.deg2rad(3):
            self.robot_state = 'goto'

    def goto(self):
        self.get_angular_error()

        if self.distance > 0.01:
            self.twist.linear.x = self.distance * self.kp_linear
        else:
            self.robot_state = 'stop'
    
    def stop(self):
        self.twist = Twist()

    def control(self):
        self.twist = Twist()
        print(f'Estado Atual: {self.robot_state}')
        self.state_machine[self.robot_state]()

        self.cmd_vel_pub.publish(self.twist)
        
            
def main(args=None):
    rclpy.init(args=args)
    ros_node = GoTo(Point( x = 2., y = 1., z = 0.))

    rclpy.spin(ros_node)

    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()