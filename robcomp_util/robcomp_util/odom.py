import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rclpy.qos import ReliabilityPolicy, QoSProfile
import numpy as np

class Odom():

    def __init__(self):
    
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        #Definindo um subscriber:
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE) )

        rclpy.spin_once(self) #Pra processar as mensagens recebidas pela primeira vez.


    def odom_callback(self, msg: Odometry):
        self.x = msg.pose.pose.position.x #Posição X do robõ no espaço global.
        self.y = msg.pose.pose.position.y #Posição Y do robõ no espaço global.

        orientation = msg.pose.pose.orientation


    def euler_from_quaternion(self, orientation):
    #Converte quaternion (formato [x, y, z, w]) para roll, pitch, yaw.

    x = orientation.x
    y = orientation.y
    z = orientation.z
    w = orientation.w

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw
        
        

