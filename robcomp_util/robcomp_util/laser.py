import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile

class Laser():
    def __init__(self):
        self.laser_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

        self.opening = 5

        rclpy.spin_once(self)

    def laser_callback(self, msg: LaserScan):
        self.laser_msg = np.array(msg.ranges).round(decimals=2)
        self.laser_msg[self.laser_msg == 0] = np.inf
        self.laser_msg = list(self.laser_msg)

        self.left = self.laser_msg[89-self.opening:89+self.opening]
        self.right = self.laser_msg[269-self.opening:269+self.opening]
        self.back = self.laser_msg[179-self.opening:179+self.opening]
        self.front = self.laser_msg[-self.opening:] + self.laser_msg[:self.opening]