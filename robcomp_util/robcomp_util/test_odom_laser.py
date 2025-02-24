import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rclpy.qos import ReliabilityPolicy, QoSProfile
from robcomp_util.odom import Odom

class SecondNode(Node, Odom):

    def __init__(self):
        super().__init__('second_node')
        Odom.__init__(self)
        self.timer = self.create_timer(0.25, self.control)

    def control(self):
        print(f'Posição x: {self.x}')
        print(f'Posição y: {self.y}\n')


def main(args=None):
    rclpy.init(args=args)
    second_node = SecondNode()

    rclpy.spin(second_node)

    second_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()