import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from robcomp_util.odom import Odom

# Adicione aqui os imports necessários

class TestOdomNode(Node, Odom): # Mude o nome da classe

    def __init__(self):
        super().__init__('test_odom_node') # Mude o nome do nó
        Odom.__init__(self)
        rclpy.spin_once(self) # Roda pelo menos uma vez para pegar os valores de x, y e front

        self.timer = self.create_timer(0.25, self.control)

    def control(self):
        print("self.x: ", self.x)
        print("self.y: ", self.y)
        print("self.yaw: ", self.yaw)

def main(args=None):
    rclpy.init(args=args)
    ros_node = TestOdomNode() # Mude o nome da classe

    rclpy.spin(ros_node)

    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()