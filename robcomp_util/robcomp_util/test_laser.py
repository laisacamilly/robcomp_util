import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from robcomp_util.laser import Laser
from robcomp_util.odom import Odom


class TestLaser(Node, Laser, Odom): # Mude o nome da classe
    def __init__(self):
        super().__init__('test_laser_node') # Mude o nome do nó
        Laser.__init__(self)
        Odom.__init__(self)
        rclpy.spin_once(self) # Roda pelo menos uma vez para pegar os valores de x, y e front

        # Inicialização de variáveis
        
        # Subscribers
        ## Coloque aqui os subscribers

        # Publishers
        ## Coloque aqui os publishers

        # Por fim, inicialize o timer
        self.timer = self.create_timer(0.25, self.control)

    def control(self):
        print("self.x: ", self.x)
        print("self.y: ", self.y)
        print("self.yaw: ", self.yaw)
        print("Front: ", self.front)
        print("Muito perto?", min(self.front) < 1.0)
        
            
def main(args=None):
    rclpy.init(args=args)
    ros_node = TestLaser() # Mude o nome da classe

    rclpy.spin(ros_node)

    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()