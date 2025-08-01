import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from geometry_msgs.msg import Twist
from robcomp_util.odom import Odom
from robcomp_util.laser import Laser
# Adicione aqui os imports necessários

class Girar(Node, Odom, Laser): # Mude o nome da classe

    def __init__(self):
        super().__init__('girar_node') # Mude o nome do nó
        Odom.__init__(self)
        self.timer = None

        self.robot_state = 'stop'
        self.state_machine = {
            'girar': self.girar,
            'stop': self.stop
        }

        # Inicialização de variáveis
        self.giro = 0.2
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
    
    @staticmethod
    def _wrap(angle):
        return np.arctan2(np.sin(angle), np.cos(angle))
    
    def reset(self, delta=np.pi/2):
        self.twist = Twist()
        self.goal_yaw = self._wrap(self.yaw + delta)
        self.robot_state = 'girar'
        if self.timer is None:
            self.timer = self.create_timer(0.25, self.control)
    
    def girar(self):
        
        self.erro = self._wrap(self.goal_yaw - self.yaw )

        print(f'Erro: {self.erro}')

        if self.erro > 0:
            self.twist.angular.z = self.giro
        else: 
            self.twist.angular.z = -self.giro

        if abs(self.erro) < np.radians(2):
                self.robot_state = 'stop'
    
    def stop(self):
        self.twist = Twist()
        print("Girar: Parando o robô.")
        self.timer.cancel()
        self.timer = None

    def control(self):
        print(f'Estado Atual: {self.robot_state}')
        self.state_machine[self.robot_state]()     
        self.cmd_vel_pub.publish(self.twist)
 
def main(args=None):
    rclpy.init(args=args)
    ros_node = Girar()

    rclpy.spin_once(ros_node)
    # Reset the node to initialize the goal yaw
    ros_node.reset()

    while not ros_node.robot_state == 'stop':
        rclpy.spin_once(ros_node)

    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()