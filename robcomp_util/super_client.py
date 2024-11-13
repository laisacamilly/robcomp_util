import numpy as np
from geometry_msgs.msg import Point
from robcomp_interfaces.action import GoToPoint, SimpleStart
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from robcomp_util.client_base import BaseActionClientNode
from robcomp_util.odom import Odom

class SuperClient(Node, Odom):
    def __init__(self):
        """
        Inicializa o cliente de ação específico de 'GoToPoint'.
        """
        super().__init__('super_client')
        Odom.__init__(self)
        self.goto_action_client = BaseActionClientNode('goto_action_client', GoToPoint, 'goto_point')
        self.segue_linha_client = BaseActionClientNode('segue_linha_client', SimpleStart, 'segue_linha')

        # Estado inicial da máquina de estados
        self.robot_state = 'goto'
        self.state_machine = {
            'goto': self.goto,
            'segue_linha': self.segue_linha,
            'waiting_for_goto': self.waiting_for_goto,
            'waiting_for_segue_linha': self.waiting_for_segue_linha,
            'stop': self.stop
        }

        # Cria um timer para controlar o robô
        self.timer = self.create_timer(0.25, self.control)

    def goto(self):
        """
        Envia um objetivo de movimentação para um ponto específico.
        """
        goal_msg = GoToPoint.Goal()
        goal_msg.goal = Point(x=-2.0, y=-2.0, z=0.0)
        self.goto_action_client.send_goal(goal_msg)
        self.robot_state = 'waiting_for_goto'

    def waiting_for_goto(self):
        """
        Verifica se o objetivo foi concluído e muda para o estado 'stop' se terminado.
        """
        print(self.goto_action_client._goal_done)
        rclpy.spin_once(self.goto_action_client, timeout_sec=0.1)
        if self.goto_action_client._goal_done:
            self.robot_state = 'segue_linha'
    
    def segue_linha(self):
        goal_msg = SimpleStart.Goal()
        self.segue_linha_client.send_goal(goal_msg)
        self.robot_state = 'waiting_for_segue_linha'
        self.hora_de_parar = False
        self.pos_init = (self.x, self.y)

    def distance(self):
        """
        Calcula a distância entre dois pontos.
        """
        return np.sqrt((self.x - self.pos_init[0])**2 + (self.y - self.pos_init[1])**2)
    
    def waiting_for_segue_linha(self):
        rclpy.spin_once(self.segue_linha_client, timeout_sec=0.1)
        dist = self.distance()
        self.get_logger().info(f'Distância percorrida: {dist:.2f} metros')

        if self.hora_de_parar is False and dist > 0.6:
            self.get_logger().info('Robô atingiu 0.6 metros, preparado para parar...')
            self.hora_de_parar = True

        elif self.hora_de_parar is True and dist < 0.5:
            self.get_logger().info('Cancelando objetivo, o robô está perto demais.')
            if self.segue_linha_client._goal_handle is not None:
                self.segue_linha_client._goal_handle.cancel_goal_async()  # Envia o comando para cancelar o objetivo
            self.robot_state = 'stop'

    def stop(self):
        """
        Para o robô, publicando um comando de velocidade zero.
        """
        self.twist = Twist()

    def control(self):
        """
        Executa a máquina de estados, chamando a função correspondente ao estado atual.
        """
        self.twist = Twist()
        print(self.robot_state)
        self.state_machine[self.robot_state]()


def main(args=None):
    """
    Função principal que inicializa o ROS2, cria o Action Client específico
    e mantém o nó rodando até ser encerrado.
    """
    rclpy.init(args=args)
    ros_node = SuperClient()

    while rclpy.ok():
        rclpy.spin_once(ros_node)

    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
