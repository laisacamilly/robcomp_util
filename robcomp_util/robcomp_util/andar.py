import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
from robcomp_util.odom import Odom

class Andar(Node, Odom): # Mude o nome da classe

    def __init__(self, node = 'andar_node'): # Mude o nome do nó
        super().__init__(node)
        Odom.__init__(self)
        self.timer = None

        self.robot_state = 'done' # Comece em 'done' - reset iniciará a ação
        self.state_machine = { # Adicione quantos estados forem necessários
            'andar': self.andar,
            'stop': self.stop,
            'done': self.stop
        }

        # Inicialização de variáveis
        self.velocidade = 0.2

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Subscribers
        # ...
    
    def reset(self, distancia = 1.0):
        self.twist = Twist()
        self.robot_state = 'andar' # Inicie a ação
        if self.timer is None:
            self.timer = self.create_timer(0.25, self.control) # Timer para o controle
        
        ### Iniciar variaveis da ação
        self.threshold = distancia / self.velocidade
        current_time = self.get_clock().now().to_msg()
        self.tempo_inicial = float(current_time.sec) + float(current_time.nanosec)/10**9

    def andar(self):
        self.twist.linear.x = self.velocidade  # m/s
        current_time = self.get_clock().now().to_msg()
        current_time = float(current_time.sec) + float(current_time.nanosec)/10**9
        dt = current_time - self.tempo_inicial
        if dt > self.threshold:
            self.robot_state = 'stop'
            self.twist = Twist()
        # 4. Se o delta é maior ou igual a t segundos, pare o robô
        pass

    def stop(self):
        self.twist = Twist() # Zera a velocidade
        print("Parando o robô.")
        self.timer.cancel() # Finaliza o timer
        self.timer = None # Reseta a variável do timer
        self.robot_state = 'done' # Ação finalizada

    def control(self): # Controla a máquina de estados - eh chamado pelo timer
        print(f'Estado Atual: {self.robot_state}')
        self.state_machine[self.robot_state]() # Chama o método do estado atual 
        self.cmd_vel_pub.publish(self.twist) # Publica a velocidade
 
def main(args=None):
    rclpy.init(args=args) # Inicia o ROS2
    ros_node = Andar() # Cria o nó

    rclpy.spin_once(ros_node) # Processa as callbacks uma vez
    ros_node.reset() # Reseta o nó para iniciar a ação

    while not ros_node.robot_state == 'done': # Enquanto a ação não estiver finalizada
        rclpy.spin_once(ros_node) # Processa os callbacks e o timer

    ros_node.destroy_node() # Destroi o nó
    rclpy.shutdown()    # Finaliza o ROS2

if __name__ == '__main__': # Executa apenas se for o arquivo principal
    main()