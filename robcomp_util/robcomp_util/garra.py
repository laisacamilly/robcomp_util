from rclpy.node import Node
from std_msgs.msg import Float64, String
import time

class Garra:
    
    def __init__(self):
        
        # Variável de delay para controlar tempo de espera
        self.delay = 2.0  # segundos
        
        # Publishers para controlar os servos da garra:
        self.ombro_pub = self.create_publisher( Float64, '/joint1_position_controller/command', 10)
        self.garra_pub = self.create_publisher( Float64, '/joint2_position_controller/command', 10)
    
    def controla_garra(self, comando):
        #Comandos disponíveis:
        #'abrir': abre a garra
        #'fechar': fecha a garra
        #'cima': move o ombro para cima
        #'baixo': move o ombro para baixo
        #'frente': move o ombro para frente
   
        msg = String()
        
        if comando == 'abrir':
            msg.data = 'abrir'
            self.garra_pub.publish(msg)
            time.sleep(self.delay)
            
        elif comando == 'fechar':
            msg.data = 'fechar'
            self.garra_pub.publish(msg)
            time.sleep(self.delay)
            
        elif comando == 'cima':
            msg.data = 'cima'
            self.ombro_pub.publish(msg)
            time.sleep(self.delay)
            
        elif comando == 'baixo':
            msg.data = 'baixo'
            self.ombro_pub.publish(msg)
            time.sleep(self.delay)
            
        elif comando == 'frente':
            msg.data = 'frente'
            self.ombro_pub.publish(msg)
            time.sleep(self.delay)
    
    # Métodos auxiliares para facilitar o uso
    def abrir_garra(self):
        self.controla_garra('abrir')
    
    def fechar_garra(self):
        self.controla_garra('fechar')
    
    def ombro_cima(self):
        self.controla_garra('cima')
    
    def ombro_baixo(self):
        self.controla_garra('baixo')
    
    def ombro_frente(self):
        self.controla_garra('frente')