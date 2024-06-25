import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.duration import Duration
import time

class Nav2GoalSender(Node):
    def __init__(self):
        super().__init__('nav2_goal_sender')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.goal_pose = PoseStamped()
        self.goal_pose.header.frame_id = 'map'
        self.goal_pose.pose.position.x = 2.  # Defina a posição x do objetivo
        self.goal_pose.pose.position.y = 0.0  # Defina a posição y do objetivo
        self.goal_pose.pose.orientation.w = 1.0  # Defina a orientação do objetivo
        
        self.send_goal()

    def send_goal(self):
        while not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Aguardando o servidor de ação...')
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.goal_pose
        
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Objetivo recusado :(')
            return

        self.get_logger().info('Objetivo aceito :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Resultado: {result}')
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Recebendo feedback: {feedback}')

def main(args=None):
    rclpy.init(args=args)
    node = Nav2GoalSender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
