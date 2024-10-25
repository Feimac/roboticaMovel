#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from rclpy.qos import QoSProfile
from math import atan2, sqrt, pi

class TurtleMoveToPosition(Node):
    def __init__(self, target_x=2.0, target_y=2.0):
        super().__init__('turtle_move_to_position')

        # Publicador e assinante
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', QoSProfile(depth=10))
        self.subscription = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, QoSProfile(depth=10))

        # Posição atual da tartaruga
        self.turtle_x = 0.0
        self.turtle_y = 0.0
        self.turtle_theta = 0.0

        # Posição alvo
        self.target_x = target_x
        self.target_y = target_y

    def pose_callback(self, msg):
        """Atualiza a pose da tartaruga."""
        self.turtle_x = msg.x
        self.turtle_y = msg.y
        self.turtle_theta = msg.theta

    def normalize_angle(self, angle):
        """Normaliza o ângulo para o intervalo [-pi, pi]."""
        while angle > pi:
            angle -= 2 * pi
        while angle < -pi:
            angle += 2 * pi
        return angle

    def move_towards_target(self):
        """Move a tartaruga em direção ao alvo."""
        twist = Twist()
        
        # Calcula a diferença entre a posição atual e o alvo
        diff_x = self.target_x - self.turtle_x
        diff_y = self.target_y - self.turtle_y
        distance = sqrt(diff_x**2 + diff_y**2)

        # Para a tartaruga se ela estiver perto do alvo
        if distance < 0.1:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher.publish(twist)
            self.get_logger().info("Tartaruga chegou ao destino!")
            return

        # Calcula o ângulo para o alvo e normaliza a diferença angular
        angle_to_target = atan2(diff_y, diff_x)
        angular_error = self.normalize_angle(angle_to_target - self.turtle_theta)

        # Ajusta a velocidade linear e angular
        twist.linear.x = min(distance, 1.0)  # Limita a velocidade linear
        twist.angular.z = 1.5 * angular_error  # Ajuste proporcional para rotação

        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)

    # Definir o destino (pode ser alterado dinamicamente)
    target_x = float(input("Digite a coordenada X do destino: "))
    target_y = float(input("Digite a coordenada Y do destino: "))

    turtle_move = TurtleMoveToPosition(target_x, target_y)

    # Loop principal
    try:
        while rclpy.ok():
            rclpy.spin_once(turtle_move)
            turtle_move.move_towards_target()
    except KeyboardInterrupt:
        pass  # Permite encerrar com Ctrl+C

    turtle_move.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

