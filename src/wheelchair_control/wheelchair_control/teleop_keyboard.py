#!/usr/bin/env python3
"""
teleop_keyboard.py
==================
Teleoperacao da cadeira pelo TECLADO.

Publica geometry_msgs/Twist em /cmd_vel_user (padrao). Esse topico e a
"entrada de usuario" da arquitetura de autonomia compartilhada:

    [teclado | joystick | cabeca]  ->  /cmd_vel_user  ->  [safety_arbiter]  ->  /cmd_vel

Comportamento: o comando fica ATIVO ate voce apertar 'k' ou espaco (parar).
Rode SEMPRE em um terminal proprio (precisa de TTY):

    ros2 run wheelchair_control teleop_keyboard

Para pular o arbitro e ir direto ao robo:

    ros2 run wheelchair_control teleop_keyboard --ros-args -p output_topic:=/cmd_vel
"""
import sys
import termios
import tty
import select

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

MSG = """
------------------------------------------------
  Cadeira de rodas - teleop de teclado
------------------------------------------------
Movimento:
     u    i    o
     j    k    l
     m    ,    .

  i / ,  : frente / re
  j / l  : girar esquerda / direita
  u o m .: combinacoes (frente/re + giro)
  k / espaco : PARAR

Ajuste de velocidade:
  q / z : + / - velocidade geral (10%)
  w / x : + / - somente linear
  e / c : + / - somente angular

  CTRL-C : sair
------------------------------------------------
"""

# tecla -> (linear, angular) em fracoes das velocidades maximas
MOVE_BINDINGS = {
    'i': (1.0, 0.0),
    ',': (-1.0, 0.0),
    'j': (0.0, 1.0),
    'l': (0.0, -1.0),
    'u': (1.0, 1.0),
    'o': (1.0, -1.0),
    'm': (-1.0, 1.0),
    '.': (-1.0, -1.0),
}
STOP_KEYS = ('k', ' ')

# tecla -> (fator_linear, fator_angular)
SPEED_BINDINGS = {
    'q': (1.1, 1.1),
    'z': (0.9, 0.9),
    'w': (1.1, 1.0),
    'x': (0.9, 1.0),
    'e': (1.0, 1.1),
    'c': (1.0, 0.9),
}


class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')
        self.declare_parameter('output_topic', '/cmd_vel_user')
        self.declare_parameter('linear_speed', 0.5)    # m/s
        self.declare_parameter('angular_speed', 1.0)   # rad/s
        self.declare_parameter('publish_rate', 20.0)   # Hz

        topic = self.get_parameter('output_topic').value
        self.lin = float(self.get_parameter('linear_speed').value)
        self.ang = float(self.get_parameter('angular_speed').value)
        rate = float(self.get_parameter('publish_rate').value)

        self.pub = self.create_publisher(Twist, topic, 10)
        self.target = (0.0, 0.0)  # (fracao_linear, fracao_angular)

        self.settings = termios.tcgetattr(sys.stdin)
        self.timer = self.create_timer(1.0 / rate, self._publish)
        self.get_logger().info(f'Publicando em "{topic}"')
        print(MSG)
        self._print_speeds()

    def _print_speeds(self):
        print(f'\r  [linear={self.lin:.2f} m/s | angular={self.ang:.2f} rad/s]     ')

    def _get_key(self):
        tty.setraw(sys.stdin.fileno())
        r, _, _ = select.select([sys.stdin], [], [], 0.0)
        key = sys.stdin.read(1) if r else ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def _publish(self):
        key = self._get_key()
        if key in MOVE_BINDINGS:
            self.target = MOVE_BINDINGS[key]
        elif key in STOP_KEYS:
            self.target = (0.0, 0.0)
        elif key in SPEED_BINDINGS:
            fl, fa = SPEED_BINDINGS[key]
            self.lin = max(0.05, min(1.0, self.lin * fl))
            self.ang = max(0.05, min(1.5, self.ang * fa))
            self._print_speeds()
        elif key == '\x03':  # CTRL-C
            raise KeyboardInterrupt

        fl, fa = self.target
        msg = Twist()
        msg.linear.x = fl * self.lin
        msg.angular.z = fa * self.ang
        self.pub.publish(msg)

    def stop(self):
        self.pub.publish(Twist())
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


def main():
    rclpy.init()
    node = TeleopKeyboard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
