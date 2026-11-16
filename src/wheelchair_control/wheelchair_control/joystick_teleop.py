#!/usr/bin/env python3
"""
joystick_teleop.py
==================
Teleoperacao da cadeira por JOYSTICK.

Assina sensor_msgs/Joy (do pacote 'joy' / joy_node) e publica
geometry_msgs/Twist em /cmd_vel_user.

Mapeamento padrao (controle tipo Xbox):
  - eixo vertical do analogico esquerdo  -> frente/re
  - eixo horizontal do analogico esquerdo-> giro
  - botao "deadman" (LB) precisa estar PRESSIONADO para andar (seguranca)
  - botao "turbo"  (RB) aumenta a velocidade

Todos os indices sao parametros, entao voce adapta a qualquer controle.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


class JoystickTeleop(Node):
    def __init__(self):
        super().__init__('joystick_teleop')

        self.declare_parameter('output_topic', '/cmd_vel_user')
        self.declare_parameter('linear_axis', 1)      # analogico esq. vertical
        self.declare_parameter('angular_axis', 0)     # analogico esq. horizontal
        self.declare_parameter('linear_scale', 0.6)   # m/s
        self.declare_parameter('angular_scale', 1.2)  # rad/s
        self.declare_parameter('deadman_button', 4)   # LB
        self.declare_parameter('turbo_button', 5)     # RB
        self.declare_parameter('turbo_scale', 1.6)
        self.declare_parameter('require_deadman', True)

        topic = self.get_parameter('output_topic').value
        self.lin_axis = self.get_parameter('linear_axis').value
        self.ang_axis = self.get_parameter('angular_axis').value
        self.lin_scale = self.get_parameter('linear_scale').value
        self.ang_scale = self.get_parameter('angular_scale').value
        self.deadman = self.get_parameter('deadman_button').value
        self.turbo = self.get_parameter('turbo_button').value
        self.turbo_scale = self.get_parameter('turbo_scale').value
        self.require_deadman = self.get_parameter('require_deadman').value

        self.pub = self.create_publisher(Twist, topic, 10)
        self.sub = self.create_subscription(Joy, '/joy', self._on_joy, 10)
        self.get_logger().info(
            f'Joystick pronto. Publicando em "{topic}". '
            f'Segure o botao {self.deadman} (deadman) para andar.')

    def _on_joy(self, joy: Joy):
        msg = Twist()

        enabled = True
        if self.require_deadman:
            enabled = (len(joy.buttons) > self.deadman
                       and joy.buttons[self.deadman] == 1)

        if enabled:
            scale = 1.0
            if len(joy.buttons) > self.turbo and joy.buttons[self.turbo] == 1:
                scale = self.turbo_scale
            if len(joy.axes) > self.lin_axis:
                msg.linear.x = joy.axes[self.lin_axis] * self.lin_scale * scale
            if len(joy.axes) > self.ang_axis:
                msg.angular.z = joy.axes[self.ang_axis] * self.ang_scale * scale

        self.pub.publish(msg)


def main():
    rclpy.init()
    node = JoystickTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
