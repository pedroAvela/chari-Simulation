#!/usr/bin/env python3
"""
obstacle_mover.py
=================
Anima os obstaculos MOVEIS do mundo (person_a, person_b, cart).

Cada obstaculo tem o sistema VelocityControl no SDF e assina
/model/<nome>/cmd_vel (bridge ROS->gz). Este no publica uma velocidade
constante que INVERTE de sinal a cada 'half_period' segundos, gerando um
vai-e-vem (patrulha) dentro dos limites da sala.

Como os modelos nao giram (yaw=0), o eixo do corpo coincide com o do mundo,
entao basta comandar linear.x (patrulha em X) ou linear.y (patrulha em Y).
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class Mover:
    def __init__(self, name, axis, speed, half_period):
        self.name = name
        self.axis = axis            # 'x' ou 'y'
        self.speed = speed          # m/s
        self.half_period = half_period  # s por sentido


class ObstacleMover(Node):
    def __init__(self):
        super().__init__('obstacle_mover')
        self.declare_parameter('speed_scale', 1.0)
        scale = float(self.get_parameter('speed_scale').value)

        # Configuracao dos moveis (casada com as poses iniciais do clinic.sdf)
        self.movers = [
            Mover('cart', 'x', 0.25 * scale, 6.0),  # patrulha lado norte
        ]

        self.pubs = {
            m.name: self.create_publisher(Twist, f'/model/{m.name}/cmd_vel', 10)
            for m in self.movers
        }
        self.t0 = self.get_clock().now()
        self.create_timer(0.1, self._tick)  # 10 Hz
        self.get_logger().info(
            'Animando obstaculos: ' + ', '.join(m.name for m in self.movers))

    def _tick(self):
        t = (self.get_clock().now() - self.t0).nanoseconds * 1e-9
        for m in self.movers:
            # sentido alterna a cada half_period (paridade do intervalo)
            direction = 1.0 if int(t / m.half_period) % 2 == 0 else -1.0
            v = direction * m.speed
            msg = Twist()
            if m.axis == 'x':
                msg.linear.x = v
            else:
                msg.linear.y = v
            self.pubs[m.name].publish(msg)


def main():
    rclpy.init()
    node = ObstacleMover()
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
