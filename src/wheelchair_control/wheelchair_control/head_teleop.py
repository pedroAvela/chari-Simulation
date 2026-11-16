#!/usr/bin/env python3
"""
head_teleop.py
==============
CONTROLE PELA CABECA (fase futura do projeto - ja funcional como scaffold).

Este no e o objetivo final do anteprojeto: substituir teclado/joystick por
um IMU preso a cabeca do paciente. Ele foi feito para ser um DROP-IN: publica
no MESMO topico /cmd_vel_user, entao entra na arquitetura sem mexer em mais
nada (o safety_arbiter continua garantindo o desvio de obstaculos).

Entrada:  sensor_msgs/Imu em /head/imu
          (pode vir de um MPU6050 num ESP32/Arduino publicando via micro-ROS,
           ou de qualquer driver que exponha a orientacao da cabeca)

Mapeamento (ajustavel):
  - PITCH  (inclinar a cabeca p/ frente / tras) -> velocidade linear
  - YAW ou ROLL (virar / inclinar p/ os lados)  -> velocidade angular

Recursos de seguranca embutidos:
  - Zona morta (deadzone): pequenas variacoes nao geram movimento.
  - Calibracao de neutro: guarda a pose de repouso como "zero".
  - Saturacao: limita as velocidades maximas.
  - Watchdog: se o IMU para de publicar, para o robo.

Como calibrar o neutro em runtime:
  ros2 topic pub --once /head/calibrate std_msgs/msg/Empty "{}"
"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty


def quat_to_euler(x, y, z, w):
    """Quaternion -> (roll, pitch, yaw) em rad (convencao ZYX)."""
    # roll (x)
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)
    # pitch (y)
    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)
    # yaw (z)
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny, cosy)
    return roll, pitch, yaw


class HeadTeleop(Node):
    def __init__(self):
        super().__init__('head_teleop')

        self.declare_parameter('output_topic', '/cmd_vel_user')
        self.declare_parameter('imu_topic', '/head/imu')
        self.declare_parameter('steer_source', 'yaw')   # 'yaw' ou 'roll'

        # zona morta (rad) - abaixo disso nao anda
        self.declare_parameter('pitch_deadzone', 0.15)
        self.declare_parameter('steer_deadzone', 0.15)
        # angulo (rad) que corresponde a velocidade MAXIMA
        self.declare_parameter('pitch_full', 0.5)
        self.declare_parameter('steer_full', 0.6)
        # velocidades maximas
        self.declare_parameter('max_linear', 0.5)
        self.declare_parameter('max_angular', 1.0)
        # inverter sinais se necessario
        self.declare_parameter('invert_pitch', False)
        self.declare_parameter('invert_steer', False)
        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('rate', 20.0)

        self.out_topic = self.get_parameter('output_topic').value
        imu_topic = self.get_parameter('imu_topic').value
        self.steer_source = self.get_parameter('steer_source').value
        self.pitch_dz = float(self.get_parameter('pitch_deadzone').value)
        self.steer_dz = float(self.get_parameter('steer_deadzone').value)
        self.pitch_full = float(self.get_parameter('pitch_full').value)
        self.steer_full = float(self.get_parameter('steer_full').value)
        self.max_lin = float(self.get_parameter('max_linear').value)
        self.max_ang = float(self.get_parameter('max_angular').value)
        self.inv_pitch = bool(self.get_parameter('invert_pitch').value)
        self.inv_steer = bool(self.get_parameter('invert_steer').value)
        self.timeout = float(self.get_parameter('cmd_timeout').value)
        rate = float(self.get_parameter('rate').value)

        self.neutral_pitch = 0.0
        self.neutral_steer = 0.0
        self.cur_pitch = 0.0
        self.cur_steer = 0.0
        self.last_imu_time = None
        self._auto_calibrated = False

        self.pub = self.create_publisher(Twist, self.out_topic, 10)
        self.create_subscription(Imu, imu_topic, self._on_imu, 20)
        self.create_subscription(Empty, '/head/calibrate', self._on_calib, 1)
        self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(
            f'Head teleop: {imu_topic} -> {self.out_topic} '
            f'(steer={self.steer_source}). Mantenha a cabeca parada 1s '
            f'para auto-calibrar o neutro.')

    def _on_imu(self, msg: Imu):
        q = msg.orientation
        roll, pitch, yaw = quat_to_euler(q.x, q.y, q.z, q.w)
        self.cur_pitch = pitch
        self.cur_steer = yaw if self.steer_source == 'yaw' else roll
        if self.last_imu_time is None and not self._auto_calibrated:
            # primeira leitura: usa como neutro provisorio
            self.neutral_pitch = self.cur_pitch
            self.neutral_steer = self.cur_steer
            self._auto_calibrated = True
            self.get_logger().info('Neutro auto-calibrado na 1a leitura.')
        self.last_imu_time = self.get_clock().now()

    def _on_calib(self, _msg: Empty):
        self.neutral_pitch = self.cur_pitch
        self.neutral_steer = self.cur_steer
        self.get_logger().info('Neutro recalibrado.')

    @staticmethod
    def _map(value, deadzone, full, vmax, invert):
        mag = abs(value)
        if mag <= deadzone:
            return 0.0
        span = max(1e-6, full - deadzone)
        norm = min(1.0, (mag - deadzone) / span)
        out = math.copysign(norm * vmax, value)
        return -out if invert else out

    def _tick(self):
        msg = Twist()
        if self.last_imu_time is not None:
            age = (self.get_clock().now() - self.last_imu_time).nanoseconds * 1e-9
            if age <= self.timeout:
                dp = self.cur_pitch - self.neutral_pitch
                ds = self.cur_steer - self.neutral_steer
                msg.linear.x = self._map(dp, self.pitch_dz, self.pitch_full,
                                         self.max_lin, self.inv_pitch)
                msg.angular.z = self._map(ds, self.steer_dz, self.steer_full,
                                          self.max_ang, self.inv_steer)
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = HeadTeleop()
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
