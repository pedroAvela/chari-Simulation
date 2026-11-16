#!/usr/bin/env python3
"""
scan_filter.py
==============
Filtra os retornos do PROPRIO CORPO do robo num LiDAR 360 graus.

    /scan (360 cru)  ->  [scan_filter]  ->  /scan_filtered (limpo)

Por que existe:
  O LiDAR fica montado A FRENTE (x = 0.40 m no base_link). Numa varredura de
  360 graus ele "enxerga" o proprio corpo da cadeira atras dele (assento,
  encosto, ocupante), o que o arbitro leria como um obstaculo colado e
  permanente. Este no descarta esses retornos por geometria.

Como funciona:
  Cada feixe (r, ang) do LiDAR e convertido para o frame do robo:
      x_b = lidar_x + r*cos(ang)
      y_b = lidar_y + r*sin(ang)
  Se o ponto cai DENTRO do retangulo do corpo (footprint), ele e descartado
  (vira +inf = "nada ali"). O resto passa intacto.

Assim o robo ganha visao 360 real: passa a enxergar obstaculos que se aproximam
por TRAS e pelos LADOS (ex.: o carrinho movel), nao so os que estao a frente.
"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ScanFilter(Node):
    def __init__(self):
        super().__init__('scan_filter')

        self.declare_parameter('input_topic', '/scan')
        self.declare_parameter('output_topic', '/scan_filtered')

        # posicao do LiDAR no frame base_link (casar com o URDF!)
        self.declare_parameter('lidar_x', 0.40)
        self.declare_parameter('lidar_y', 0.0)

        # footprint do corpo a descartar (retangulo no frame base_link).
        # Corpo: x in [-0.32, 0.32], y in [-0.30, 0.30]; rodas ate |y| = 0.325.
        # Usamos uma margem de folga para nao deixar passar "sujeira" da borda.
        self.declare_parameter('body_x_min', -0.45)
        self.declare_parameter('body_x_max', 0.36)
        self.declare_parameter('body_y_abs', 0.40)

        gp = self.get_parameter
        in_topic = gp('input_topic').value
        out_topic = gp('output_topic').value
        self.lx = float(gp('lidar_x').value)
        self.ly = float(gp('lidar_y').value)
        self.bx_min = float(gp('body_x_min').value)
        self.bx_max = float(gp('body_x_max').value)
        self.by_abs = float(gp('body_y_abs').value)

        self.pub = self.create_publisher(LaserScan, out_topic, 10)
        self.create_subscription(LaserScan, in_topic, self._on_scan, 10)

        self.get_logger().info(
            f'Filtro de scan ativo: {in_topic} -> {out_topic} | '
            f'descartando footprint x[{self.bx_min}, {self.bx_max}] '
            f'|y|<{self.by_abs} (lidar em x={self.lx})')

    def _on_scan(self, msg: LaserScan):
        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        out.intensities = msg.intensities

        ranges = list(msg.ranges)
        for i, r in enumerate(ranges):
            if not math.isfinite(r) or r < msg.range_min or r > msg.range_max:
                continue
            ang = msg.angle_min + i * msg.angle_increment
            # ponto no frame do robo
            xb = self.lx + r * math.cos(ang)
            yb = self.ly + r * math.sin(ang)
            # dentro do corpo? -> descarta
            if self.bx_min <= xb <= self.bx_max and abs(yb) <= self.by_abs:
                ranges[i] = float('inf')

        out.ranges = ranges
        self.pub.publish(out)


def main():
    rclpy.init()
    node = ScanFilter()
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
