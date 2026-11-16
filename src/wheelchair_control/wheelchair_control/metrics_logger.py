#!/usr/bin/env python3
"""
metrics_logger.py
=================
Registra metricas de desempenho da cadeira durante uma corrida, para o
experimento de ablacao COM vs SEM assistencia (autonomia compartilhada).

Assina:
  /scan          -> folga ao obstaculo mais proximo (proxy de seguranca)
  /odom          -> trajetoria, distancia percorrida, velocidade
  /cmd_vel_user  -> intencao do usuario (bruta)
  /cmd_vel       -> comando efetivo (pos-arbitro)

Gera:
  <output_dir>/metrics_<label>.csv       -> serie temporal
  <output_dir>/metrics_<label>_resumo.txt-> resumo agregado (impresso ao sair)

Uso:
  # rodada COM assistencia (arbitro no circuito)
  ros2 run wheelchair_control metrics_logger --ros-args \\
     -p label:=com_assist -p use_sim_time:=true

  # rodada SEM assistencia (teleop direto em /cmd_vel)
  ros2 run wheelchair_control metrics_logger --ros-args \\
     -p label:=sem_assist -p use_sim_time:=true

Encerre com CTRL-C ao terminar o percurso: o resumo e gravado e impresso.
"""
import os
import csv
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class MetricsLogger(Node):
    def __init__(self):
        super().__init__('metrics_logger')
        self.declare_parameter('label', 'run')
        self.declare_parameter('output_dir', '~/wheelchair_metrics')
        self.declare_parameter('collision_dist', 0.20)   # m (proxy de colisao)
        self.declare_parameter('nearmiss_dist', 0.45)    # m (quase-colisao)
        self.declare_parameter('sample_rate', 20.0)      # Hz

        self.label = self.get_parameter('label').value
        out_dir = os.path.abspath(os.path.expanduser(self.get_parameter('output_dir').value))
        os.makedirs(out_dir, exist_ok=True)
        self.csv_path = os.path.join(out_dir, f'metrics_{self.label}.csv')
        self.summary_path = os.path.join(out_dir, f'metrics_{self.label}_resumo.txt')
        self.collision_dist = float(self.get_parameter('collision_dist').value)
        self.nearmiss_dist = float(self.get_parameter('nearmiss_dist').value)
        rate = float(self.get_parameter('sample_rate').value)

        # estado
        self.min_range = float('inf')
        self.pos = None
        self.prev_pos = None
        self.yaw = 0.0
        self.speed = 0.0
        self.v_user = 0.0
        self.w_user = 0.0
        self.v_out = 0.0
        self.w_out = 0.0

        # acumuladores
        self.t0 = None
        self.path_length = 0.0
        self.min_clearance = float('inf')
        self.clear_sum = 0.0
        self.clear_n = 0
        self.collisions = 0
        self.near_misses = 0
        self._below_col = False
        self._below_near = False
        self.interv_sum = 0.0
        self.interv_n = 0
        self.interv_active = 0

        # csv
        self.csv_file = open(self.csv_path, 'w', newline='')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(['t', 'x', 'y', 'yaw', 'min_range',
                              'v_user', 'w_user', 'v_out', 'w_out'])

        self.create_subscription(LaserScan, '/scan_filtered', self._on_scan, 10)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.create_subscription(Twist, '/cmd_vel_user', self._on_user, 10)
        self.create_subscription(Twist, '/cmd_vel', self._on_out, 10)
        self.create_timer(1.0 / rate, self._sample)

        self.get_logger().info(
            f'Gravando metricas [{self.label}] em {self.csv_path} '
            f'(CTRL-C para finalizar).')

    def _on_scan(self, msg: LaserScan):
        m = float('inf')
        for r in msg.ranges:
            if math.isfinite(r) and msg.range_min <= r <= msg.range_max:
                m = min(m, r)
        self.min_range = m

    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        self.pos = (p.x, p.y)
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny, cosy)
        vx = msg.twist.twist.linear.x
        self.speed = vx
        if self.prev_pos is not None:
            dx = self.pos[0] - self.prev_pos[0]
            dy = self.pos[1] - self.prev_pos[1]
            self.path_length += math.hypot(dx, dy)
        self.prev_pos = self.pos

    def _on_user(self, msg: Twist):
        self.v_user = msg.linear.x
        self.w_user = msg.angular.z

    def _on_out(self, msg: Twist):
        self.v_out = msg.linear.x
        self.w_out = msg.angular.z

    def _sample(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.t0 is None:
            self.t0 = now
        t = now - self.t0

        mr = self.min_range
        if math.isfinite(mr):
            self.min_clearance = min(self.min_clearance, mr)
            self.clear_sum += mr
            self.clear_n += 1
            # contagem de eventos (transicao para dentro do limiar)
            below_col = mr < self.collision_dist
            if below_col and not self._below_col:
                self.collisions += 1
            self._below_col = below_col
            below_near = mr < self.nearmiss_dist
            if below_near and not self._below_near:
                self.near_misses += 1
            self._below_near = below_near

        # intervencao do arbitro (diferenca intencao vs comando efetivo)
        di = abs(self.v_out - self.v_user) + abs(self.w_out - self.w_user)
        self.interv_sum += di
        self.interv_n += 1
        if di > 0.02:
            self.interv_active += 1

        x, y = (self.pos if self.pos else (float('nan'), float('nan')))
        self.writer.writerow([f'{t:.3f}', f'{x:.3f}', f'{y:.3f}',
                              f'{self.yaw:.3f}', f'{mr:.3f}',
                              f'{self.v_user:.3f}', f'{self.w_user:.3f}',
                              f'{self.v_out:.3f}', f'{self.w_out:.3f}'])

    def finish(self):
        try:
            self.csv_file.flush()
            self.csv_file.close()
        except Exception:
            pass

        dur = 0.0
        if self.t0 is not None:
            dur = self.get_clock().now().nanoseconds * 1e-9 - self.t0
        mean_clear = (self.clear_sum / self.clear_n) if self.clear_n else float('nan')
        mean_interv = (self.interv_sum / self.interv_n) if self.interv_n else 0.0
        pct_interv = (100.0 * self.interv_active / self.interv_n) if self.interv_n else 0.0

        lines = [
            f'===== RESUMO [{self.label}] =====',
            f'Duracao ................ {dur:.1f} s',
            f'Distancia percorrida ... {self.path_length:.2f} m',
            f'Folga MINIMA ........... {self.min_clearance:.3f} m',
            f'Folga media ............ {mean_clear:.3f} m',
            f'Colisoes (<{self.collision_dist:.2f} m) ... {self.collisions}',
            f'Quase-colisoes (<{self.nearmiss_dist:.2f} m) . {self.near_misses}',
            f'Intervencao media ...... {mean_interv:.3f}',
            f'Tempo intervindo ....... {pct_interv:.1f} %',
            '(colisao/folga sao proxy via LiDAR, nao contato fisico)',
        ]
        text = '\n'.join(lines)
        try:
            with open(self.summary_path, 'w') as f:
                f.write(text + '\n')
        except Exception:
            pass
        print('\n' + text + '\n')
        self.get_logger().info(f'Resumo salvo em {self.summary_path}')


def main():
    rclpy.init()
    node = MetricsLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.finish()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
