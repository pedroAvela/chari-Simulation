#!/usr/bin/env python3
"""
safety_arbiter.py
=================
No de ARBITRAGEM / AUTONOMIA COMPARTILHADA - protecao 360 graus.

    /cmd_vel_user  (intencao)  --+
                                 |--> [ARBITRO] --> /cmd_vel
    /scan_filtered (LiDAR 360) --+

FILOSOFIA: AUTORIDADE GRADUADA - O USUARIO VENCE NA DIRECAO, NAO NA FISICA.
  O robo ASSISTE, nao pilota. A autoridade dele no GIRO cresce com o risco mas
  tem TETO (`assist_authority`), e RECUA se o usuario comanda giro de proposito
  (deferencia: "ele sabe o que esta fazendo").
  MAS a FRENAGEM de seguranca NAO cede a deferencia: voce escolhe para onde ir,
  nao escolhe bater a toda velocidade. Um assistente que nao pode ser sobreposto
  e um piloto automatico; um que cede tudo e inutil.

COMPORTAMENTOS:
  1) CORREDOR / PORTA (bloqueado dos dois lados)
     -> CENTRALIZA pela DIFERENCA das folgas. No meio, as forcas se cancelam
        e a cadeira segue RETA.
  2) OBSTACULO ISOLADO (livre de um lado)
     -> desvia p/ o lado livre, com histerese (sem sanfona).
  3) OBSTACULO DINAMICO SE APROXIMANDO (ex.: o carrinho, vindo de qualquer
     direcao - inclusive de tras/lado)
     -> detecta a TAXA DE APROXIMACAO (derivada da distancia por setor) e
        freia/afasta, mesmo que voce nao esteja indo naquela direcao.
  4) PROTECAO DO TRASEIRO NA MANOBRA
     -> ao girar, a traseira "varre" lateralmente. Se ha algo perto no lado
        varrido, o giro e limitado (evita raspar a roda ao contornar).
  5) RE (opcional): protege tambem andando p/ tras, usando o setor traseiro.
  6) EMERGENCIA: freia de verdade; o giro continua LIVRE p/ manobrar e sair.
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class SafetyArbiter(Node):
    def __init__(self):
        super().__init__('safety_arbiter')

        self.declare_parameter('input_topic', '/cmd_vel_user')
        self.declare_parameter('output_topic', '/cmd_vel')
        self.declare_parameter('scan_topic', '/scan_filtered')

        # geometria
        self.declare_parameter('robot_radius', 0.38)
        self.declare_parameter('influence_distance', 1.8)
        self.declare_parameter('emergency_distance', 0.40)
        self.declare_parameter('side_fov_deg', 100.0)
        self.declare_parameter('corridor_range', 1.2)

        # autoridade graduada
        self.declare_parameter('assist_authority', 0.5)
        self.declare_parameter('user_override_gain', 1.0)

        # ganhos
        self.declare_parameter('steer_gain', 1.5)
        self.declare_parameter('center_gain', 1.2)
        self.declare_parameter('slow_gain', 0.6)
        self.declare_parameter('min_speed_factor', 0.35)
        self.declare_parameter('max_angular', 1.5)
        self.declare_parameter('deflect_hold', 0.8)

        # --- obstaculo dinamico (aproximacao) ---
        self.declare_parameter('approach_enable', True)
        self.declare_parameter('approach_speed_thr', 0.25)  # m/s de aproximacao
        self.declare_parameter('approach_dist', 1.5)        # so vale se < isto
        self.declare_parameter('approach_brake', 0.7)       # quanto freia (0..1)

        # --- protecao do traseiro na manobra ---
        self.declare_parameter('rear_guard_enable', True)
        self.declare_parameter('rear_guard_dist', 0.55)     # folga minima lateral

        # --- re ---
        self.declare_parameter('reverse_enable', True)

        self.declare_parameter('cmd_timeout', 0.6)
        self.declare_parameter('rate', 20.0)

        gp = self.get_parameter
        in_topic = gp('input_topic').value
        out_topic = gp('output_topic').value
        scan_topic = gp('scan_topic').value
        self.radius = float(gp('robot_radius').value)
        self.influence = float(gp('influence_distance').value)
        self.emergency = float(gp('emergency_distance').value)
        self.side_fov = math.radians(float(gp('side_fov_deg').value))
        self.corridor_range = float(gp('corridor_range').value)
        self.authority = float(gp('assist_authority').value)
        self.override_gain = float(gp('user_override_gain').value)
        self.steer_gain = float(gp('steer_gain').value)
        self.center_gain = float(gp('center_gain').value)
        self.slow_gain = float(gp('slow_gain').value)
        self.min_factor = float(gp('min_speed_factor').value)
        self.max_ang = float(gp('max_angular').value)
        self.deflect_hold = float(gp('deflect_hold').value)
        self.appr_on = bool(gp('approach_enable').value)
        self.appr_thr = float(gp('approach_speed_thr').value)
        self.appr_dist = float(gp('approach_dist').value)
        self.appr_brake = float(gp('approach_brake').value)
        self.rear_on = bool(gp('rear_guard_enable').value)
        self.rear_dist = float(gp('rear_guard_dist').value)
        self.rev_on = bool(gp('reverse_enable').value)
        self.timeout = float(gp('cmd_timeout').value)
        self.rate = float(gp('rate').value)

        self.last_cmd = Twist()
        self.last_cmd_time = None
        self.scan = None
        self.deflect_dir = 0
        self.deflect_until = 0
        self.prev_sectors = None    # p/ detectar aproximacao
        self.prev_t = None

        self.pub = self.create_publisher(Twist, out_topic, 10)
        self.create_subscription(Twist, in_topic, self._on_cmd, 10)
        self.create_subscription(LaserScan, scan_topic, self._on_scan, 10)
        self.create_timer(1.0 / self.rate, self._tick)

        self.get_logger().info(
            f'Arbitro 360 ativo: {in_topic} + {scan_topic} -> {out_topic} | '
            f'autoridade={self.authority:.2f} (usuario vence na direcao), '
            f'aproximacao={self.appr_on}, guarda-traseira={self.rear_on}, '
            f're={self.rev_on}')

    def _on_cmd(self, msg: Twist):
        self.last_cmd = msg
        self.last_cmd_time = self.get_clock().now()

    def _on_scan(self, msg: LaserScan):
        self.scan = msg

    # ---------- percepcao ----------
    def _sectors(self):
        """Menor distancia em 8 setores de 45 graus (0 = frente, sentido CCW).
           Usado para detectar aproximacao vinda de QUALQUER direcao."""
        s = self.scan
        sec = [float('inf')] * 8
        if s is None:
            return sec
        for i, r in enumerate(s.ranges):
            if not math.isfinite(r) or r < s.range_min or r > s.range_max:
                continue
            ang = s.angle_min + i * s.angle_increment
            k = int(((ang + math.pi / 8.0) % (2 * math.pi)) / (math.pi / 4.0)) % 8
            sec[k] = min(sec[k], r)
        return sec

    def _analyze(self, forward=True):
        """(front, left, right) no sentido do movimento."""
        s = self.scan
        if s is None:
            return self.influence, self.influence, self.influence
        front = left = right = self.influence
        center = 0.0 if forward else math.pi
        for i, r in enumerate(s.ranges):
            if not math.isfinite(r) or r < s.range_min or r >= self.influence:
                continue
            ang = s.angle_min + i * s.angle_increment
            d = _wrap(ang - center)          # relativo ao sentido do movimento
            if abs(d) > self.side_fov:
                continue
            x = r * math.cos(d)              # + = a frente (do movimento)
            y = r * math.sin(d)              # + = a esquerda
            if x > 0.0 and abs(y) < self.radius:
                front = min(front, r)
            if d > 0.0:
                left = min(left, r)
            else:
                right = min(right, r)
        return front, left, right

    def _rear_clearance(self, side):
        """Folga no quadrante TRASEIRO do lado dado (+1 esq, -1 dir).
           Ao girar, a traseira varre para o lado OPOSTO ao giro."""
        s = self.scan
        if s is None:
            return float('inf')
        lo, hi = (math.radians(100), math.radians(180)) if side > 0 else \
                 (math.radians(-180), math.radians(-100))
        best = float('inf')
        for i, r in enumerate(s.ranges):
            if not math.isfinite(r) or r < s.range_min or r > s.range_max:
                continue
            ang = _wrap(s.angle_min + i * s.angle_increment)
            if lo <= ang <= hi:
                best = min(best, r)
        return best

    def _approach_brake(self, dt):
        """Fator de frenagem (0..1) por obstaculo se APROXIMANDO de qualquer lado.
           1.0 = sem frenagem; <1 = freia."""
        if not self.appr_on or dt <= 0.0:
            return 1.0
        sec = self._sectors()
        factor = 1.0
        if self.prev_sectors is not None:
            for k in range(8):
                d_now, d_prev = sec[k], self.prev_sectors[k]
                if not (math.isfinite(d_now) and math.isfinite(d_prev)):
                    continue
                if d_now > self.appr_dist:
                    continue
                closing = (d_prev - d_now) / dt        # m/s (positivo = vem vindo)
                if closing > self.appr_thr:
                    sev = min(1.0, (closing - self.appr_thr) / max(1e-6, self.appr_thr))
                    prox = 1.0 - min(1.0, d_now / self.appr_dist)
                    factor = min(factor, 1.0 - self.appr_brake * sev * prox)
        self.prev_sectors = sec
        return max(0.0, factor)

    # ---------- controle ----------
    def _tick(self):
        out = Twist()
        now_ns = self.get_clock().now().nanoseconds
        dt = (now_ns - self.prev_t) * 1e-9 if self.prev_t else 1.0 / self.rate
        self.prev_t = now_ns

        v_user, w_user = 0.0, 0.0
        if self.last_cmd_time is not None:
            age = (now_ns - self.last_cmd_time.nanoseconds) * 1e-9
            if age <= self.timeout:
                v_user = self.last_cmd.linear.x
                w_user = self.last_cmd.angular.z

        # frenagem por aproximacao roda SEMPRE (ate parado, algo pode vir em voce)
        appr = self._approach_brake(dt)

        if abs(v_user) <= 0.05 and abs(w_user) <= 0.05:
            self.pub.publish(out)
            return

        v_out, w_out = v_user, w_user
        forward = v_user > 0.05
        reverse = v_user < -0.05 and self.rev_on

        if forward or reverse:
            front, left, right = self._analyze(forward=forward)

            risk = 0.0
            if front < self.influence:
                risk = max(0.0, min(1.0, (self.influence - front) /
                                    max(1e-6, self.influence - self.radius)))

            in_corridor = (left < self.corridor_range and
                           right < self.corridor_range)

            # ----- assistencia de giro (so p/ frente; na re nao esterça) -----
            w_assist = 0.0
            if forward:
                if in_corridor:
                    gap = max(1e-6, left + right)
                    w_assist = self.center_gain * ((left - right) / gap)
                    self.deflect_dir = 0
                elif front < self.influence:
                    if self.deflect_dir == 0:
                        self.deflect_dir = 1 if left >= right else -1
                    self.deflect_until = now_ns + int(self.deflect_hold * 1e9)
                    w_assist = self.steer_gain * risk * self.deflect_dir
                else:
                    if now_ns >= self.deflect_until:
                        self.deflect_dir = 0

            # ----- AUTORIDADE GRADUADA (so no giro) -----
            authority = min(self.authority,
                            self.authority * (1.0 if in_corridor else risk))
            user_intent = min(1.0, abs(w_user) / max(1e-6, self.max_ang))
            authority *= max(0.0, 1.0 - self.override_gain * user_intent)
            w_out = w_user + authority * w_assist

            # ----- velocidade: risco + aproximacao (NAO cedem a deferencia) -----
            v_out = v_user * max(self.min_factor, 1.0 - self.slow_gain * risk)
            v_out *= appr

            if front <= self.emergency:
                v_out = 0.0
        else:
            # girando no lugar: so a frenagem por aproximacao age no linear
            v_out = v_user * appr

        # ----- GUARDA-TRASEIRA: ao girar, a traseira varre p/ o lado oposto -----
        if self.rear_on and abs(w_out) > 0.05:
            sweep = -1 if w_out > 0 else 1          # giro CCW varre p/ a direita
            clear = self._rear_clearance(sweep)
            if clear < self.rear_dist:
                scale = max(0.0, clear / max(1e-6, self.rear_dist))
                w_out *= scale                      # limita o giro perto da parede

        w_out = max(-self.max_ang, min(self.max_ang, w_out))
        out.linear.x = v_out
        out.angular.z = w_out
        self.pub.publish(out)


def _wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def main():
    rclpy.init()
    node = SafetyArbiter()
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
