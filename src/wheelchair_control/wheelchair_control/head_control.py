#!/usr/bin/env python3
"""
head_control.py
===============
CONTROLE PELA CABECA via WEBCAM - SOMENTE OpenCV (sem MediaPipe).

Detecta o rosto com Haar cascade (ja embutido no OpenCV) e usa a POSICAO do
rosto no quadro como gesto. Sem dependencias exoticas, sem conflito de NumPy.

    webcam -> [head_control] -> /cmd_vel_user -> [safety_arbiter] -> /cmd_vel

GESTOS (modo INCREMENTAL - a velocidade PERSISTE, nao cansa):
  - Abaixar a cabeca (rosto DESCE no quadro)  -> ACELERA (v sobe e fica)
  - Levantar a cabeca (rosto SOBE no quadro)  -> FREIA (v cai ate 0)
  - Cabeca no neutro                          -> MANTEM a velocidade (cruise)
  - Virar/inclinar a cabeca p/ um lado
    (rosto vai p/ o lado do quadro)           -> GIRA para aquele lado
  - Voltar ao centro                          -> giro volta a zero sozinho
  (Sem re: a velocidade linear tem piso 0.)

O sinal e normalizado pela LARGURA DO ROSTO, entao aproximar/afastar da
camera nao muda a sensibilidade.

Teclas na janela:
  c = calibrar o neutro (cabeca em repouso)
  s = parar tudo
  q = sair

Dependencia unica:  sudo apt install python3-opencv
"""
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

try:
    import cv2
    _DEPS_OK = True
    _IMPORT_ERR = ''
except ImportError as e:
    _DEPS_OK = False
    _IMPORT_ERR = str(e)


class HeadControl(Node):
    def __init__(self):
        super().__init__('head_control')

        self.declare_parameter('output_topic', '/cmd_vel_user')
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('show_preview', True)

        # limiares dos gestos, em FRACAO DA LARGURA DO ROSTO (adimensional)
        # ex.: 0.25 = mover o rosto 1/4 da propria largura ja dispara o gesto
        self.declare_parameter('pitch_threshold', 0.25)   # subir/descer
        self.declare_parameter('yaw_threshold', 0.30)     # esquerda/direita

        # dinamica incremental
        self.declare_parameter('accel', 0.3)          # m/s^2
        self.declare_parameter('turn_accel', 1.5)     # rad/s^2
        self.declare_parameter('turn_decay', 0.85)    # auto-centragem do giro
        self.declare_parameter('max_linear', 0.5)
        self.declare_parameter('max_angular', 1.0)

        self.declare_parameter('invert_pitch', False)
        self.declare_parameter('invert_yaw', False)
        self.declare_parameter('smooth_alpha', 0.4)   # filtro passa-baixa

        # --- RE (esquema A): com a cadeira PARADA, segurar o gesto de
        #     "levantar a cabeca" por `reverse_hold` segundos engata a RE.
        #     Voltar ao neutro por `reverse_exit` segundos desengata.
        self.declare_parameter('reverse_enable', True)
        self.declare_parameter('reverse_hold', 1.0)   # s segurando p/ engatar
        self.declare_parameter('reverse_exit', 1.0)   # s no neutro p/ desengatar
        self.declare_parameter('max_reverse', 0.25)   # m/s (re mais lenta)

        gp = self.get_parameter
        self.out_topic = gp('output_topic').value
        self.cam_idx = int(gp('camera_index').value)
        self.show = bool(gp('show_preview').value)
        self.pitch_thr = float(gp('pitch_threshold').value)
        self.yaw_thr = float(gp('yaw_threshold').value)
        self.accel = float(gp('accel').value)
        self.turn_accel = float(gp('turn_accel').value)
        self.turn_decay = float(gp('turn_decay').value)
        self.max_lin = float(gp('max_linear').value)
        self.max_ang = float(gp('max_angular').value)
        self.inv_pitch = bool(gp('invert_pitch').value)
        self.inv_yaw = bool(gp('invert_yaw').value)
        self.alpha = float(gp('smooth_alpha').value)
        self.rev_on = bool(gp('reverse_enable').value)
        self.rev_hold = float(gp('reverse_hold').value)
        self.rev_exit = float(gp('reverse_exit').value)
        self.max_rev = float(gp('max_reverse').value)

        # estado da marcha: +1 = FRENTE, -1 = RE
        self.gear = 1
        self.up_timer = 0.0        # tempo segurando "levantar" com v=0
        self.neutral_timer = 0.0   # tempo no neutro (p/ desengatar a re)

        self.pub = self.create_publisher(Twist, self.out_topic, 10)

        # estado
        self.v = 0.0
        self.w = 0.0
        self.f_cx = None          # centro do rosto filtrado (x)
        self.f_cy = None          # centro do rosto filtrado (y)
        self.f_fw = None          # largura do rosto filtrada
        self.neutral_x = None
        self.neutral_y = None
        self.last_t = time.monotonic()
        self.no_face_frames = 0

        # webcam
        self.cap = cv2.VideoCapture(self.cam_idx)
        if not self.cap.isOpened():
            self.get_logger().error(
                f'Nao consegui abrir a webcam (index {self.cam_idx}). '
                f'Cheque /dev/video* e o grupo "video".')
            raise RuntimeError('webcam indisponivel')

        # detector Haar - procura o cascade em varios locais possiveis
        # (o OpenCV do apt nao tem cv2.data; o do pip tem)
        import os
        candidatos = []
        if hasattr(cv2, 'data'):
            candidatos.append(cv2.data.haarcascades +
                              'haarcascade_frontalface_default.xml')
        candidatos += [
            '/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml',
            '/usr/share/opencv/haarcascades/haarcascade_frontalface_default.xml',
        ]
        casc = next((c for c in candidatos if os.path.exists(c)), None)
        self.detector = cv2.CascadeClassifier(casc) if casc else None
        if self.detector is None or self.detector.empty():
            self.get_logger().error(
                'Cascade nao encontrado. Rode: sudo apt install opencv-data')
            raise RuntimeError('cascade indisponivel')
        self.get_logger().info(f'Cascade carregado: {casc}')

        self.create_timer(1.0 / 20.0, self._loop)
        self.get_logger().info(
            f'Head control (OpenCV) ativo -> {self.out_topic}. '
            f'Fique com a cabeca em repouso e aperte "c" para calibrar. '
            f'c=calibrar  s=parar  q=sair')

    def _detect(self, frame):
        """Retorna (cx, cy, fw) do maior rosto, ou None."""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.equalizeHist(gray)
        faces = self.detector.detectMultiScale(
            gray, scaleFactor=1.15, minNeighbors=5, minSize=(80, 80))
        if len(faces) == 0:
            return None
        # maior rosto (o mais proximo da camera)
        x, y, w, h = max(faces, key=lambda f: f[2] * f[3])
        return (x + w / 2.0, y + h / 2.0, float(w))

    def _loop(self):
        ok, frame = self.cap.read()
        if not ok:
            return
        frame = cv2.flip(frame, 1)   # espelho (mais intuitivo)

        now = time.monotonic()
        dt = max(1e-3, now - self.last_t)
        self.last_t = now

        det = self._detect(frame)

        if det is not None:
            cx, cy, fw = det
            self.no_face_frames = 0
            # filtro passa-baixa (reduz tremor da deteccao)
            if self.f_cx is None:
                self.f_cx, self.f_cy, self.f_fw = cx, cy, fw
            else:
                a = self.alpha
                self.f_cx = (1 - a) * self.f_cx + a * cx
                self.f_cy = (1 - a) * self.f_cy + a * cy
                self.f_fw = (1 - a) * self.f_fw + a * fw

            if self.neutral_x is None:
                self.neutral_x = self.f_cx
                self.neutral_y = self.f_cy
                self.get_logger().info('Neutro auto-calibrado.')

            # deslocamento normalizado pela largura do rosto
            # (assim afastar/aproximar da camera nao muda a sensibilidade)
            scale = max(30.0, self.f_fw)
            dx = (self.f_cx - self.neutral_x) / scale   # >0 = rosto p/ direita
            dy = (self.f_cy - self.neutral_y) / scale   # >0 = rosto DESCEU

            if self.inv_pitch:
                dy = -dy
            if self.inv_yaw:
                dx = -dx

            # ---- LINEAR: incremental, com marcha FRENTE/RE (esquema A) ----
            # speed = modulo da velocidade; o sentido vem de self.gear
            speed = abs(self.v)
            v_max = self.max_lin if self.gear > 0 else self.max_rev

            if dy > self.pitch_thr:          # abaixou a cabeca -> acelera
                speed += self.accel * dt
                self.up_timer = 0.0
                self.neutral_timer = 0.0
            elif dy < -self.pitch_thr:       # levantou -> freia
                speed -= self.accel * dt
                self.neutral_timer = 0.0
                # RE: com a cadeira ja PARADA, segurar o gesto engata a re
                if self.rev_on and speed <= 0.001:
                    self.up_timer += dt
                    if self.gear > 0 and self.up_timer >= self.rev_hold:
                        self.gear = -1
                        self.up_timer = 0.0
                        self.get_logger().info('>>> RE engatada')
                else:
                    self.up_timer = 0.0
            else:                            # neutro -> cruise
                self.up_timer = 0.0
                # desengata a re se ficar parado no neutro
                if self.gear < 0 and speed <= 0.001:
                    self.neutral_timer += dt
                    if self.neutral_timer >= self.rev_exit:
                        self.gear = 1
                        self.neutral_timer = 0.0
                        self.get_logger().info('>>> FRENTE engatada')
                else:
                    self.neutral_timer = 0.0

            speed = max(0.0, min(v_max, speed))
            self.v = speed * self.gear

            # ---- GIRO (incremental, auto-centra) ----
            # rosto p/ a esquerda do quadro (dx<0) -> girar p/ a esquerda (w>0)
            if dx < -self.yaw_thr:
                self.w += self.turn_accel * dt
            elif dx > self.yaw_thr:
                self.w -= self.turn_accel * dt
            else:
                self.w *= self.turn_decay
            self.w = max(-self.max_ang, min(self.max_ang, self.w))

            self._dx, self._dy = dx, dy
        else:
            # sem rosto -> desacelera por seguranca
            self.no_face_frames += 1
            self.v *= 0.9
            self.w *= 0.9
            self._dx = self._dy = 0.0

        msg = Twist()
        msg.linear.x = self.v
        msg.angular.z = self.w
        self.pub.publish(msg)

        if self.show:
            self._draw(frame, det)
            k = cv2.waitKey(1) & 0xFF
            if k == ord('c') and det is not None:
                self.neutral_x = self.f_cx
                self.neutral_y = self.f_cy
                self.get_logger().info('Neutro recalibrado.')
            elif k == ord('s'):
                self.v = 0.0
                self.w = 0.0
                self.gear = 1
                self.up_timer = 0.0
                self.neutral_timer = 0.0
            elif k == ord('q'):
                raise KeyboardInterrupt

    def _draw(self, frame, det):
        h, w = frame.shape[:2]
        color = (0, 200, 0) if det is not None else (0, 0, 220)
        if det is not None:
            cx, cy, fw = det
            r = int(fw / 2)
            cv2.rectangle(frame, (int(cx - r), int(cy - r)),
                          (int(cx + r), int(cy + r)), color, 2)
        if self.neutral_x is not None:
            cv2.drawMarker(frame, (int(self.neutral_x), int(self.neutral_y)),
                           (255, 200, 0), cv2.MARKER_CROSS, 20, 2)
        cv2.putText(frame, f'v={self.v:+.2f}  w={self.w:+.2f}',
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
        # indicador de marcha
        gear_txt = 'FRENTE' if self.gear > 0 else 'RE'
        gear_col = (0, 200, 0) if self.gear > 0 else (0, 140, 255)
        cv2.putText(frame, gear_txt, (w - 130, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, gear_col, 2)
        if self.rev_on and self.up_timer > 0.05 and self.gear > 0:
            pct = min(1.0, self.up_timer / max(1e-6, self.rev_hold))
            cv2.putText(frame, f'engatando RE... {int(pct*100)}%',
                        (w - 260, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (0, 140, 255), 2)
        dx = getattr(self, '_dx', 0.0)
        dy = getattr(self, '_dy', 0.0)
        cv2.putText(frame, f'dx={dx:+.2f}  dy={dy:+.2f}',
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
        cv2.putText(frame, 'c=calibrar  s=parar  q=sair',
                    (10, h - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (255, 255, 255), 1)
        cv2.imshow('head_control', frame)

    def close(self):
        try:
            self.pub.publish(Twist())
            self.cap.release()
            cv2.destroyAllWindows()
        except Exception:
            pass


def main():
    if not _DEPS_OK:
        print('ERRO: OpenCV nao encontrado: ' + _IMPORT_ERR)
        print('Instale:  sudo apt install python3-opencv')
        return
    rclpy.init()
    try:
        node = HeadControl()
    except RuntimeError:
        rclpy.shutdown()
        return
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
