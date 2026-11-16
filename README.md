# Cadeira de Rodas Robotizada — Simulação ROS 2 + Gazebo

Workspace de simulação da **cadeira de rodas robotizada com navegação assistida
e controle compartilhado** (anteprojeto do Pedro Henrique / UFES).

**Alvo:** ROS 2 **Jazzy** + Gazebo **Harmonic** (gz-sim) · Ubuntu 24.04

---

## 1. Ideia central da arquitetura

Tudo converge para **um único tópico de comando** `/cmd_vel`. As fontes de
entrada (teclado, joystick e, no futuro, o **movimento da cabeça**) são apenas
*publishers intercambiáveis*. Trocar a fonte **não exige mexer no robô nem na
simulação**.

```
  [ teclado ]  ─┐
  [ joystick ] ─┼─►  /cmd_vel_user ─►  [ safety_arbiter ] ─►  /cmd_vel ─►  robô (Gazebo)
  [ cabeça/IMU ]┘                            ▲
                                             │
                                          /scan (LiDAR)
```

O **safety_arbiter** é o "nó de arbitragem" citado no anteprojeto (item 5):
implementa a **autonomia compartilhada** — o usuário dá a intenção e o robô
**assiste o desvio**, esterçando em torno dos obstáculos por um campo
potencial (LiDAR), **sem parar a cadeira**. A parada total só ocorre em
**emergência** (obstáculo muito perto logo à frente). Funciona sem o Nav2;
o Nav2 pode ser somado depois publicando na mesma `/cmd_vel`.

Parâmetros úteis do arbitro (ajustáveis via `--ros-args -p`):
`influence_distance` (alcance do desvio), `emergency_distance` (limiar de
parada), `steer_gain` (agressividade do contorno), `min_speed_factor`
(piso de velocidade perto de obstáculos).

---

## 2. Pacotes

| Pacote                   | Conteúdo |
|--------------------------|----------|
| `wheelchair_description` | Modelo URDF/xacro (tração diferencial + LiDAR 2D + IMU + câmera), plugins gz-sim, RViz |
| `wheelchair_gazebo`      | Mundo `clinic.sdf` (obstáculos fixos e móveis), `ros_gz_bridge`, launch da simulação |
| `wheelchair_control`     | Nós: teleop teclado, teleop joystick, arbitro de segurança, controle pela cabeça (futuro), animador de obstáculos |

---

## 3. Dependências

O ambiente ROS 2 / Gazebo já está configurado. Instale só os pacotes usados:

```bash
sudo apt update
sudo apt install \
  ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge \
  ros-jazzy-robot-state-publisher ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-xacro ros-jazzy-joy ros-jazzy-rviz2
```

---

## 4. Compilar

```bash
cd ~/wheelchair_ws          # (ou onde você extraiu)
colcon build --symlink-install
source install/setup.bash
```

Coloque o `source` no `~/.bashrc` ou repita em cada terminal novo.

---

## 5. Rodar

### 5.1 Simulação (Gazebo + robô + RViz + obstáculos)

```bash
ros2 launch wheelchair_gazebo simulation.launch.py
```

Se o Gazebo não abrir a janela sozinho, clique no ▶ (play) no canto inferior
esquerdo do gz para iniciar a física. Sem RViz: `... simulation.launch.py rviz:=false`.

### 5.2 Dirigir com TECLADO (recomendado: teleop nativo)

Instale uma vez: `sudo apt install ros-jazzy-teleop-twist-keyboard`

**Opção A — direto ao robô (mais simples, sem arbitro):**
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
O teleop nativo publica em `/cmd_vel` por padrão — vai direto para o robô.

**Opção B — com autonomia compartilhada (passa pelo arbitro):**
```bash
# Terminal A — arbitro de seguranca
ros2 launch wheelchair_control safety.launch.py
# Terminal B — teleop nativo remapeado para a entrada do usuario
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r /cmd_vel:=/cmd_vel_user
```

> Existe tambem o no proprio `ros2 run wheelchair_control teleop_keyboard`
> (publica em `/cmd_vel_user`), mas ele SEMPRE depende do arbitro rodando.
> Se o robo nao anda, quase sempre e porque o arbitro nao foi iniciado —
> por isso a Opcao A acima e a mais confiavel para o primeiro teste.

**Teste decisivo de movimento** (com a sim aberta, sem teleop):
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}" -r 10
```
Se isso NAO mover o robo, o problema e o nome do topico no lado do gz:
`gz topic -l | grep cmd_vel` e ajuste o `bridge.yaml`.

### 5.3 Dirigir com JOYSTICK

```bash
ros2 launch wheelchair_control teleop_joystick.launch.py
```

Segure **LB** (deadman) para andar; **RB** = turbo. Analógico esquerdo
controla frente/ré e giro. Ajuste os índices em
`wheelchair_control/config/joystick.yaml` se seu controle for diferente
(descubra com `ros2 topic echo /joy`).

### 5.4 Atalho "sem segurança" (comando direto)

```bash
ros2 run wheelchair_control teleop_keyboard --ros-args -p output_topic:=/cmd_vel
# ou
ros2 launch wheelchair_control teleop_joystick.launch.py use_safety:=false
```

---

## 6. Fase futura — controle pela CABEÇA

Já existe o nó `head_teleop`, pronto como *drop-in*. Ele lê um IMU preso à
cabeça em `/head/imu` (sensor_msgs/Imu) e publica no mesmo `/cmd_vel_user`:

- **pitch** (inclinar a cabeça p/ frente/trás) → velocidade linear
- **yaw/roll** (virar/inclinar para os lados) → giro

Com zona morta, calibração de neutro, saturação e watchdog. Quando tiver o
hardware (ex.: MPU6050 num ESP32 publicando via micro-ROS), basta:

```bash
ros2 launch wheelchair_control safety.launch.py           # arbitro
ros2 run wheelchair_control head_teleop                    # cabeça -> /cmd_vel_user
# recalibrar o "neutro" com a cabeça em repouso:
ros2 topic pub --once /head/calibrate std_msgs/msg/Empty "{}"
```

Nada mais muda: o arbitro e o robô continuam idênticos.

---

## 7. Tópicos principais

| Tópico | Tipo | Sentido |
|--------|------|---------|
| `/cmd_vel_user` | geometry_msgs/Twist | entrada do usuário (teclado/joystick/cabeça) |
| `/cmd_vel` | geometry_msgs/Twist | comando final (pós-arbitro) → robô |
| `/scan` | sensor_msgs/LaserScan | LiDAR 2D |
| `/odom` | nav_msgs/Odometry | odometria |
| `/imu` | sensor_msgs/Imu | IMU do chassi |
| `/camera/image` | sensor_msgs/Image | câmera RGB |
| `/joint_states` | sensor_msgs/JointState | rodas |
| `/tf`, `/tf_static` | tf2_msgs/TFMessage | árvore de frames |
| `/model/<nome>/cmd_vel` | geometry_msgs/Twist | comando dos obstáculos móveis |

---

## 8. O mundo `clinic.sdf`

- Sala fechada ~12 × 10 m, uma parede interna com **porta** (~1,8 m).
- **Fixos:** 3 pilares, 2 mesas, 2 caixas.
- **Móveis:** `person_a` (atravessa a porta), `person_b` (patrulha leste),
  `cart` (patrulha norte) — animados pelo `obstacle_mover`.

O robô nasce em `(-4.5, -1.5)` olhando para +x (área aberta, de frente para
a porta, com ~3 m livres). Edite as poses no `.sdf`/launch e,
se mudar as posições iniciais dos móveis, ajuste `obstacle_mover.py`.

---

## 9. Correspondência com o anteprojeto

| Etapa do anteprojeto | Onde está |
|----------------------|-----------|
| 3. Construção de cenários no Gazebo | `worlds/clinic.sdf` (fixos + móveis) |
| 5. Nós de controle + arbitragem | `wheelchair_control` (teleop + `safety_arbiter`) |
| Interface alternativa (cabeça) | `head_teleop.py` |
| 6. Validação em simulação | dirigir entre os obstáculos móveis |
| 4. Nav2 (opcional, futuro) | publica na mesma `/cmd_vel`; base já emite `/scan`, `/odom`, TF |

---

## 10. Solução de problemas

- **`/joint_states` vazio:** confira o nome real do tópico com `gz topic -l`.
  Se o `JointStatePublisher` não respeitou `<topic>`, o padrão é
  `/world/clinic/model/wheelchair/joint_state` — ajuste o `gz_topic_name`
  correspondente em `config/bridge.yaml`.
- **Câmera não aparece:** verifique com `gz topic -l` o nome exato
  (`/camera`, `/camera_info`) e ajuste o `bridge.yaml`.
- **Robô "afunda"/treme ao iniciar:** dê o play na física do gz e confira
  que ele nasceu acima do chão (o spawn usa `-z 0.12`).
- **Nada anda:** confirme a cadeia `ros2 topic echo /cmd_vel` enquanto
  teleopera; e que o `parameter_bridge` está de pé (ROS→gz do `/cmd_vel`).
- **Robô não se move no RViz (TF odom→base ausente):** confira com
  `gz topic -l` o tópico de TF do DiffDrive. Se não for `/tf` e sim
  `/model/wheelchair/tf`, ajuste o `gz_topic_name` da entrada `/tf` no
  `config/bridge.yaml`.
- **Ver todos os tópicos gz de uma vez:** `gz topic -l` — é a fonte da
  verdade para casar os nomes no `bridge.yaml`.
