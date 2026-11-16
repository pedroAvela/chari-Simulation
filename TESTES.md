# Protocolo de Testes — Ablação COM vs SEM assistência

Objetivo: gerar evidência quantitativa de que o controle compartilhado
(`safety_arbiter`) reduz colisões/quase-colisões **sem** tirar o controle
contínuo do usuário. É o resultado central para o relatório/artigo.

## Ideia do experimento

O mesmo percurso é dirigido em duas condições:

- **COM assistência:** teleop → `/cmd_vel_user` → `safety_arbiter` → `/cmd_vel`
- **SEM assistência:** teleop → `/cmd_vel` (direto, arbitro fora)

Em ambas, o `metrics_logger` grava folga ao obstáculo, trajetória, colisões,
quase-colisões e o quanto o arbitro interveio. Depois o `analyze_runs` compara.

Sugestão de percurso fixo (para as duas condições serem comparáveis):
partir do spawn `(-4.5, -1.5)`, atravessar a **porta** (1,0 m) e contornar
os pilares/mesas do outro lado até o canto oposto. Faça o mesmo caminho nas
duas condições. Se possível, repita 3× cada condição.

---

## Condição 1 — COM assistência

Terminal A (simulação, já sobe o arbitro):
```bash
ros2 launch wheelchair_gazebo simulation.launch.py
```
Terminal B (teleop na ENTRADA DO USUÁRIO):
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r /cmd_vel:=/cmd_vel_user
```
Terminal C (métricas):
```bash
ros2 run wheelchair_control metrics_logger --ros-args \
  -p label:=com_assist -p use_sim_time:=true
```
Dirija o percurso. Ao terminar, **CTRL-C no Terminal C** (grava o resumo).

---

## Condição 2 — SEM assistência

Terminal A (simulação SEM o arbitro):
```bash
ros2 launch wheelchair_gazebo simulation.launch.py use_safety:=false
```
Terminal B (teleop DIRETO no robô):
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
Terminal C (métricas):
```bash
ros2 run wheelchair_control metrics_logger --ros-args \
  -p label:=sem_assist -p use_sim_time:=true
```
Dirija o MESMO percurso. CTRL-C no Terminal C ao terminar.

---

## Gerar as figuras

```bash
pip install matplotlib      # uma vez
ros2 run wheelchair_control analyze_runs metrics_com_assist.csv metrics_sem_assist.csv
```
Sai a tabela no terminal e três PNGs: folga×tempo, trajetória e barras.

---

## O que reportar

- **Folga mínima** e **nº de colisões/quase-colisões**: devem ser melhores COM
  assistência — é a tese do trabalho.
- **Trajetória**: mostra o desvio suave contornando os obstáculos.
- **Intervenção média / % do tempo intervindo**: mostra que o arbitro age
  pouco (só quando necessário), preservando o controle do usuário.

Observação honesta: a "colisão" aqui é um **proxy via LiDAR** (folga abaixo de
um limiar), não contato físico medido. Para número exato de contatos daria para
adicionar um sensor de bumper depois; para a demonstração da tese o proxy de
folga é suficiente e transparente.
