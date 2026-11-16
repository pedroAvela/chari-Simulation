#!/usr/bin/env python3
"""
analyze_runs.py
===============
Compara duas corridas (COM vs SEM assistencia) e gera as figuras do artigo.

Le os CSVs gerados pelo metrics_logger e produz:
  - comparativo_folga.png     : folga ao obstaculo ao longo do tempo
  - comparativo_trajetoria.png: trajetorias no plano X-Y
  - comparativo_barras.png    : metricas-resumo lado a lado
  - comparativo_normalizado.png: colisoes/quase-col. por metro e por minuto
  - tabela impressa no terminal (inclui metricas NORMALIZADAS)

Metricas normalizadas: como o percurso e dirigido a mao e as rotas variam de
tamanho, contagens absolutas (ex.: nro de quase-colisoes) sao injustas entre
rotas de comprimentos diferentes. Por isso reportamos tambem POR METRO e POR
MINUTO, alem do esforco de intervencao do arbitro e a folga min. FORA da porta.

Uso:
  ros2 run wheelchair_control analyze_runs metrics_com_assist.csv metrics_sem_assist.csv
  python3 analyze_runs.py metrics_com_assist.csv metrics_sem_assist.csv

Requer: matplotlib   (sudo apt install python3-matplotlib)
"""
import sys
import csv
import math

# Regiao da porta no mundo clinic.sdf: parede divisoria em x = -1.5,
# vao entre y = -0.5 e 0.5. Ignoramos a folga minima quando o robo esta
# atravessando a soleira (paredes proximas dos dois lados nao sao "risco").
DOOR_X = -1.5
DOOR_HALF_W = 0.9   # margem em x ao redor da soleira para excluir


def load(path):
    d = dict(t=[], x=[], y=[], mr=[], vu=[], wu=[], vo=[], wo=[], name=path)
    with open(path) as f:
        for row in csv.DictReader(f):
            d['t'].append(float(row['t']))
            d['x'].append(float(row['x']))
            d['y'].append(float(row['y']))
            r = float(row['min_range'])
            d['mr'].append(r if math.isfinite(r) else float('nan'))
            d['vu'].append(float(row['v_user']))
            d['wu'].append(float(row['w_user']))
            d['vo'].append(float(row['v_out']))
            d['wo'].append(float(row['w_out']))
    return d


def _count_events(vals, thr):
    """Conta transicoes para dentro do limiar (evita contar o mesmo evento 2x)."""
    n = 0
    below = False
    for v in vals:
        b = v < thr
        if b and not below:
            n += 1
        below = b
    return n


def summarize(d, collision=0.20, nearmiss=0.45):
    mr = [v for v in d['mr'] if math.isfinite(v)]

    # folga min ignorando a regiao da porta
    mr_no_door = []
    for i in range(len(d['mr'])):
        if not math.isfinite(d['mr'][i]):
            continue
        if abs(d['x'][i] - DOOR_X) <= DOOR_HALF_W:
            continue   # esta na soleira -> ignora
        mr_no_door.append(d['mr'][i])

    # distancia e duracao
    path_len = 0.0
    for i in range(1, len(d['x'])):
        path_len += math.hypot(d['x'][i] - d['x'][i-1], d['y'][i] - d['y'][i-1])
    dur = d['t'][-1] - d['t'][0] if len(d['t']) > 1 else 0.0

    col = _count_events(mr, collision)
    near = _count_events(mr, nearmiss)

    # esforco de intervencao: media de |cmd_out - cmd_user| (linear+angular)
    interv = []
    for i in range(len(d['vo'])):
        interv.append(abs(d['vo'][i] - d['vu'][i]) + abs(d['wo'][i] - d['wu'][i]))
    mean_interv = sum(interv) / len(interv) if interv else 0.0
    pct_interv = 100.0 * sum(1 for x in interv if x > 0.02) / len(interv) if interv else 0.0

    km = max(1e-6, path_len)
    minutes = max(1e-6, dur / 60.0)
    return dict(
        dur=dur, path=path_len,
        min_clear=min(mr) if mr else float('nan'),
        min_clear_no_door=min(mr_no_door) if mr_no_door else float('nan'),
        mean_clear=sum(mr)/len(mr) if mr else float('nan'),
        collisions=col, near_misses=near,
        col_per_m=col/km, near_per_m=near/km,
        col_per_min=col/minutes, near_per_min=near/minutes,
        mean_interv=mean_interv, pct_interv=pct_interv,
    )


def main():
    args = sys.argv[1:]
    if len(args) < 2:
        print('uso: analyze_runs <com_assist.csv> <sem_assist.csv>')
        return
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        print('ERRO: instale matplotlib -> sudo apt install python3-matplotlib')
        return

    a = load(args[0])   # com assistencia
    b = load(args[1])   # sem assistencia
    sa, sb = summarize(a), summarize(b)

    # ---------------- tabela ----------------
    print(f'\n{"metrica":<34}{"COM assist":>13}{"SEM assist":>13}')
    print('-' * 60)

    def row(lbl, ka, kb, fmt='{:.3f}'):
        print(f'{lbl:<34}{fmt.format(ka):>13}{fmt.format(kb):>13}')

    print('-- absolutas --')
    row('Colisoes', sa['collisions'], sb['collisions'], '{:d}')
    row('Quase-colisoes', sa['near_misses'], sb['near_misses'], '{:d}')
    row('Folga minima (m)', sa['min_clear'], sb['min_clear'])
    row('Folga min. FORA da porta (m)', sa['min_clear_no_door'], sb['min_clear_no_door'])
    row('Folga media (m)', sa['mean_clear'], sb['mean_clear'])
    row('Distancia (m)', sa['path'], sb['path'])
    row('Duracao (s)', sa['dur'], sb['dur'], '{:.1f}')
    print('-- normalizadas (comparacao justa) --')
    row('Colisoes / metro', sa['col_per_m'], sb['col_per_m'])
    row('Quase-col. / metro', sa['near_per_m'], sb['near_per_m'])
    row('Colisoes / minuto', sa['col_per_min'], sb['col_per_min'])
    row('Quase-col. / minuto', sa['near_per_min'], sb['near_per_min'])
    print('-- intervencao do arbitro --')
    row('Esforco medio de intervencao', sa['mean_interv'], sb['mean_interv'])
    row('Tempo intervindo (%)', sa['pct_interv'], sb['pct_interv'], '{:.1f}')
    print()

    # ---------------- 1) folga x tempo ----------------
    plt.figure(figsize=(9, 4))
    plt.plot(a['t'], a['mr'], label='COM assistencia', color='tab:green')
    plt.plot(b['t'], b['mr'], label='SEM assistencia', color='tab:red', alpha=0.8)
    plt.axhline(0.20, ls='--', color='k', lw=0.8, label='limiar colisao')
    plt.xlabel('tempo (s)'); plt.ylabel('folga ao obstaculo (m)')
    plt.title('Folga ao obstaculo mais proximo'); plt.legend(); plt.grid(alpha=0.3)
    plt.tight_layout(); plt.savefig('comparativo_folga.png', dpi=140)

    # ---------------- 2) trajetorias ----------------
    plt.figure(figsize=(6, 6))
    plt.plot(a['x'], a['y'], label='COM assistencia', color='tab:green')
    plt.plot(b['x'], b['y'], label='SEM assistencia', color='tab:red', alpha=0.8)
    plt.scatter([a['x'][0]], [a['y'][0]], c='blue', marker='o', label='inicio')
    plt.xlabel('x (m)'); plt.ylabel('y (m)'); plt.axis('equal')
    plt.title('Trajetoria'); plt.legend(); plt.grid(alpha=0.3)
    plt.tight_layout(); plt.savefig('comparativo_trajetoria.png', dpi=140)

    # ---------------- 3) barras absolutas ----------------
    plt.figure(figsize=(8, 4))
    labels = ['folga min (m)', 'folga min\nfora porta (m)', 'colisoes', 'quase-col.']
    va = [sa['min_clear'], sa['min_clear_no_door'], sa['collisions'], sa['near_misses']]
    vb = [sb['min_clear'], sb['min_clear_no_door'], sb['collisions'], sb['near_misses']]
    xpos = range(len(labels)); wbar = 0.35
    plt.bar([p - wbar/2 for p in xpos], va, wbar, label='COM', color='tab:green')
    plt.bar([p + wbar/2 for p in xpos], vb, wbar, label='SEM', color='tab:red')
    plt.xticks(list(xpos), labels); plt.title('Resumo comparativo (absoluto)')
    plt.legend(); plt.grid(alpha=0.3, axis='y')
    plt.tight_layout(); plt.savefig('comparativo_barras.png', dpi=140)

    # ---------------- 4) barras NORMALIZADAS ----------------
    plt.figure(figsize=(8, 4))
    labels = ['colisoes/m', 'quase-col./m', 'colisoes/min', 'quase-col./min']
    va = [sa['col_per_m'], sa['near_per_m'], sa['col_per_min'], sa['near_per_min']]
    vb = [sb['col_per_m'], sb['near_per_m'], sb['col_per_min'], sb['near_per_min']]
    xpos = range(len(labels))
    plt.bar([p - wbar/2 for p in xpos], va, wbar, label='COM', color='tab:green')
    plt.bar([p + wbar/2 for p in xpos], vb, wbar, label='SEM', color='tab:red')
    plt.xticks(list(xpos), labels)
    plt.title('Metricas normalizadas (por metro e por minuto)')
    plt.legend(); plt.grid(alpha=0.3, axis='y')
    plt.tight_layout(); plt.savefig('comparativo_normalizado.png', dpi=140)

    print('Figuras: comparativo_folga.png, comparativo_trajetoria.png, '
          'comparativo_barras.png, comparativo_normalizado.png')


if __name__ == '__main__':
    main()
