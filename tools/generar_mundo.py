#!/usr/bin/env python3
"""Genera un mundo de Gazebo (.world) a partir de una descripción YAML del laboratorio.

Toma un mundo plantilla (con los plugins de física, sensores, etc. que ya funcionan)
e inserta los modelos estáticos descritos en el YAML justo antes de </world>.

Uso:
    python3 generar_mundo.py laboratorio.yaml plantilla.world salida.world
"""
import argparse
import math
import sys

import yaml

COLORES = {
    'wall': '0.75 0.75 0.75 1',
    'box': '0.55 0.45 0.35 1',
    'table': '0.60 0.50 0.40 1',
    'cylinder': '0.40 0.40 0.45 1',
}


def material(tipo):
    c = COLORES[tipo]
    return f'<material><ambient>{c}</ambient><diffuse>{c}</diffuse></material>'


def geom_box(sx, sy, sz):
    return f'<geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>'


def geom_cyl(r, h):
    return (f'<geometry><cylinder><radius>{r:.3f}</radius>'
            f'<length>{h:.3f}</length></cylinder></geometry>')


def modelo(nombre, x, y, z, yaw, cuerpo):
    return (
        f'    <model name="{nombre}">\n'
        f'      <static>true</static>\n'
        f'      <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 {yaw:.4f}</pose>\n'
        f'      <link name="link">\n'
        f'{cuerpo}'
        f'      </link>\n'
        f'    </model>\n'
    )


def pieza(nombre, geom, tipo, pose='0 0 0 0 0 0'):
    return (
        f'        <collision name="{nombre}_col"><pose>{pose}</pose>{geom}</collision>\n'
        f'        <visual name="{nombre}_vis"><pose>{pose}</pose>{geom}{material(tipo)}</visual>\n'
    )


def pared(p):
    (x1, y1), (x2, y2) = p['from'], p['to']
    largo = math.hypot(x2 - x1, y2 - y1)
    if largo < 1e-3:
        raise ValueError(f"Pared '{p['name']}' con longitud cero")
    t, h = p['thickness'], p['height']
    yaw = math.atan2(y2 - y1, x2 - x1)
    # +t en el largo para que las esquinas queden cerradas
    cuerpo = pieza('pared', geom_box(largo + t, t, h), 'wall')
    return modelo(p['name'], (x1 + x2) / 2, (y1 + y2) / 2, h / 2, yaw, cuerpo)


def caja(b):
    sx, sy, sz = b['size']
    x, y = b['center']
    cuerpo = pieza('caja', geom_box(sx, sy, sz), 'box')
    return modelo(b['name'], x, y, sz / 2, math.radians(b.get('yaw', 0)), cuerpo)


def mesa(m):
    largo, ancho, alto = m['size']
    x, y = m['center']
    pata = m.get('leg', 0.05)
    grosor = 0.03
    cuerpo = pieza('tablero', geom_box(largo, ancho, grosor), 'table',
                   f'0 0 {alto - grosor / 2:.3f} 0 0 0')
    alto_pata = alto - grosor
    dx, dy = largo / 2 - pata / 2, ancho / 2 - pata / 2
    for i, (sx, sy) in enumerate([(1, 1), (1, -1), (-1, 1), (-1, -1)]):
        cuerpo += pieza(f'pata{i}', geom_box(pata, pata, alto_pata), 'table',
                        f'{sx * dx:.3f} {sy * dy:.3f} {alto_pata / 2:.3f} 0 0 0')
    return modelo(m['name'], x, y, 0.0, math.radians(m.get('yaw', 0)), cuerpo)


def cilindro(c):
    x, y = c['center']
    cuerpo = pieza('cilindro', geom_cyl(c['radius'], c['height']), 'cylinder')
    return modelo(c['name'], x, y, c['height'] / 2, 0.0, cuerpo)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('descripcion', help='YAML con las medidas del laboratorio')
    ap.add_argument('plantilla', help='.world base con los plugins (p. ej. kmr_iiwa.world)')
    ap.add_argument('salida', help='.world generado')
    args = ap.parse_args()

    with open(args.descripcion) as f:
        d = yaml.safe_load(f)

    generadores = [('walls', pared), ('boxes', caja), ('tables', mesa), ('cylinders', cilindro)]
    nombres, bloques, cuenta = set(), [], {}
    for clave, fn in generadores:
        items = d.get(clave) or []
        cuenta[clave] = len(items)
        for it in items:
            if it['name'] in nombres:
                sys.exit(f"ERROR: nombre repetido '{it['name']}' (deben ser únicos)")
            nombres.add(it['name'])
            bloques.append(fn(it))

    with open(args.plantilla) as f:
        base = f.read()
    pos = base.rfind('</world>')
    if pos == -1:
        sys.exit('ERROR: la plantilla no contiene </world>')

    info = d.get('info', {})
    cabecera = (f'\n    <!-- ===== Generado por generar_mundo.py a partir de '
                f'{args.descripcion} ===== -->\n'
                f'    <!-- Origen: {info.get("origen", "sin describir")} -->\n')
    salida = base[:pos] + cabecera + ''.join(bloques) + '  ' + base[pos:]

    with open(args.salida, 'w') as f:
        f.write(salida)

    print(f'Mundo generado: {args.salida}')
    for clave, n in cuenta.items():
        print(f'  {clave:10s} {n}')

    r = d.get('robot_inicio')
    if r:
        print('\nLanza la simulación con el robot en su posición inicial:')
        print(f"  ros2 launch kuka_lab gazebo.launch.py world:=<ruta>/{args.salida.split('/')[-1]} "
              f"x:={r['x']} y:={r['y']} Y:={math.radians(r.get('yaw', 0)):.4f}")


if __name__ == '__main__':
    main()
