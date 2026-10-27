#!/usr/bin/env python3
"""Genera un mundo de Gazebo (.world) a partir de una descripción YAML del laboratorio.

Toma un mundo plantilla (con los plugins de física, sensores, etc. que ya funcionan)
e inserta los modelos estáticos descritos en el YAML justo antes de </world>.

Uso:
    python3 generar_mundo.py laboratorio.yaml plantilla.world salida.world

Además de paredes, cajas, mesas y cilindros admite paneles de atornillado
(`screw_panels`): un tabique con una rejilla de huecos marcados en la cara de
trabajo. Las posiciones de los huecos en coordenadas del mundo se guardan en
<salida>_huecos.yaml (verdad de referencia para las Fases 6 y 7).
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
    'panel': '0.86 0.83 0.76 1',     # tablero claro, contrasta con las marcas
    'hole': '0.08 0.08 0.08 1',      # hueco: disco oscuro
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


def visual(nombre, geom, tipo, pose):
    """Solo visual (sin colisión): las marcas no deben alterar el lidar ni la física."""
    return f'        <visual name="{nombre}_vis"><pose>{pose}</pose>{geom}{material(tipo)}</visual>\n'


def panel_atornillado(p, huecos_out):
    """Tabique con una rejilla de huecos en la cara de trabajo.

    center: centro de la base del panel [x, y]; yaw: grados.
    Con yaw = 0 el panel va a lo largo de X y la cara de trabajo mira a +Y
    (en general, la normal de la cara es (-sin(yaw), cos(yaw))).
    holes: cols x rows, spacing [horizontal, vertical], bottom = altura de la
    fila inferior (m), diameter (m). La rejilla se centra en horizontal.
    """
    ancho, grosor, alto = p['size']
    x, y = p['center']
    yaw = math.radians(p.get('yaw', 0))
    h = p['holes']
    cols, rows = h['cols'], h['rows']
    sh, sv = h['spacing']
    diam = h.get('diameter', 0.04)
    z0 = h['bottom']

    cuerpo = pieza('panel', geom_box(ancho, grosor, alto), 'panel', f'0 0 {alto / 2:.3f} 0 0 0')
    disco = geom_cyl(diam / 2, 0.004)
    nx, ny = -math.sin(yaw), math.cos(yaw)          # normal de la cara de trabajo
    lista = []
    for r in range(rows):
        for c in range(cols):
            u = (c - (cols - 1) / 2) * sh            # posición a lo largo del panel
            z = z0 + r * sv
            if abs(u) > ancho / 2 - diam or z > alto - diam:
                raise ValueError(f"Panel '{p['name']}': el hueco ({c},{r}) se sale del panel")
            nombre = f'hueco_f{r}_c{c}'
            # cilindro con su eje a lo largo de +Y local (roll -90°), asomando 2 mm
            cuerpo += visual(nombre, disco, 'hole',
                             f'{u:.3f} {grosor / 2 + 0.002:.3f} {z:.3f} -1.5708 0 0')
            # centro del hueco sobre la cara, en coordenadas del mundo
            wx = x + u * math.cos(yaw) + (grosor / 2) * nx
            wy = y + u * math.sin(yaw) + (grosor / 2) * ny
            lista.append({'name': nombre, 'xyz': [round(wx, 4) + 0.0, round(wy, 4) + 0.0, round(z, 4)]})
    huecos_out.append({
        'panel': p['name'],
        'normal': [round(nx, 4) + 0.0, round(ny, 4) + 0.0, 0.0],   # +0.0 evita '-0.0'   # sale de la cara de trabajo
        'huecos': lista,
    })
    return modelo(p['name'], x, y, 0.0, yaw, cuerpo)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('descripcion', help='YAML con las medidas del laboratorio')
    ap.add_argument('plantilla', help='.world base con los plugins (p. ej. kmr_iiwa.world)')
    ap.add_argument('salida', help='.world generado')
    args = ap.parse_args()

    with open(args.descripcion) as f:
        d = yaml.safe_load(f)

    huecos = []
    generadores = [('walls', pared), ('boxes', caja), ('tables', mesa), ('cylinders', cilindro),
                   ('screw_panels', lambda p: panel_atornillado(p, huecos))]
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
        print(f'  {clave:12s} {n}')

    if huecos:
        ruta = args.salida.rsplit('.', 1)[0] + '_huecos.yaml'
        with open(ruta, 'w') as f:
            f.write('# Generado por generar_mundo.py: posición de cada hueco sobre la cara\n'
                    '# de trabajo del panel (frame del mundo de Gazebo, m) y normal de la cara.\n')
            yaml.safe_dump({'paneles': huecos}, f, sort_keys=False, allow_unicode=True,
                           default_flow_style=None)
        print(f'\nHuecos de atornillado: {ruta}')
        for pnl in huecos:
            print(f"  {pnl['panel']}: {len(pnl['huecos'])} huecos, cara hacia {pnl['normal'][:2]}")

    r = d.get('robot_inicio')
    if r:
        print('\nLanza la simulación con el robot en su posición inicial:')
        print(f"  ros2 launch kuka_lab gazebo.launch.py world:=<ruta>/{args.salida.split('/')[-1]} "
              f"x:={r['x']} y:={r['y']} Y:={math.radians(r.get('yaw', 0)):.4f}")


if __name__ == '__main__':
    main()
