#!/usr/bin/env python3
"""Cliente de prueba del gateway (pasos 2.1 y 2.3).

Secuencia:
  1. Login por HTTP y conexión al WebSocket con la cookie de sesión.
  2. Teleop SIN control -> debe rechazarse.
  3. Toma el control (requiere rol operador o admin).
  4. Teleop: avanza a 0.2 m/s durante 2 s; luego deja de enviar -> watchdog.
  5. Orden fuera de límites y marcha atrás -> deben rechazarse.
  6. (opcional) --goal X Y : manda un goal y espera a que termine.
  7. Libera el control.

Uso:
  python tools/test_client.py --user aitor
  python tools/test_client.py --user aitor --goal 3.0 2.0
"""
import argparse
import asyncio
import getpass
import json
import time
import urllib.error
import urllib.request

import websockets

TERMINAL = ('succeeded', 'aborted', 'canceled', 'rejected')


def login(base, user, password):
    req = urllib.request.Request(
        f'{base}/api/login', method='POST',
        data=json.dumps({'username': user, 'password': password}).encode(),
        headers={'Content-Type': 'application/json'})
    try:
        with urllib.request.urlopen(req) as r:
            cookie = r.headers.get('set-cookie').split(';', 1)[0]
            print(f'Login OK: {json.loads(r.read())}')
            return cookie
    except urllib.error.HTTPError as e:
        raise SystemExit(f'Login fallido ({e.code}): {e.read().decode()}')


async def connect(url, cookie):
    headers = {'Cookie': cookie}
    try:   # websockets >= 14
        return await websockets.connect(url, additional_headers=headers)
    except TypeError:   # versiones antiguas
        return await websockets.connect(url, extra_headers=headers)


async def receiver(ws, st):
    async for raw in ws:
        msg = json.loads(raw)
        kind = msg.get('type')
        if kind == 'state':
            st['nav'] = msg['nav'].get('state')
            st['control'] = msg['control']
            if st['verbose']:
                p = msg['pose']
                pose = f"x={p['x']:.2f} y={p['y']:.2f} yaw={p['yaw']:.2f}" if p else 'sin pose'
                c = msg['control']
                holder = c['holder']['username'] if c['holder'] else '—'
                print(f"[state] {pose} | ros={msg['ros_ok']} nav={st['nav']} "
                      f"{msg['nav'].get('distance_remaining', '')} | control={holder} (yo={c['you']})")
                st['verbose'] = False
        elif kind == 'config':
            print(f"[config] usuario={msg['user']} límites={msg['limits']}")
        elif kind in ('event', 'notice'):
            print(f"[{kind}] {msg['level']}: {msg['message']}")
        elif kind == 'error':
            print(f"[ERROR] {msg['message']}")


async def main(args):
    password = args.password or getpass.getpass(f'Contraseña de {args.user}: ')
    cookie = login(args.base, args.user, password)
    ws_url = args.base.replace('http', 'ws', 1) + '/ws'
    st = {'nav': None, 'control': None, 'verbose': True}
    ws = await connect(ws_url, cookie)
    rx = asyncio.create_task(receiver(ws, st))
    send = lambda m: ws.send(json.dumps(m))

    await asyncio.sleep(1)

    print('\n== 2. Teleop sin control: debe rechazarse ==')
    await send({'type': 'teleop', 'vx': 0.1})
    await asyncio.sleep(0.5)

    print('\n== 3. Tomar el control ==')
    await send({'type': 'take_control'})
    await asyncio.sleep(0.5)

    print('\n== 4. Teleop 2 s a 0.2 m/s y luego silencio (watchdog) ==')
    t0 = time.monotonic()
    while time.monotonic() - t0 < 2.0:
        await send({'type': 'teleop', 'vx': 0.2, 'vy': 0.0, 'wz': 0.0})
        await asyncio.sleep(0.1)
    await asyncio.sleep(1.0)

    print('\n== 5. Fuera de límites y marcha atrás: deben rechazarse ==')
    await send({'type': 'teleop', 'vx': 5.0})
    await send({'type': 'teleop', 'vx': -0.1})
    await asyncio.sleep(0.5)

    if args.goal:
        print(f'\n== 6. Goal a x={args.goal[0]} y={args.goal[1]} (espera hasta terminar) ==')
        await send({'type': 'goal', 'x': args.goal[0], 'y': args.goal[1], 'yaw': 0.0})
        t0 = time.monotonic()
        await asyncio.sleep(1.0)
        while st['nav'] not in TERMINAL and time.monotonic() - t0 < args.timeout:
            await asyncio.sleep(0.5)
        print(f'   resultado: {st["nav"]} en {time.monotonic() - t0:.1f} s')

    print('\n== 7. Liberar el control ==')
    await send({'type': 'release_control'})
    await asyncio.sleep(0.5)
    rx.cancel()
    await ws.close()


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--base', default='http://127.0.0.1:8000')
    ap.add_argument('--user', required=True)
    ap.add_argument('--password', help='si se omite, se pide por teclado')
    ap.add_argument('--goal', nargs=2, type=float, metavar=('X', 'Y'))
    ap.add_argument('--timeout', type=float, default=180.0, help='espera máxima del goal (s)')
    asyncio.run(main(ap.parse_args()))
