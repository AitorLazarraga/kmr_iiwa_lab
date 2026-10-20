#!/usr/bin/env python3
"""Cliente de prueba del gateway (paso 2.1).

Secuencia:
  1. Escucha 2 s y muestra la pose y lo que llega.
  2. Teleop: avanza a 0.2 m/s durante 2 s (órdenes a 10 Hz).
  3. Deja de enviar sin mandar parada -> el watchdog debe parar el robot.
  4. Envía una orden fuera de límites -> debe llegar un error 'rejected'.
  5. (opcional) --goal X Y : manda un goal de Nav2 y sigue su estado.

Uso:
  python tools/test_client.py
  python tools/test_client.py --goal 3.0 2.0
"""
import argparse
import asyncio
import json
import time

import websockets


async def receiver(ws, quiet):
    async for raw in ws:
        msg = json.loads(raw)
        kind = msg.get('type')
        if kind == 'state':
            if not quiet['state']:
                p = msg['pose']
                pose = f"x={p['x']:.2f} y={p['y']:.2f} yaw={p['yaw']:.2f}" if p else 'sin pose'
                print(f"[state] {pose} | nav={msg['nav'].get('state')} "
                      f"{msg['nav'].get('distance_remaining', '')} | "
                      f"teleop={msg['teleop_active']} | nav2={msg['nav2_ready']}")
        elif kind == 'map':
            print(f"[map]   {msg['width']}x{msg['height']} @ {msg['resolution']} m")
        elif kind == 'scan':
            valid = sum(r is not None for r in msg['ranges'])
            if not quiet['scan']:
                print(f"[scan]  {len(msg['ranges'])} haces, {valid} válidos")
                quiet['scan'] = True   # solo una vez
        elif kind == 'error':
            print(f"[ERROR] {msg['code']}: {msg['message']}")


async def main(url, goal):
    quiet = {'state': False, 'scan': False}
    async with websockets.connect(url) as ws:
        rx = asyncio.create_task(receiver(ws, quiet))

        print('\n== 1. Escuchando 2 s ==')
        await asyncio.sleep(2)

        print('\n== 2. Teleop: avanzar a 0.2 m/s durante 2 s ==')
        t0 = time.monotonic()
        while time.monotonic() - t0 < 2.0:
            await ws.send(json.dumps({'type': 'teleop', 'vx': 0.2, 'vy': 0.0, 'wz': 0.0}))
            await asyncio.sleep(0.1)

        print('\n== 3. Dejo de enviar (sin parada): el watchdog debe parar el robot ==')
        await asyncio.sleep(1.5)

        print('\n== 4. Orden fuera de límites (vx=5): debe rechazarse ==')
        await ws.send(json.dumps({'type': 'teleop', 'vx': 5.0}))
        await asyncio.sleep(0.5)

        if goal:
            print(f'\n== 5. Goal a x={goal[0]} y={goal[1]} ==')
            await ws.send(json.dumps({'type': 'goal', 'x': goal[0], 'y': goal[1], 'yaw': 0.0}))
            await asyncio.sleep(20)
            await ws.send(json.dumps({'type': 'cancel'}))

        await asyncio.sleep(0.5)
        rx.cancel()


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--url', default='ws://127.0.0.1:8000/ws')
    ap.add_argument('--goal', nargs=2, type=float, metavar=('X', 'Y'))
    args = ap.parse_args()
    asyncio.run(main(args.url, args.goal))
