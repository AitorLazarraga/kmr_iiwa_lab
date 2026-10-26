#!/usr/bin/env python3
"""Prueba del jog articular: movimiento, parada al soltar, watchdog y rechazos."""
import argparse, asyncio, getpass, json, math, sys, time
sys.path.insert(0, 'tools')
from test_client import connect, login

DEG = math.radians


async def main(a):
    cookie = login(a.base, a.user, a.password or getpass.getpass())
    ws = await connect(a.base.replace('http', 'ws', 1) + '/ws', cookie)
    state = {}

    async def rx():
        async for raw in ws:
            m = json.loads(raw)
            if m['type'] == 'state':
                state.update(m['arm'] or {})
            elif m['type'] in ('error', 'event', 'notice'):
                print(f"  [{m['type']}] {m.get('message')}")
    task = asyncio.create_task(rx())
    send = lambda m: ws.send(json.dumps(m))
    a1 = lambda: math.degrees(state['positions'][0]) if state.get('positions') else float('nan')

    async def jog(v_deg, seconds):
        v = [DEG(x) for x in v_deg]
        t0 = time.monotonic()
        while time.monotonic() - t0 < seconds:
            await send({'type': 'arm_jog', 'v': v})
            await asyncio.sleep(0.1)

    await send({'type': 'take_control'}); await asyncio.sleep(1)
    print(f'A1 inicial: {a1():.1f}°  modo={state.get("mode")}')

    print('\n== 1. A1 a +10°/s durante 3 s, luego soltar (v=0) ==')
    await jog([10, 0, 0, 0, 0, 0, 0], 3)
    await send({'type': 'arm_jog', 'v': [0] * 7}); await asyncio.sleep(1)
    print(f'A1: {a1():.1f}°  (debe haber subido ~30°)')

    print('\n== 2. A1 a -10°/s 2 s y silencio (watchdog) ==')
    await jog([-10, 0, 0, 0, 0, 0, 0], 2)
    before = a1(); await asyncio.sleep(1.5)
    print(f'A1: {before:.1f}° -> {a1():.1f}°  (debe estar quieto tras el watchdog)')

    print('\n== 3. Rechazos ==')
    await send({'type': 'arm_jog', 'v': [DEG(40), 0, 0, 0, 0, 0, 0]})   # > 15°/s
    await send({'type': 'arm_jog', 'v': [0, 0, 0]})                      # 3 valores
    await asyncio.sleep(1)

    print('\n== 4. PARADA a mitad de jog ==')
    asyncio.create_task(jog([10, 0, 0, 0, 0, 0, 0], 3))
    await asyncio.sleep(1); await send({'type': 'stop'}); await asyncio.sleep(2.5)
    print(f'A1 final: {a1():.1f}°')

    task.cancel(); await ws.close()

ap = argparse.ArgumentParser()
ap.add_argument('--base', default='http://127.0.0.1:8000')
ap.add_argument('--user', required=True)
ap.add_argument('--password')
asyncio.run(main(ap.parse_args()))