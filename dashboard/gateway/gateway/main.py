"""Parte web del gateway: FastAPI + WebSocket + web estática.

Protocolo (JSON por WebSocket en /ws):

  Gateway -> navegador
    {"type": "config", "limits": {vx_min, vx_max, vy_max, wz_max}, ...}  al conectar
    {"type": "state", "ros_ok", "pose": {x,y,yaw}|null, "nav": {state, goal, ...},
     "route_running", "teleop_active", "nav2_ready"}                       a STATE_RATE
    {"type": "map", "width", "height", "resolution", "origin": {x,y,yaw}, "data": base64(int8)}
    {"type": "scan", "frame", "angle_min", "angle_increment", "ranges": [m|null, ...]}
    {"type": "event", "id", "level": info|warning|emergency, "code", "message", "time"}
    {"type": "error", "code", "message"}        solo al cliente que envió una orden inválida

  Navegador -> gateway
    {"type": "teleop", "vx": m/s, "vy": m/s, "wz": rad/s}   a 10 Hz mientras se mueve
    {"type": "stop"}                                         parada: cancela ruta y frena
    {"type": "goal", "x": m, "y": m, "yaw": rad}             en el frame del mapa
    {"type": "cancel"}                                       cancela la ruta o borra el goal

Ejecución:
    python -m gateway.main
"""
import asyncio
import json
import logging
import os
from contextlib import asynccontextmanager

import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles

from . import config as cfg
from .ros_bridge import CommandRejected, start_ros, stop_ros

log = logging.getLogger('gateway.web')
bridge = None


def setup_logging():
    os.makedirs(os.path.dirname(cfg.LOG_FILE) or '.', exist_ok=True)
    fmt = logging.Formatter('%(asctime)s %(levelname)-7s %(name)s: %(message)s')
    root = logging.getLogger('gateway')
    root.setLevel(logging.INFO)
    for handler in (logging.FileHandler(cfg.LOG_FILE), logging.StreamHandler()):
        handler.setFormatter(fmt)
        root.addHandler(handler)


@asynccontextmanager
async def lifespan(app):
    global bridge
    bridge, executor, _ = start_ros(cfg)
    try:
        yield
    finally:
        stop_ros(bridge, executor)


app = FastAPI(title='KMR iiwa gateway', lifespan=lifespan)


@app.get('/api/health')
async def health():
    return {'ok': True, 'nav2_ready': bridge.nav_client.server_is_ready()}


async def send_loop(ws: WebSocket):
    """Envía estado periódico, scan cada N envíos, el mapa cuando cambia y
    los eventos nuevos."""
    period = 1.0 / cfg.STATE_RATE
    sent_map_version = -1
    last_event = bridge.last_event_id()   # solo eventos posteriores a conectarse
    tick = 0
    await ws.send_json(bridge.get_config())
    while True:
        await ws.send_json(bridge.get_state())

        for event in bridge.get_events_since(last_event):
            await ws.send_json(event)
            last_event = event['id']

        version, map_msg = bridge.get_map()
        if map_msg is not None and version != sent_map_version:
            await ws.send_json(map_msg)
            sent_map_version = version

        if tick % max(1, cfg.SCAN_EVERY) == 0:
            scan = bridge.get_scan()
            if scan is not None:
                await ws.send_json(scan)

        tick += 1
        await asyncio.sleep(period)


def _num(msg, key, default=None):
    if key not in msg:
        if default is None:
            raise CommandRejected(f"falta el campo '{key}'")
        return default
    try:
        return float(msg[key])
    except (TypeError, ValueError):
        raise CommandRejected(f"'{key}' debe ser un número")


def handle_command(msg, who):
    kind = msg.get('type')
    if kind == 'teleop':
        bridge.teleop(_num(msg, 'vx', 0.0), _num(msg, 'vy', 0.0),
                      _num(msg, 'wz', 0.0), who=who)
    elif kind == 'stop':
        bridge.stop(who=who)
    elif kind == 'goal':
        bridge.send_goal(_num(msg, 'x'), _num(msg, 'y'), _num(msg, 'yaw', 0.0), who=who)
    elif kind == 'cancel':
        bridge.cancel_goal(who=who)
    else:
        raise CommandRejected(f'tipo de mensaje desconocido: {kind!r}')


@app.websocket('/ws')
async def websocket_endpoint(ws: WebSocket):
    await ws.accept()
    who = f'{ws.client.host}:{ws.client.port}'
    log.info('Cliente conectado: %s', who)
    sender = asyncio.create_task(send_loop(ws))
    try:
        while True:
            raw = await ws.receive_text()
            try:
                msg = json.loads(raw)
                if not isinstance(msg, dict):
                    raise CommandRejected('el mensaje debe ser un objeto JSON')
                handle_command(msg, who)
            except CommandRejected as e:
                await ws.send_json({'type': 'error', 'code': 'rejected', 'message': str(e)})
            except json.JSONDecodeError:
                await ws.send_json({'type': 'error', 'code': 'bad_json',
                                    'message': 'JSON no válido'})
    except WebSocketDisconnect:
        pass
    finally:
        sender.cancel()
        # No hace falta parar aquí: si este cliente estaba teleoperando, el
        # watchdog del nodo ROS para el robot al dejar de recibir órdenes.
        log.info('Cliente desconectado: %s', who)


# La web estática se monta al final para que /ws y /api tengan prioridad.
if os.path.isdir(cfg.WEB_DIR):
    app.mount('/', StaticFiles(directory=cfg.WEB_DIR, html=True), name='web')


def main():
    setup_logging()
    if os.path.isdir(cfg.WEB_DIR):
        log.info('Sirviendo la web desde %s', cfg.WEB_DIR)
    else:
        log.warning('No existe WEB_DIR=%s: solo WebSocket y API', cfg.WEB_DIR)
    log.info('Gateway escuchando en http://%s:%d', cfg.HOST, cfg.PORT)
    uvicorn.run(app, host=cfg.HOST, port=cfg.PORT, log_level='warning')


if __name__ == '__main__':
    main()
