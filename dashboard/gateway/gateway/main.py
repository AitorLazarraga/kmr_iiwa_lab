"""Parte web del gateway: FastAPI + WebSocket + web estática.

API HTTP
  POST   /api/login             {username, password} -> cookie de sesión
  POST   /api/logout
  GET    /api/me
  GET    /api/users             (admin)
  POST   /api/users             (admin) {username, password, role}
  PATCH  /api/users/{id}        (admin) {role?, active?, password?}
  DELETE /api/users/{id}        (admin)
  GET    /api/health

  Puntos y rutas (del mapa activo cfg.MAP_NAME salvo ?map=<nombre> en los GET)
  GET    /api/points            ?map=
  POST   /api/points            (operador) {name, x, y, yaw}
  PATCH  /api/points/{id}       (operador) {name?, x?, y?, yaw?}
  DELETE /api/points/{id}       (operador)  -> quita el punto de las rutas que lo usan
  GET    /api/routes            ?map=
  GET    /api/routes/{id}
  POST   /api/routes            (operador) {name, stops: [{point_id, wait_s}, ...]}
  PATCH  /api/routes/{id}       (operador) {name?, stops?}  (stops sustituye todas)
  DELETE /api/routes/{id}       (operador)
  Cualquier cambio emite un evento 'points_changed' para que los clientes recarguen.

WebSocket /ws (requiere sesión; se cierra con código 4401 si no la hay)

  Gateway -> navegador
    {"type": "config", "limits": {...}, "user": {username, role}}      al conectar
    {"type": "state", "ros_ok", "pose", "nav", "route_running", "teleop_active",
     "nav2_ready", "control": {holder: {username, role, connected}|null, you, can_take}}
    {"type": "map", ...}  {"type": "scan", ...}
    {"type": "event", "id", "level", "code", "message", "time"}        para todos
    {"type": "notice", "level", "message"}                             solo para este cliente
    {"type": "error", "code", "message"}                               orden rechazada

  Navegador -> gateway
    {"type": "take_control"} / {"type": "release_control"}             rol >= operador
    {"type": "teleop", "vx", "vy", "wz"}                               requiere control
    {"type": "goal", "x", "y", "yaw"}                                  requiere control
    {"type": "cancel"}                                                 requiere control
    {"type": "stop"}                                                   cualquier usuario

Ejecución:
    python -m gateway.main
"""
import asyncio
import itertools
import json
import logging
import math
import os
import sqlite3
from contextlib import asynccontextmanager
from urllib.parse import urlparse

import uvicorn
from fastapi import Depends, FastAPI, HTTPException, Request, Response, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field, field_validator

from . import auth
from . import config as cfg
from .control import Client, ControlDenied, ControlManager
from .db import Database
from .ros_bridge import CommandRejected, start_ros, stop_ros



log = logging.getLogger('gateway.web')

bridge = None
db = None
control = None
limiter = auth.LoginRateLimiter(cfg.LOGIN_MAX_FAILS, cfg.LOGIN_WINDOW)
clients = {}                       # client_id -> (Client, WebSocket)
_client_ids = itertools.count(1)

WS_CLOSE_AUTH = 4401

class PointIn(BaseModel):
    name: str = Field(min_length=1, max_length=40)
    x: float
    y: float
    yaw: float = 0.0

    @field_validator('name')
    @classmethod
    def clean_name(cls, v):
        v = v.strip()
        if not v:
            raise ValueError('el nombre no puede estar vacío')
        return v

    @field_validator('x', 'y', 'yaw')
    @classmethod
    def finite(cls, v):
        if not math.isfinite(v):
            raise ValueError('debe ser un número finito')
        return v

class PointPatch(BaseModel):
    name: str | None = Field(default=None, min_length=1, max_length=40)
    x: float | None = None
    y: float | None = None
    yaw: float | None = None

    @field_validator('name')
    @classmethod
    def clean_name(cls, v):
        if v is None:
            return v
        v = v.strip()
        if not v:
            raise ValueError("No se pueden ingresar nombres vacios")
        return v

    @field_validator('x', 'y', 'yaw')
    @classmethod
    def finite(cls,v):
        if v is None:
            return v
        if not math.isfinite(v):
            raise ValueError('Debe ser un numero finito')
        return v

class RouteStop(BaseModel):
    point_id: int
    # ge/le ya rechazan NaN e infinitos: no hace falta validador propio
    wait_s: float = Field(default=0.0, ge=0, le=600)

class RoutePatch(BaseModel):
    name: str | None = Field(default=None, min_length=1, max_length=40)
    stops: list[RouteStop] | None = Field(default=None, min_length=1, max_length=50)
    
    @field_validator('name')
    @classmethod
    def clean_name(cls,v):
        if v is None:
            return v
        v = v.strip()
        
        if not v:
            raise ValueError("No se pueden ingresar nombres vacios")
        return v

class RouteIn(BaseModel):
    name : str = Field(min_length=1, max_length=40)
    stops : list[RouteStop] = Field(min_length=1, max_length=50)

    @field_validator('name')
    @classmethod
    def clean_name(cls,v):
        v = v.strip()
        if not v:
            raise ValueError("No se pueden ingresar nombres vacios")
        return v

def setup_logging():
    os.makedirs(os.path.dirname(cfg.LOG_FILE) or '.', exist_ok=True)
    fmt = logging.Formatter('%(asctime)s %(levelname)-7s %(name)s: %(message)s')
    root = logging.getLogger('gateway')
    root.setLevel(logging.INFO)
    if not root.handlers:
        for handler in (logging.FileHandler(cfg.LOG_FILE), logging.StreamHandler()):
            handler.setFormatter(fmt)
            root.addHandler(handler)


# ---------------------------------------------------------------- control
def on_control_change(new, old, reason):
    """Callback del ControlManager: registra, avisa y para la teleop si cambia de manos."""
    if reason in ('tomado', 'arrebatado', 'liberado', 'caducado'):
        bridge.halt_teleop(who=f'cambio de control ({reason})')
    if reason == 'tomado':
        log.info('CONTROL tomado por %s', new.who)
        bridge.emit('info', 'control', f'{new.username} ha tomado el control')
    elif reason == 'arrebatado':
        log.warning('CONTROL arrebatado: %s -> %s', old.who, new.who)
        bridge.emit('warning', 'control', f'{new.username} ha tomado el control (antes {old.username})')
        notify(old.id, 'warning', f'{new.username} te ha quitado el control')
    elif reason == 'liberado':
        log.info('CONTROL liberado por %s', old.who)
        bridge.emit('info', 'control', f'{old.username} ha liberado el control')
    elif reason == 'desconectado':
        log.warning('CONTROL: %s desconectado, se reserva %.0f s', new.who, cfg.CONTROL_GRACE)
        bridge.emit('warning', 'control',
                    f'{new.username} (con el control) se ha desconectado')
    elif reason == 'recuperado':
        log.info('CONTROL recuperado por %s tras reconectar', new.who)
        bridge.emit('info', 'control', f'{new.username} ha recuperado el control')
    elif reason == 'caducado':
        log.warning('CONTROL liberado: %s no ha vuelto en %.0f s', old.who, cfg.CONTROL_GRACE)
        bridge.emit('warning', 'control',
                    f'Control libre: {old.username} no se ha reconectado')


def notify(client_id, level, message):
    """Mensaje directo a un cliente (sin esperar)."""
    entry = clients.get(client_id)
    if entry:
        asyncio.get_running_loop().create_task(
            _safe_send(entry[1], {'type': 'notice', 'level': level, 'message': message}))


async def _safe_send(ws, msg):
    try:
        await ws.send_json(msg)
    except Exception:  # noqa: BLE001 - el cliente puede haberse ido
        pass


async def control_ticker():
    while True:
        control.tick()
        await asyncio.sleep(0.5)


# ---------------------------------------------------------------- arranque
@asynccontextmanager
async def lifespan(app):
    global bridge, db, control
    db = Database(cfg.DB_PATH)
    db.purge_expired_sessions()
    if db.count_users() == 0:
        log.warning('No hay usuarios. Crea un administrador con: '
                    'python -m gateway.users add <usuario> --role admin')
    control = ControlManager(cfg.CONTROL_GRACE, on_control_change)
    bridge, executor, _ = start_ros(cfg)
    ticker = asyncio.create_task(control_ticker())
    try:
        yield
    finally:
        ticker.cancel()
        stop_ros(bridge, executor)


app = FastAPI(title='KMR iiwa gateway', lifespan=lifespan)


# ---------------------------------------------------------------- sesiones HTTP
def client_ip(conn):
    return conn.client.host if conn.client else '?'


def current_user(request: Request):
    token = request.cookies.get(auth.SESSION_COOKIE)
    user = db.get_session_user(auth.token_hash(token)) if token else None
    if user is None:
        raise HTTPException(401, 'sesión no válida o caducada')
    return user


def admin_user(user=Depends(current_user)):
    if user['role'] != 'admin':
        raise HTTPException(403, 'solo administradores')
    return user

def operator_user(user=Depends(current_user)):
    if auth.ROLES[user['role']] < auth.ROLES['operator']:
        raise HTTPException(403, 'se necesita rol de operador o superior')
    return user


def public_user(u):
    return {k: u[k] for k in ('id', 'username', 'role', 'active', 'created_at', 'last_login')
            if k in u}


class LoginIn(BaseModel):
    username: str
    password: str


class UserIn(BaseModel):
    username: str
    password: str
    role: str = 'observer'


class UserPatch(BaseModel):
    role: str | None = None
    active: bool | None = None
    password: str | None = None


@app.post('/api/login')
async def login(body: LoginIn, request: Request, response: Response):
    ip = client_ip(request)
    if limiter.blocked(ip):
        log.warning('LOGIN bloqueado por exceso de intentos: %s', ip)
        raise HTTPException(429, 'demasiados intentos; espera un minuto')
    user = auth.check_credentials(db, body.username.strip(), body.password)
    if user is None:
        limiter.fail(ip)
        log.warning('LOGIN fallido: usuario=%r ip=%s', body.username, ip)
        raise HTTPException(401, 'usuario o contraseña incorrectos')
    limiter.success(ip)
    token, token_hash = auth.new_session_token()
    db.create_session(token_hash, user['id'], cfg.SESSION_HOURS, ip)
    db.touch_login(user['id'])
    response.set_cookie(auth.SESSION_COOKIE, token, httponly=True, samesite='strict',
                        secure=cfg.COOKIE_SECURE, max_age=int(cfg.SESSION_HOURS * 3600))
    log.info('LOGIN %s (%s) desde %s', user['username'], user['role'], ip)
    return public_user(user)


@app.post('/api/logout')
async def logout(request: Request, response: Response):
    token = request.cookies.get(auth.SESSION_COOKIE)
    if token:
        th = auth.token_hash(token)
        db.delete_session(th)
        control.session_closed(th)
        await close_session_sockets(th)
    response.delete_cookie(auth.SESSION_COOKIE)
    return {'ok': True}


@app.get('/api/me')
async def me(user=Depends(current_user)):
    return public_user(user)


@app.get('/api/health')
async def health():
    return {'ok': True, 'nav2_ready': bridge.nav_client.server_is_ready()}


# ---------------------------------------------------------------- gestión de usuarios
@app.get('/api/users')
async def users_list(_=Depends(admin_user)):
    return [public_user(u) for u in db.list_users()]


@app.post('/api/users', status_code=201)
async def users_create(body: UserIn, admin=Depends(admin_user)):
    try:
        auth.validate_role(body.role)
        auth.validate_password(body.password)
        uid = db.create_user(body.username.strip(), auth.hash_password(body.password), body.role)
    except ValueError as e:
        raise HTTPException(400, str(e))
    except sqlite3.IntegrityError as e: 
        raise HTTPException(409, 'ese usuario ya existe') from e
    log.info('USUARIO creado por %s: %s (%s)', admin['username'], body.username, body.role)
    return public_user(db.get_user(uid))


@app.patch('/api/users/{user_id}')
async def users_update(user_id: int, body: UserPatch, admin=Depends(admin_user)):
    target = db.get_user(user_id)
    if target is None:
        raise HTTPException(404, 'usuario no encontrado')
    if user_id == admin['id'] and (body.active is False or (body.role and body.role != 'admin')):
        raise HTTPException(400, 'no puedes desactivarte ni quitarte el rol de administrador')
    try:
        if body.role is not None:
            auth.validate_role(body.role)
        pw_hash = None
        if body.password is not None:
            auth.validate_password(body.password)
            pw_hash = auth.hash_password(body.password)
    except ValueError as e:
        raise HTTPException(400, str(e))
    db.update_user(user_id, role=body.role, active=body.active, password_hash=pw_hash)
    if body.active is False or pw_hash is not None or body.role is not None:
        await close_user_sockets(user_id)   # que reconecte con los permisos nuevos
    log.info('USUARIO %s modificado por %s: %s', target['username'], admin['username'],
             body.model_dump(exclude_none=True, exclude={'password'}))
    return public_user(db.get_user(user_id))


@app.delete('/api/users/{user_id}')
async def users_delete(user_id: int, admin=Depends(admin_user)):
    if user_id == admin['id']:
        raise HTTPException(400, 'no puedes borrarte a ti mismo')
    target = db.get_user(user_id)
    if target is None:
        raise HTTPException(404, 'usuario no encontrado')
    await close_user_sockets(user_id)
    db.delete_user(user_id)
    log.info('USUARIO %s borrado por %s', target['username'], admin['username'])
    return {'ok': True}

# ---------------------------------------------------------------- puntos
@app.post('/api/points', status_code=201)
async def points_create(body: PointIn, user=Depends(operator_user)):
    try:
        pid = db.create_point(body.name, cfg.MAP_NAME, body.x, body.y, body.yaw, user['id'])
    except sqlite3.IntegrityError:
        raise HTTPException(409, f'ya existe un punto llamado {body.name!r} en este mapa')
    log.info('PUNTO creado por %s: %s (%.2f, %.2f)', user['username'], body.name, body.x, body.y)
    bridge.emit('info', 'points_changed', f'{user["username"]} ha creado el punto {body.name}')
    return db.get_point(pid)

@app.get('/api/points')
async def get_points(map:str | None = None, user=Depends(current_user)):
    return db.list_points(map or cfg.MAP_NAME)

@app.patch('/api/points/{point_id}')
async def change_points(point_id:int, body: PointPatch, user=Depends(operator_user)):
    target = db.get_point(point_id)
    if target is None:
        raise HTTPException(404, 'Point not Found')
    changes = body.model_dump(exclude_unset=True, exclude_none=True)
    if not changes:
        raise HTTPException(400, 'No datos de cambio')
    try:
        db.update_point(point_id, **changes)
    except sqlite3.IntegrityError:
        raise HTTPException(409, 'Ya existe un punto con este nombre')
    
    log.info('PUNTO %s modificado por %s: %s', target['name'], user['username'], changes)
    bridge.emit('info', 'points_changed',
                f'{user["username"]} ha modificado el punto {changes.get("name", target["name"])}')
    return db.get_point(point_id)

@app.delete('/api/points/{point_id}')
async def points_delete(point_id: int, user=Depends(operator_user)):
    target = db.get_point(point_id)
    if target is None:
        raise HTTPException(404, 'punto no encontrado')
    affected = db.routes_using_point(point_id)          # antes de borrar
    db.delete_point(point_id)
    modified = [r['name'] for r in affected]
    emptied = [r['name'] for r in affected if not db.get_route(r['id'])['executable']]

    message = f'{user["username"]} ha borrado el punto {target["name"]}'
    if modified:
        message += f'; rutas modificadas: {", ".join(modified)}'
    if emptied:
        message += f'. Han quedado VACÍAS: {", ".join(emptied)}'
    log.info('PUNTO borrado: %s', message)
    bridge.emit('warning' if emptied else 'info', 'points_changed', message)
    return {'ok': True, 'routes_modified': modified, 'routes_emptied': emptied}


# ---------------------------------------------------------------- rutas
def check_stops(stops):
    """Comprueba que todos los puntos existen y son del mapa activo.
    Devuelve la lista [(point_id, wait_s), ...] que espera la base de datos."""
    ids = {s.point_id for s in stops}
    valid = {p['id'] for p in db.list_points(cfg.MAP_NAME)}
    missing = ids - valid
    if missing:
        raise HTTPException(400, f'puntos inexistentes o de otro mapa: {sorted(missing)}')
    return [(s.point_id, s.wait_s) for s in stops]


def describe_route(route):
    n = len(route['stops'])
    return f'{route["name"]} ({n} parada{"" if n == 1 else "s"})'


@app.get('/api/routes')
async def routes_list(map: str | None = None, user=Depends(current_user)):
    return db.list_routes(map or cfg.MAP_NAME)


@app.get('/api/routes/{route_id}')
async def routes_get(route_id: int, user=Depends(current_user)):
    route = db.get_route(route_id)
    if route is None:
        raise HTTPException(404, 'ruta no encontrada')
    return route


@app.post('/api/routes', status_code=201)
async def routes_create(body: RouteIn, user=Depends(operator_user)):
    stops = check_stops(body.stops)
    try:
        rid = db.create_route(body.name, cfg.MAP_NAME, stops, user['id'])
    except sqlite3.IntegrityError:
        raise HTTPException(409, f'ya existe una ruta llamada {body.name!r} en este mapa')
    route = db.get_route(rid)
    log.info('RUTA creada por %s: %s', user['username'], describe_route(route))
    bridge.emit('info', 'points_changed',
                f'{user["username"]} ha creado la ruta {describe_route(route)}')
    return route


@app.patch('/api/routes/{route_id}')
async def routes_update(route_id: int, body: RoutePatch, user=Depends(operator_user)):
    target = db.get_route(route_id)
    if target is None:
        raise HTTPException(404, 'ruta no encontrada')
    if body.name is None and body.stops is None:
        raise HTTPException(400, 'no hay cambios')
    stops = check_stops(body.stops) if body.stops is not None else None
    try:
        db.update_route(route_id, name=body.name, stops=stops)
    except sqlite3.IntegrityError:
        raise HTTPException(409, f'ya existe una ruta llamada {body.name!r} en este mapa')
    route = db.get_route(route_id)

    changed = []
    if body.name is not None and body.name != target['name']:
        changed.append(f'nombre {target["name"]} -> {body.name}')
    if stops is not None:
        changed.append(f'paradas {len(target["stops"])} -> {len(route["stops"])}')
    log.info('RUTA %s modificada por %s: %s', target['name'], user['username'],
             ', '.join(changed) or 'sin cambios efectivos')
    bridge.emit('info', 'points_changed',
                f'{user["username"]} ha modificado la ruta {route["name"]}')
    return route


@app.delete('/api/routes/{route_id}')
async def routes_delete(route_id: int, user=Depends(operator_user)):
    target = db.get_route(route_id)
    if target is None:
        raise HTTPException(404, 'ruta no encontrada')
    db.delete_route(route_id)
    log.info('RUTA borrada por %s: %s', user['username'], describe_route(target))
    bridge.emit('info', 'points_changed',
                f'{user["username"]} ha borrado la ruta {target["name"]}')
    return {'ok': True}


async def close_session_sockets(session):
    for client, ws in list(clients.values()):
        if client.session == session:
            await ws.close(code=WS_CLOSE_AUTH)


async def close_user_sockets(user_id):
    for client, ws in list(clients.values()):
        if client.user_id == user_id:
            control.session_closed(client.session)
            await ws.close(code=WS_CLOSE_AUTH)


# ---------------------------------------------------------------- WebSocket
def origin_ok(ws: WebSocket):
    """Los navegadores siempre mandan Origin: debe coincidir con el Host.
    Evita que otra web abierta en el mismo navegador use la sesión.
    Clientes que no son navegador (scripts) no mandan Origin y se aceptan."""
    origin = ws.headers.get('origin')
    if not origin:
        return True
    return urlparse(origin).netloc == ws.headers.get('host')


async def send_loop(ws: WebSocket, client: Client):
    """Estado periódico, scan, mapa cuando cambia y eventos nuevos."""
    period = 1.0 / cfg.STATE_RATE
    sent_map_version = -1
    last_event = bridge.last_event_id()
    tick = 0
    await ws.send_json({**bridge.get_config(),
                        'user': {'username': client.username, 'role': client.role}})
    while True:
        # Revalidar la sesión cada ~5 s (caducidad, usuario desactivado)
        if tick % max(1, int(5 * cfg.STATE_RATE)) == 0 and tick:
            if db.get_session_user(client.session) is None:
                control.session_closed(client.session)
                await ws.close(code=WS_CLOSE_AUTH)
                return

        state = bridge.get_state()
        state['control'] = control.snapshot(client)
        await ws.send_json(state)

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


NEEDS_CONTROL = ('teleop', 'goal', 'cancel')


def handle_command(msg, client: Client):
    kind = msg.get('type')
    who = client.who

    if kind == 'stop':                       # cualquier usuario puede parar
        bridge.stop(who=who)
        return
    if kind == 'take_control':
        try:
            control.request(client)
        except ControlDenied as e:
            log.warning('CONTROL denegado a %s: %s', who, e)
            raise CommandRejected(str(e))
        return
    if kind == 'release_control':
        control.release(client)
        return

    if kind in NEEDS_CONTROL and not control.has_control(client):
        log.warning('RECHAZADO %s de %s: no tiene el control', kind, who)
        raise CommandRejected('no tienes el control del robot')

    if kind == 'teleop':
        bridge.teleop(_num(msg, 'vx', 0.0), _num(msg, 'vy', 0.0), _num(msg, 'wz', 0.0), who=who)
    elif kind == 'goal':
        bridge.send_goal(_num(msg, 'x'), _num(msg, 'y'), _num(msg, 'yaw', 0.0), who=who)
    elif kind == 'cancel':
        bridge.cancel_goal(who=who)
    else:
        raise CommandRejected(f'tipo de mensaje desconocido: {kind!r}')

    


@app.websocket('/ws')
async def websocket_endpoint(ws: WebSocket):
    if not origin_ok(ws):
        log.warning('WS rechazado: origen %s no coincide con %s',
                    ws.headers.get('origin'), ws.headers.get('host'))
        await ws.close(code=1008)
        return
    token = ws.cookies.get(auth.SESSION_COOKIE)
    th = auth.token_hash(token) if token else None
    user = db.get_session_user(th) if th else None
    if user is None:
        await ws.accept()
        await ws.close(code=WS_CLOSE_AUTH)
        return

    await ws.accept()
    client = Client(id=next(_client_ids), user_id=user['id'], username=user['username'],
                    role=user['role'], session=th, ip=client_ip(ws))
    clients[client.id] = (client, ws)
    log.info('Cliente conectado: %s (%s)', client.who, client.role)
    control.client_connected(client)
    sender = asyncio.create_task(send_loop(ws, client))
    try:
        while True:
            raw = await ws.receive_text()
            try:
                msg = json.loads(raw)
                if not isinstance(msg, dict):
                    raise CommandRejected('el mensaje debe ser un objeto JSON')
                handle_command(msg, client)
            except CommandRejected as e:
                await ws.send_json({'type': 'error', 'code': 'rejected', 'message': str(e)})
            except json.JSONDecodeError:
                await ws.send_json({'type': 'error', 'code': 'bad_json',
                                    'message': 'JSON no válido'})
    except (WebSocketDisconnect, RuntimeError):
        pass
    finally:
        sender.cancel()
        clients.pop(client.id, None)
        control.client_disconnected(client)
        # Si este cliente teleoperaba, el watchdog del nodo ROS para el robot.
        log.info('Cliente desconectado: %s', client.who)


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