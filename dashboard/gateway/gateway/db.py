"""Base de datos SQLite del gateway: usuarios, sesiones, puntos, rutas y
posturas del brazo.

Una única conexión protegida con un lock: las operaciones son pequeñas y
poco frecuentes, así que no hace falta más. El esquema lleva versión.
  v1: usuarios y sesiones
  v2: puntos y rutas (2.4)
  v3: posturas del brazo (4.5)
  v4: tipo de punto (movimiento / trabajo) y datos de trabajo (6.3)
"""
import json
import os
import sqlite3
import threading
import time
from contextlib import contextmanager

SCHEMA_VERSION = 4

SCHEMA = """

CREATE TABLE IF NOT EXISTS users (
    id            INTEGER PRIMARY KEY AUTOINCREMENT,
    username      TEXT    NOT NULL UNIQUE COLLATE NOCASE,
    password_hash TEXT    NOT NULL,
    role          TEXT    NOT NULL CHECK (role IN ('observer', 'operator', 'admin')),
    active        INTEGER NOT NULL DEFAULT 1,
    created_at    REAL    NOT NULL,
    last_login    REAL
);

CREATE TABLE IF NOT EXISTS points (
    id INTEGER NOT NULL PRIMARY KEY AUTOINCREMENT,
    name TEXT NOT NULL,
    map_name TEXT NOT NULL,
    x REAL NOT NULL,
    y REAL NOT NULL,
    yaw REAL NOT NULL,
    created_by INTEGER REFERENCES users(id) ON DELETE SET NULL,
    created_at REAL NOT NULL,
    updated_at REAL,
    -- v4: 'move' = punto de movimiento; 'work' = punto de trabajo frente a una pared.
    -- En los de trabajo, x/y/yaw guardan la pose de llegada YA CALCULADA y
    -- work es un JSON {wall, distance, grid} (NULL en los de movimiento).
    kind TEXT NOT NULL DEFAULT 'move' CHECK (kind IN ('move', 'work')),
    work TEXT,
    UNIQUE ( map_name, name)
);

CREATE TABLE IF NOT EXISTS routes (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    name TEXT NOT NULL,
    map_name TEXT NOT NULL,
    created_by INTEGER REFERENCES users(id) ON DELETE SET NULL,
    created_at REAL NOT NULL,
    updated_at REAL,
    UNIQUE (name, map_name)
);

CREATE TABLE IF NOT EXISTS sessions (
    token_hash TEXT    PRIMARY KEY,
    user_id    INTEGER NOT NULL REFERENCES users(id) ON DELETE CASCADE,
    created_at REAL    NOT NULL,
    expires_at REAL    NOT NULL,
    ip         TEXT
);
CREATE TABLE IF NOT EXISTS meta (
    key   TEXT PRIMARY KEY,
    value TEXT
);

-- Posturas del brazo: 7 ángulos (rad) en el orden A1..A7, guardados como JSON.
-- No dependen del mapa: son relativas al propio robot.
CREATE TABLE IF NOT EXISTS arm_poses (
    id         INTEGER PRIMARY KEY AUTOINCREMENT,
    name       TEXT    NOT NULL UNIQUE COLLATE NOCASE,
    positions  TEXT    NOT NULL,
    created_by INTEGER REFERENCES users(id) ON DELETE SET NULL,
    created_at REAL    NOT NULL,
    updated_at REAL
);

CREATE TABLE IF NOT EXISTS route_points (
    route_id INTEGER NOT NULL REFERENCES routes(id) ON DELETE CASCADE,
    position INTEGER NOT NULL,
    point_id INTEGER NOT NULL REFERENCES points(id) ON DELETE CASCADE,
    wait_s REAL NOT NULL DEFAULT 0,
    PRIMARY KEY (route_id, position)
);
"""

USER_FIELDS = 'id, username, role, active, created_at, last_login'


class Database:

    def __init__(self, path):
        os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
        self.path = path
        self._lock = threading.RLock()
        self._conn = sqlite3.connect(path, check_same_thread=False)
        self._conn.row_factory = sqlite3.Row
        with self._lock:
            self._conn.execute('PRAGMA foreign_keys = ON')
            self._conn.execute('PRAGMA journal_mode = WAL')
            self._conn.executescript(SCHEMA)
            self._migrate()
            self._conn.execute(
                "INSERT OR IGNORE INTO meta (key, value) VALUES ('schema_version', ?)",
                (str(SCHEMA_VERSION),))
            self._conn.execute("UPDATE meta SET value = ? WHERE key = 'schema_version'", (str(SCHEMA_VERSION),))
            self._conn.commit()

    def _migrate(self):
        """Añade a tablas existentes las columnas de versiones nuevas.
        Mira las columnas que hay realmente, así que se puede ejecutar siempre."""
        cols = {r['name'] for r in self._conn.execute('PRAGMA table_info(points)')}
        if 'kind' not in cols:
            self._conn.execute("ALTER TABLE points ADD COLUMN kind TEXT NOT NULL DEFAULT 'move' "
                               "CHECK (kind IN ('move', 'work'))")
        if 'work' not in cols:
            self._conn.execute('ALTER TABLE points ADD COLUMN work TEXT')

    def _exec(self, sql, params=(), *, fetch=None):
        with self._lock:
            cur = self._conn.execute(sql, params)
            self._conn.commit()
            if fetch == 'one':
                row = cur.fetchone()
                return dict(row) if row else None
            if fetch == 'all':
                return [dict(r) for r in cur.fetchall()]
            return cur

    @contextmanager
    def _transaction(self):
        """Varias sentencias atómicas: commit al final o rollback si hay excepción."""
        with self._lock:
            with self._conn:          # la conexión como contexto hace commit/rollback
                yield self._conn    

    # ------------------------------------------------------------- usuarios
    def count_users(self):
        return self._exec('SELECT COUNT(*) AS n FROM users', fetch='one')['n']

    def create_user(self, username, password_hash, role):
        cur = self._exec(
            'INSERT INTO users (username, password_hash, role, created_at) VALUES (?, ?, ?, ?)',
            (username, password_hash, role, time.time()))
        return cur.lastrowid

    def get_user(self, user_id):
        return self._exec(f'SELECT {USER_FIELDS} FROM users WHERE id = ?',
                          (user_id,), fetch='one')

    def get_user_auth(self, username):
        """Incluye el hash de la contraseña: solo para el login."""
        return self._exec('SELECT * FROM users WHERE username = ?',
                          (username,), fetch='one')

    def list_users(self):
        return self._exec(f'SELECT {USER_FIELDS} FROM users ORDER BY username', fetch='all')

    def update_user(self, user_id, *, role=None, active=None, password_hash=None):
        sets, params = [], []
        if role is not None:
            sets.append('role = ?'); params.append(role)
        if active is not None:
            sets.append('active = ?'); params.append(1 if active else 0)
        if password_hash is not None:
            sets.append('password_hash = ?'); params.append(password_hash)
        if not sets:
            return
        params.append(user_id)
        self._exec(f'UPDATE users SET {", ".join(sets)} WHERE id = ?', params)
        if active is False or password_hash is not None:
            # Desactivar o cambiar la contraseña cierra todas sus sesiones
            self.delete_user_sessions(user_id)

    def delete_user(self, user_id):
        self._exec('DELETE FROM users WHERE id = ?', (user_id,))

    def touch_login(self, user_id):
        self._exec('UPDATE users SET last_login = ? WHERE id = ?', (time.time(), user_id))

    # ------------------------------------------------------------- sesiones
    def create_session(self, token_hash, user_id, hours, ip):
        now = time.time()
        self._exec('INSERT INTO sessions (token_hash, user_id, created_at, expires_at, ip) '
                   'VALUES (?, ?, ?, ?, ?)', (token_hash, user_id, now, now + hours * 3600, ip))

    def get_session_user(self, token_hash):
        """Usuario de una sesión válida (no caducada, usuario activo) o None."""
        return self._exec(
            f'SELECT {", ".join("u." + f for f in USER_FIELDS.split(", "))} '
            'FROM sessions s JOIN users u ON u.id = s.user_id '
            'WHERE s.token_hash = ? AND s.expires_at > ? AND u.active = 1',
            (token_hash, time.time()), fetch='one')

    def delete_session(self, token_hash):
        self._exec('DELETE FROM sessions WHERE token_hash = ?', (token_hash,))

    def delete_user_sessions(self, user_id):
        self._exec('DELETE FROM sessions WHERE user_id = ?', (user_id,))

    def purge_expired_sessions(self):
        self._exec('DELETE FROM sessions WHERE expires_at <= ?', (time.time(),))

    # ------------------------------------------------------------- Puntos

    @staticmethod
    def _decode_point(row):
        if row is not None and row.get('work') is not None:
            row['work'] = json.loads(row['work'])
        return row

    def list_points(self, map_name):
        rows = self._exec('SELECT * FROM points WHERE map_name = ? ORDER BY name', (map_name,), fetch='all')
        return [self._decode_point(r) for r in rows]

    def get_point(self, id):
        return self._decode_point(
            self._exec('SELECT * FROM points WHERE id = ?', (id,), fetch='one'))

    def create_point(self, name, map_name, x, y, yaw, crby, kind='move', work=None):
        if kind == 'work' and not work:
            raise ValueError('un punto de trabajo necesita sus datos de trabajo')
        if kind == 'move' and work:
            raise ValueError('un punto de movimiento no lleva datos de trabajo')
        cur = self._exec(
            'INSERT INTO points (name, map_name, x, y, yaw, created_by, created_at, kind, work) '
            'VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)',
            (name, map_name, x, y, yaw, crby, time.time(), kind,
             json.dumps(work) if work else None))
        return cur.lastrowid

    def delete_point(self, id):
        self._exec('DELETE FROM points WHERE id = ?', (id,))

    def update_point(self, point_id, *, name=None, x=None, y=None, yaw=None, work=None):
        """El tipo (kind) no se cambia: para convertir un punto, borrar y crear."""
        sets, params = [], []
        if work is not None:
            work = json.dumps(work)
        for column, value in (('name', name), ('x', x), ('y', y), ('yaw', yaw), ('work', work)):
            if value is not None:
                sets.append(f'{column} = ?')
                params.append(value)
        if not sets:
            return
        sets.append('updated_at = ?')
        params.extend([time.time(), point_id])
        self._exec(f'UPDATE points SET {", ".join(sets)} WHERE id = ?', params)

    # ------------------------------------------------------------- Rutas

    def list_routes(self, map_name):
        return self._query_routes('r.map_name = ?', (map_name,))

    def get_route(self, route_id):
        found = self._query_routes('r.id = ?', (route_id,))
        return found[0] if found else None

    def create_route(self, name, map_name, stops, created_by):
        with self._transaction() as c:
            cur = c.execute('INSERT INTO routes (name, map_name, created_by, created_at) VALUES (?, ?, ?, ?)', (name,map_name,created_by,time.time()))
            route_id = cur.lastrowid
            filas = [(route_id, position, point_id, wait_s) for position, (point_id, wait_s) in enumerate(stops)]
            c.executemany(
                'INSERT INTO route_points (route_id, position, point_id, wait_s) VALUES (?, ?, ?, ?)',
                filas)
        return route_id

    def update_route(self, route_id, name=None, stops=None):
        with self._transaction() as c:
            if stops is not None:
                c.execute('DELETE FROM route_points WHERE route_id = ?', (route_id,))
                filas = [(route_id, position, point_id, wait_s) for position, (point_id, wait_s) in enumerate(stops)]
                c.executemany(
                    'INSERT INTO route_points (route_id, position, point_id, wait_s) '
                    'VALUES (?, ?, ?, ?)', filas)
            if name is not None:
                c.execute('UPDATE routes SET name = ?, updated_at = ? WHERE id = ?',
                          (name, time.time(), route_id))
            else:
                c.execute('UPDATE routes SET updated_at = ? WHERE id = ?',
                          (time.time(), route_id)) 

    def delete_route(self, route_id):
        self._exec('DELETE FROM routes WHERE id = ?', (route_id,))

    def routes_using_point(self, point_id):
        return self._exec(
            'SELECT DISTINCT r.id, r.name '
            'FROM routes r '
            'JOIN route_points rp ON rp.route_id = r.id '
            'WHERE rp.point_id = ? '
            'ORDER BY r.name',
            (point_id,), fetch='all')

    _ROUTES_SQL = '''
        SELECT r.id   AS route_id,  r.name AS route_name, r.map_name,
               r.created_by, r.created_at, r.updated_at,
               rp.position, rp.wait_s,
               p.id   AS point_id,  p.name AS point_name,  p.x, p.y, p.yaw, p.kind
        FROM routes r
        LEFT JOIN route_points rp ON rp.route_id = r.id
        LEFT JOIN points p        ON p.id = rp.point_id
        WHERE {where}
        ORDER BY r.name, rp.position
    '''

    def _query_routes(self, where, params):
        rows = self._exec(self._ROUTES_SQL.format(where=where), params, fetch='all')
        routes = {}                                   # route_id -> ruta
        for row in rows:
            rid = row['route_id']
            if rid not in routes:                     # primera fila de esta ruta
                routes[rid] = {
                    'id': rid,
                    'name': row['route_name'],
                    'map_name': row['map_name'],
                    'created_by': row['created_by'],
                    'created_at': row['created_at'],
                    'updated_at': row['updated_at'],
                    'stops': [],
                }
            if row['point_id'] is not None:           # None = ruta sin paradas
                routes[rid]['stops'].append({
                    'point_id': row['point_id'],
                    'name': row['point_name'],
                    'x': row['x'],
                    'y': row['y'],
                    'yaw': row['yaw'],
                    'kind': row['kind'],
                    'wait_s': row['wait_s'],
                })
        result = list(routes.values())
        for r in result:
            r['executable'] = len(r['stops']) > 0
        return result

    # ------------------------------------------------------------- posturas del brazo
    @staticmethod
    def _decode_pose(row):
        if row is not None:
            row['positions'] = json.loads(row['positions'])
        return row

    def list_arm_poses(self):
        rows = self._exec('SELECT * FROM arm_poses ORDER BY name', fetch='all')
        return [self._decode_pose(r) for r in rows]

    def get_arm_pose(self, pose_id):
        return self._decode_pose(
            self._exec('SELECT * FROM arm_poses WHERE id = ?', (pose_id,), fetch='one'))

    def create_arm_pose(self, name, positions, created_by):
        cur = self._exec(
            'INSERT INTO arm_poses (name, positions, created_by, created_at) VALUES (?, ?, ?, ?)',
            (name, json.dumps([float(p) for p in positions]), created_by, time.time()))
        return cur.lastrowid

    def update_arm_pose(self, pose_id, *, name=None, positions=None):
        sets, params = [], []
        if name is not None:
            sets.append('name = ?'); params.append(name)
        if positions is not None:
            sets.append('positions = ?'); params.append(json.dumps([float(p) for p in positions]))
        if not sets:
            return
        sets.append('updated_at = ?')
        params.extend([time.time(), pose_id])
        self._exec(f'UPDATE arm_poses SET {", ".join(sets)} WHERE id = ?', params)

    def delete_arm_pose(self, pose_id):
        self._exec('DELETE FROM arm_poses WHERE id = ?', (pose_id,))
