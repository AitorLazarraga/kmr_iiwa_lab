"""Bloqueo de control: solo una conexión puede mandar movimiento a la vez.

Reglas:
  - Hace falta rol operador o superior.
  - Si nadie tiene el control, se concede.
  - Si lo tiene alguien de rango IGUAL o MENOR, se le quita (a igual rango,
    el último en pedirlo gana: LIFO).
  - Si lo tiene alguien de rango MAYOR, se deniega.
  - Si quien lo tiene se desconecta, se le reserva durante `grace` segundos:
    si vuelve con la misma sesión lo recupera; si no, queda libre.

Todas las llamadas se hacen desde el bucle de asyncio (un solo hilo).
"""
import time
from dataclasses import dataclass, field

from .auth import ROLES


@dataclass
class Client:
    id: int
    user_id: int
    username: str
    role: str
    session: str            # hash del token de sesión
    ip: str
    connected_at: float = field(default_factory=time.time)

    @property
    def rank(self):
        return ROLES[self.role]

    @property
    def who(self):
        return f'{self.username}@{self.ip}'


class ControlDenied(Exception):
    pass


class ControlManager:

    def __init__(self, grace, on_change):
        """on_change(new_holder: Client|None, old_holder: Client|None, reason: str)."""
        self.grace = grace
        self.on_change = on_change
        self.holder = None              # Client
        self.holder_connected = False
        self.disconnected_at = 0.0

    # -------------------------------------------------------------- consultas
    def has_control(self, client):
        return (self.holder is not None and self.holder_connected
                and self.holder.id == client.id)

    def snapshot(self, client):
        h = self.holder
        if h is None:
            return {'holder': None, 'you': False}
        return {'holder': {'username': h.username, 'role': h.role,
                           'connected': self.holder_connected},
                'you': self.has_control(client),
                'can_take': client.rank >= ROLES['operator'] and client.rank >= h.rank}

    # -------------------------------------------------------------- acciones
    def request(self, client):
        if client.rank < ROLES['operator']:
            raise ControlDenied('tu rol no permite controlar el robot')
        old = self.holder
        if old is not None and old.id == client.id and self.holder_connected:
            return  # ya lo tiene
        if old is not None and old.rank > client.rank:
            raise ControlDenied(f'el control lo tiene {old.username} ({old.role}), de rango superior')
        self.holder = client
        self.holder_connected = True
        reason = 'tomado' if old is None else 'arrebatado'
        self.on_change(client, old, reason)

    def release(self, client):
        if self.holder is not None and self.holder.id == client.id:
            self.holder = None
            self.on_change(None, client, 'liberado')

    def client_disconnected(self, client):
        if self.holder is not None and self.holder.id == client.id:
            self.holder_connected = False
            self.disconnected_at = time.monotonic()
            self.on_change(self.holder, None, 'desconectado')

    def client_connected(self, client):
        """Una reconexión con la misma sesión recupera el control reservado."""
        h = self.holder
        if h is not None and not self.holder_connected and h.session == client.session:
            self.holder = client
            self.holder_connected = True
            self.on_change(client, None, 'recuperado')

    def session_closed(self, session):
        """Logout o sesión invalidada: libera el control si era de esa sesión."""
        if self.holder is not None and self.holder.session == session:
            old = self.holder
            self.holder = None
            self.on_change(None, old, 'liberado')

    def tick(self):
        """Libera el control reservado si ha pasado el tiempo de gracia."""
        if (self.holder is not None and not self.holder_connected
                and time.monotonic() - self.disconnected_at > self.grace):
            old = self.holder
            self.holder = None
            self.on_change(None, old, 'caducado')
