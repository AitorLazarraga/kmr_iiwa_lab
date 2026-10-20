import collections
import hashlib
import hmac
import secrets
import time

ROLES = {'observer': 0, 'operator': 1, 'admin': 2}
ROLE_NAMES = {'observer': 'observador', 'operator': 'operador', 'admin': 'administrador'}

SESSION_COOKIE = 'kmr_session'
MIN_PASSWORD_LEN = 4

# scrypt de la librería estándar (sin dependencias). ~16 MB de memoria por hash.
_SCRYPT_N, _SCRYPT_R, _SCRYPT_P = 2 ** 14, 8, 1


def hash_password(password):
    salt = secrets.token_bytes(16)
    digest = hashlib.scrypt(password.encode(), salt=salt,
                            n=_SCRYPT_N, r=_SCRYPT_R, p=_SCRYPT_P, dklen=32)
    return f'scrypt${_SCRYPT_N}${_SCRYPT_R}${_SCRYPT_P}${salt.hex()}${digest.hex()}'


def verify_password(password, stored):
    try:
        algo, n, r, p, salt_hex, digest_hex = stored.split('$')
        if algo != 'scrypt':
            return False
        digest = hashlib.scrypt(password.encode(), salt=bytes.fromhex(salt_hex),
                                n=int(n), r=int(r), p=int(p), dklen=32)
        return hmac.compare_digest(digest.hex(), digest_hex)
    except (ValueError, TypeError):
        return False


# Hash ficticio para que un usuario inexistente tarde lo mismo que uno real
# (no revelar qué usuarios existen por el tiempo de respuesta).
_DUMMY_HASH = hash_password(secrets.token_hex(8))


def check_credentials(db, username, password):
    """Devuelve el usuario si las credenciales son válidas y está activo."""
    user = db.get_user_auth(username or '')
    if user is None:
        verify_password(password or '', _DUMMY_HASH)
        return None
    if not verify_password(password or '', user['password_hash']) or not user['active']:
        return None
    return user


def validate_password(password):
    if not password or len(password) < MIN_PASSWORD_LEN:
        raise ValueError(f'la contraseña debe tener al menos {MIN_PASSWORD_LEN} caracteres')


def validate_role(role):
    if role not in ROLES:
        raise ValueError(f'rol no válido: {role!r} (opciones: {", ".join(ROLES)})')


def new_session_token():
    """Devuelve (token para la cookie, hash para la base de datos)."""
    token = secrets.token_urlsafe(32)
    return token, token_hash(token)


def token_hash(token):
    return hashlib.sha256(token.encode()).hexdigest()


class LoginRateLimiter:
    """Bloquea una IP tras demasiados intentos fallidos en una ventana de tiempo."""

    def __init__(self, max_fails, window):
        self.max_fails = max_fails
        self.window = window
        self._fails = collections.defaultdict(collections.deque)

    def _prune(self, ip):
        q = self._fails[ip]
        limit = time.monotonic() - self.window
        while q and q[0] < limit:
            q.popleft()
        return q

    def blocked(self, ip):
        return len(self._prune(ip)) >= self.max_fails

    def fail(self, ip):
        self._prune(ip).append(time.monotonic())

    def success(self, ip):
        self._fails.pop(ip, None)
