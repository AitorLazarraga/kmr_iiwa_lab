"""Gestión de usuarios desde la terminal.

Uso (desde dashboard/gateway, con el venv activado):
    python -m gateway.users add <usuario> --role admin|operator|observer
    python -m gateway.users list
    python -m gateway.users passwd <usuario>
    python -m gateway.users role <usuario> <rol>
    python -m gateway.users disable <usuario>
    python -m gateway.users enable <usuario>
    python -m gateway.users delete <usuario>

Las contraseñas se piden por teclado (no quedan en el historial de la shell).
Los cambios de rol, contraseña o estado afectan a un gateway en marcha la
próxima vez que revalide la sesión (unos segundos) o al reconectar.
"""
import argparse
import getpass
import sys
from datetime import datetime

from . import auth
from . import config as cfg
from .db import Database


def ask_password():
    while True:
        p1 = getpass.getpass('Contraseña: ')
        try:
            auth.validate_password(p1)
        except ValueError as e:
            print(f'  {e}')
            continue
        if getpass.getpass('Repite la contraseña: ') != p1:
            print('  no coinciden')
            continue
        return p1


def find(db, username):
    user = db.get_user_auth(username)
    if user is None:
        sys.exit(f'No existe el usuario {username!r}')
    return user


def fmt_time(t):
    return datetime.fromtimestamp(t).strftime('%Y-%m-%d %H:%M') if t else '—'


def main():
    ap = argparse.ArgumentParser(prog='python -m gateway.users', description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    p = sub.add_parser('add'); p.add_argument('username')
    p.add_argument('--role', default='observer', choices=list(auth.ROLES))
    sub.add_parser('list')
    p = sub.add_parser('passwd'); p.add_argument('username')
    p = sub.add_parser('role'); p.add_argument('username'); p.add_argument('role', choices=list(auth.ROLES))
    for name in ('disable', 'enable', 'delete'):
        p = sub.add_parser(name); p.add_argument('username')
    args = ap.parse_args()

    db = Database(cfg.DB_PATH)

    if args.cmd == 'add':
        if db.get_user_auth(args.username):
            sys.exit(f'El usuario {args.username!r} ya existe')
        pw = ask_password()
        db.create_user(args.username, auth.hash_password(pw), args.role)
        print(f'Usuario {args.username!r} creado con rol {args.role}')

    elif args.cmd == 'list':
        users = db.list_users()
        if not users:
            print('No hay usuarios.')
        print(f'{"usuario":20} {"rol":10} {"activo":7} {"último login":16}')
        for u in users:
            print(f'{u["username"]:20} {u["role"]:10} {"sí" if u["active"] else "no":7} '
                  f'{fmt_time(u["last_login"]):16}')

    elif args.cmd == 'passwd':
        u = find(db, args.username)
        db.update_user(u['id'], password_hash=auth.hash_password(ask_password()))
        print('Contraseña cambiada (sus sesiones abiertas se han cerrado)')

    elif args.cmd == 'role':
        u = find(db, args.username)
        db.update_user(u['id'], role=args.role)
        print(f'{args.username}: rol {u["role"]} -> {args.role}')

    elif args.cmd in ('disable', 'enable'):
        u = find(db, args.username)
        db.update_user(u['id'], active=(args.cmd == 'enable'))
        print(f'{args.username}: {"activado" if args.cmd == "enable" else "desactivado"}')

    elif args.cmd == 'delete':
        u = find(db, args.username)
        if input(f'¿Borrar {args.username!r}? escribe "si": ').strip().lower() != 'si':
            sys.exit('Cancelado')
        db.delete_user(u['id'])
        print('Borrado')


if __name__ == '__main__':
    main()
