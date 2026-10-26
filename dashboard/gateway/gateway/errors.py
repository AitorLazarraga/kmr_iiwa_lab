"""Excepciones compartidas por los módulos del gateway."""


class CommandRejected(ValueError):
    """Orden rechazada (fuera de límites, sin control, servicio no disponible...)."""
