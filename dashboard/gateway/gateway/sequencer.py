import logging
import threading
import time

from rclpy.clock import Clock, ClockType

from . import work as workgeo
from .errors import CommandRejected

log = logging.getLogger('gateway.seq')

RUNNING = ('running', 'pausing', 'canceling')
PHASE_TEXT = {
    'reapproach': 'segundo ajuste',
    'position': 'colocando',
    'shift': 'desplazamiento lateral',
    'arm': 'brazo al tornillo',
    'verify': 'comprobando TCP',
    'advance': 'avance',
    'verify_adv': 'comprobando TCP (avanzado)',
    'screw': 'atornillando (simulado)',
    'retreat': 'retroceso',
    'verify_ret': 'comprobando TCP',
    'reposo': 'brazo a reposo',
    'cancel_retreat': 'cancelando: retroceso',
    'cancel_reposo': 'cancelando: brazo a reposo',
}


def distance_for(z, d0, bands, same_tol=0.05):
    """Distancia a la pared a la que trabajar un tornillo a la altura z.
    bands = [(distancia, z_min, z_max), ...]. Si la distancia actual d0 (la del
    punto de trabajo) vale para z, se queda; si no, la de la tabla más cercana a
    d0 que cubra z. None si ninguna llega."""
    ok = [d for d, lo, hi in bands if lo - 1e-9 <= z <= hi + 1e-9]
    if not ok:
        return None
    near = [d for d in ok if abs(d - d0) <= same_tol]
    if near:
        return d0
    return min(ok, key=lambda d: abs(d - d0))


def build_plan(screws, d0, bands, start_right=False):
    """Lista ordenada de pasos [{row, col, z, u, distance}] y grupos.
    Grupos por distancia (primero la actual, después por cercanía); dentro de
    cada grupo, filas de abajo arriba en serpiente."""
    rows = sorted({s['row'] for s in screws})
    by_row = {r: sorted((s for s in screws if s['row'] == r), key=lambda s: s['col']) for r in rows}
    row_dist = {}
    for r in rows:
        z = by_row[r][0]['z']
        d = distance_for(z, d0, bands)
        if d is None:
            raise CommandRejected(f'la fila F{r + 1} ({z:.2f} m) no se alcanza a ninguna distancia de '
                                  'REACH_BANDS: revisa la rejilla o la tabla')
        row_dist[r] = d
    dists = sorted(set(row_dist.values()), key=lambda d: (abs(d - d0) > 1e-9, abs(d - d0)))
    plan, right = [], start_right
    for d in dists:
        for r in [r for r in rows if row_dist[r] == d]:
            cols = by_row[r][::-1] if right else by_row[r]
            for s in cols:
                plan.append({'row': s['row'], 'col': s['col'], 'z': s['z'], 'u': s['u'], 'distance': d})
            right = not right
    groups = [{'distance': d, 'rows': [r for r in rows if row_dist[r] == d]} for d in dists]
    return plan, groups


class Sequencer:
    def __init__(self, bridge, cfg):
        self.b = bridge
        self.cfg = cfg
        self._lock = threading.Lock()
        self.state = 'idle'          # idle | running | pausing | paused | canceling | done | canceled
        self.point = None            # punto de trabajo (dict de la base de datos)
        self.plan = []
        self.groups = []
        self.index = 0
        self.status = {}             # (row, col) -> pending | doing | done
        self.times = {}              # (row, col) -> s
        self.phase = None
        self.error = None
        self.who = '?'
        self._phase_t = 0.0
        self._screw_t = 0.0
        self._started = 0.0
        self._ended = None
        self._quiet_since = None
        self._distance = None        # distancia de la pared a la que está ajustado ahora
        self._phase_started = False  # para fases que arrancan al quedar todo quieto
        self._lin_retry = False      # ya se ha recolocado con LIN este tornillo
        steady = Clock(clock_type=ClockType.STEADY_TIME)
        bridge.create_timer(1.0 / cfg.SEQ_RATE, self._tick, clock=steady)

    # ------------------------------------------------------------ API
    def busy(self):
        with self._lock:
            return self.state in RUNNING

    def start(self, point, who='?'):
        if self.busy():
            raise CommandRejected('ya hay una secuencia en marcha')
        if point.get('kind') != 'work' or not point.get('work'):
            raise CommandRejected(f'{point.get("name")!r} no es un punto de trabajo')
        w = self.b.work_here_raw()
        if w is None or w['point_id'] != point['id']:
            raise CommandRejected('el robot no está ajustado frente a ese punto: usa «Ir» o «Ajustar» primero')
        if self.b.nav_running() or self.b.arm.moving():
            raise CommandRejected('hay un movimiento en curso: espera a que termine')
        if self.b.safe_pose() is None:
            raise CommandRejected(f'no hay postura «{self.cfg.ARM_SAFE_POSE_NAME}»')
        if not self.b.arm.planner.ready():
            raise CommandRejected('MoveIt no está en marcha')
        screws = workgeo.screws(point['work']['wall'], point['work']['grid'])
        d0 = w.get('distance_goal', point['work']['distance'])
        plan, groups = build_plan(screws, d0, self.cfg.REACH_BANDS,
                                  start_right=w['est']['lateral'] > 0)
        with self._lock:
            self.point, self.plan, self.groups = point, plan, groups
            self.index = 0
            self.status = {(s['row'], s['col']): 'pending' for s in plan}
            self.times = {}
            self.error = None
            self.who = f'secuencia ({who})'
            self._started = time.monotonic()
            self._ended = None
            self._distance = d0
            self.state = 'running'
        log.info('SECUENCIA en %s iniciada por %s: %d tornillos, grupos %s', point['name'], who,
                 len(plan), [(g['distance'], [r + 1 for r in g['rows']]) for g in groups])
        self.b.emit('info', 'seq', f'Secuencia en {point["name"]}: {len(plan)} tornillos'
                    + (f' en {len(groups)} ajustes' if len(groups) > 1 else ''))
        self._next_screw()

    def pause(self, who='?'):
        with self._lock:
            if self.state != 'running':
                raise CommandRejected('no hay ninguna secuencia en marcha')
            self.state = 'pausing'
        log.info('SECUENCIA: pausa pedida por %s (al terminar el tornillo en curso)', who)
        self.b.emit('info', 'seq', 'Secuencia: se pausará al terminar el tornillo en curso')

    def resume(self, who='?'):
        with self._lock:
            if self.state == 'pausing':
                self.state = 'running'          # se arrepiente antes de terminar el tornillo
                return
            if self.state != 'paused':
                raise CommandRejected('la secuencia no está en pausa')
        w = self.b.work_here_raw()
        if w is None or w['point_id'] != self.point['id']:
            raise CommandRejected('el robot ya no está ajustado frente al punto: usa «Ajustar» y reanuda')
        if self.b.nav_running() or self.b.arm.moving():
            raise CommandRejected('hay un movimiento en curso: espera a que termine')
        with self._lock:
            self.state = 'running'
            self.error = None
            self._distance = w.get('distance_goal', self._distance)
        log.info('SECUENCIA reanudada por %s', who)
        self.b.emit('info', 'seq', 'Secuencia reanudada')
        self._next_screw()

    def cancel(self, who='?'):
        with self._lock:
            if self.state not in RUNNING + ('paused',):
                raise CommandRejected('no hay ninguna secuencia que cancelar')
            self.state = 'canceling'
        log.warning('SECUENCIA cancelada por %s', who)
        self.b.emit('warning', 'seq', f'Secuencia cancelada ({who}): retroceso si hace falta y reposo')
        self._halt_motion(who)
        # Esperar a que se pare lo que hubiera en marcha y después retroceso + reposo
        self._set_phase('cancel_retreat', started=False)

    def abort(self, who='?'):
        """PARADA: sin retroceso ni reposo."""
        with self._lock:
            if self.state not in RUNNING:
                return
            self.state = 'canceled'
            self.phase = None
            self._ended = time.monotonic()
            self.error = f'PARADA ({who})'
        log.warning('SECUENCIA abortada por PARADA (%s)', who)

    def get_state(self):
        with self._lock:
            if self.point is None:
                return {'state': 'idle'}
            cur = self.plan[self.index] if self.index < len(self.plan) else None
            end = self._ended or time.monotonic()
            return {
                'state': self.state,
                'point_id': self.point['id'], 'name': self.point['name'],
                'index': self.index, 'total': len(self.plan),
                'done': sum(1 for v in self.status.values() if v == 'done'),
                'current': ({'row': cur['row'], 'col': cur['col'], 'distance': cur['distance']}
                            if cur and self.state not in ('done', 'canceled') else None),
                'phase': self.phase, 'phase_text': PHASE_TEXT.get(self.phase, self.phase),
                'screws': [{'row': s['row'], 'col': s['col'], 'status': self.status[(s['row'], s['col'])],
                            'time': self.times.get((s['row'], s['col']))} for s in self.plan],
                'order': [[s['row'], s['col']] for s in self.plan],
                'groups': self.groups,
                'error': self.error,
                'elapsed': round(end - self._started, 1),
            }

    # ------------------------------------------------------------ fases
    def _set_phase(self, phase, started=True):
        with self._lock:
            self.phase = phase
            self._phase_t = time.monotonic()
            self._quiet_since = None
            self._phase_started = started

    def _cur(self):
        return self.plan[self.index]

    def _label(self, s=None):
        s = s or self._cur()
        return f'F{s["row"] + 1}·C{s["col"] + 1}'

    def _next_screw(self):
        """Empieza el tornillo self.index (o termina)."""
        with self._lock:
            state = self.state
        if state == 'pausing':
            self._to_paused(None)
            return
        if self.index >= len(self.plan):
            self._set_phase('reposo', started=False)
            return
        s = self._cur()
        with self._lock:
            self.status[(s['row'], s['col'])] = 'doing'
            self._screw_t = time.monotonic()
        if abs(s['distance'] - self._distance) > 1e-6:
            self._do(lambda: self.b.approach(self.point, who=self.who, distance=s['distance'],
                                             lateral=self.b.work_here_raw()['est']['lateral']),
                     'reapproach')
            return
        self._position()

    def _position(self):
        s = self._cur()
        tool = self.b.tool_state_now()
        if tool is not None and tool['screw']['row'] == s['row'] and tool['screw']['col'] != s['col']:
            self._do(lambda: self.b.shift_to_column(self.point['id'], s['col'], who=self.who), 'shift')
        else:
            self._arm_to_screw()

    def _arm_to_screw(self, after_shift=False):
        """Brazo al tornillo actual. Tras un desplazamiento la herramienta ya
        está delante y el check lo lanza el propio desplazamiento: solo hay que
        verificar. En cualquier otro caso (también al reanudar con la
        herramienta ya delante) se usa «ir a tornillo», que con la herramienta
        en un tornillo es un LIN corto y deja un check nuevo."""
        s = self._cur()
        tool = self.b.tool_state_now()
        with self._lock:
            self._lin_retry = False
        if after_shift and tool is not None and tool['screw'] == {'row': s['row'], 'col': s['col']} \
                and tool['state'] == 'standoff':
            self._set_phase('verify')
            return
        self._do(lambda: self.b.arm_to_screw(self.point['id'], s['row'], s['col'], who=self.who), 'arm')

    def _do(self, action, phase):
        """Lanza una acción del gateway y pasa a sondear la fase."""
        self._set_phase(phase)
        try:
            action()
        except CommandRejected as e:
            self._fail(str(e))

    # ------------------------------------------------------------ bucle
    def _tick(self):
        with self._lock:
            state, phase = self.state, self.phase
        if state not in RUNNING or phase is None:
            return
        try:
            getattr(self, f'_poll_{phase}')()
        except CommandRejected as e:
            self._fail(str(e))
        except Exception as e:  # noqa: BLE001 - que un fallo no deje la secuencia colgada
            log.exception('Secuencia: error inesperado en la fase %s', phase)
            self._fail(f'error interno en {phase}: {e}')

    def _elapsed(self):
        return time.monotonic() - self._phase_t

    def _quiet(self, secs=0.3):
        """True cuando ni la base ni el brazo (ni Servo) se mueven desde hace `secs`."""
        busy = self.b.nav_running() or self.b.arm.moving()
        now = time.monotonic()
        with self._lock:
            if busy:
                self._quiet_since = None
                return False
            if self._quiet_since is None:
                self._quiet_since = now
            return now - self._quiet_since >= secs

    def _poll_reapproach(self):
        if self._elapsed() < 0.5 or not self._quiet():
            return
        nav = self.b.nav_state()
        if nav.get('mode') == 'approach' and nav.get('state') == 'succeeded':
            with self._lock:
                self._distance = self._cur()['distance']
            self._position()
        else:
            self._fail(f'segundo ajuste a {self._cur()["distance"]:.2f} m fallido')

    def _poll_shift(self):
        if self._elapsed() < 0.5 or not self._quiet():
            return
        nav = self.b.nav_state()
        if nav.get('state') == 'succeeded':
            self._arm_to_screw(after_shift=True)
        else:
            msg = (nav.get('shift') or {}).get('message') or nav.get('state')
            self._fail(f'desplazamiento lateral fallido ({msg})')

    def _poll_arm(self):
        self._wait_arm(self._label(), 'verify')

    def _poll_advance(self):
        self._wait_arm(f'avance {self._label()}', 'verify_adv')

    def _poll_retreat(self):
        self._wait_arm(f'retroceso {self._label()}', 'verify_ret')

    def _wait_arm(self, expect, nxt):
        if self._elapsed() < 0.3 or not self._quiet():
            return
        move = self.b.arm.get_state().get('move') or {}
        if move.get('state') == 'succeeded' and expect in (move.get('label') or ''):
            self._set_phase(nxt)
        else:
            reason = move.get('reason') or move.get('state') or 'sin movimiento'
            self._fail(f'movimiento «{expect}» no completado ({reason})')

    def _verify(self, nxt, lin_retry=False):
        """Espera al check del TCP (y a la corrección automática si la hay).
        lin_retry: si falla (p. ej. más error del corregible con Servo tras un
        desplazamiento), una vez se recoloca con «ir a tornillo» (LIN corto)
        antes de dar el tornillo por fallido."""
        if self._elapsed() < self.cfg.TCP_CHECK_DELAY + 0.3 or not self._quiet(0.4):
            return
        move = self.b.arm.get_state().get('move') or {}
        check = (move.get('check') or {}).get('state')
        if check == 'checking':
            return
        if check == 'ok':
            nxt()
            return
        with self._lock:
            retry = lin_retry and not self._lin_retry
            self._lin_retry = True
        if retry:
            s = self._cur()
            log.info('SECUENCIA: check fuera en %s, recolocación con LIN', self._label())
            self._do(lambda: self.b.arm_to_screw(self.point['id'], s['row'], s['col'], who=self.who), 'arm')
            return
        sv = self.b.arm.servo.get_state()
        last = sv.get('last_correction') or {}
        why = last.get('text') if last and not last.get('ok') else 'TCP fuera de tolerancia'
        self._fail(f'check del TCP en {self._label()}: {why}')

    def _poll_verify(self):
        self._verify(lambda: self._do(
            lambda: self.b.arm_lin(self.point['id'], 'advance', who=self.who), 'advance'), lin_retry=True)

    def _poll_verify_adv(self):
        self._verify(lambda: self._set_phase('screw'))

    def _poll_screw(self):
        # PENDIENTE (Fase 7): aquí irá la acción del atornillador. Por ahora, una espera.
        if self._elapsed() < self.cfg.SEQ_SCREW_DWELL:
            return
        self._do(lambda: self.b.arm_lin(self.point['id'], 'retreat', who=self.who), 'retreat')

    def _poll_verify_ret(self):
        self._verify(self._screw_done)

    def _screw_done(self):
        s = self._cur()
        with self._lock:
            key = (s['row'], s['col'])
            self.status[key] = 'done'
            self.times[key] = round(time.monotonic() - self._screw_t, 1)
            self.index += 1
            n, total = self.index, len(self.plan)
        log.info('SECUENCIA: tornillo %s hecho (%d/%d, %.1f s)', self._label(s), n, total, self.times[key])
        self._next_screw()

    def _poll_reposo(self):
        with self._lock:
            started = self._phase_started
        if not started:
            if not self._quiet():
                return
            pose = self.b.safe_pose()
            with self._lock:
                self._phase_started = True
                self._phase_t = time.monotonic()
            try:
                self.b.arm.move_to(pose['positions'], who=self.who, label=pose['name'],
                                   speed=self.cfg.ARM_SAFE_SPEED, internal=True)
            except CommandRejected as e:
                self._fail(f'no se pudo ir a reposo: {e}')
            return
        if self._elapsed() < 0.3 or not self._quiet():
            return
        if self.b.arm_in_safe_pose():
            with self._lock:
                canceling = self.state == 'canceling'
            self._finish('canceled' if canceling else 'done')
        else:
            self._fail('el brazo no ha llegado a reposo')

    # Cancelación: esperar a que se pare, retroceder si está avanzada y reposo
    def _poll_cancel_retreat(self):
        with self._lock:
            started = self._phase_started
        if not started:
            if not self._quiet():
                return
            plan = self.b.retreat_plan_now()
            with self._lock:
                self._phase_started = True
                self._phase_t = time.monotonic()
            if plan is None:
                self._set_phase('cancel_reposo', started=False)
                return
            pos, quat, label = plan
            try:
                self.b.arm.move_tcp(pos, quat, who=self.who, label=label,
                                    lin_speed=self.cfg.LIN_SPEED_RETREAT, internal=True)
            except CommandRejected as e:
                self._finish('canceled', f'cancelada; no se pudo retroceder: {e}')
            return
        if self._elapsed() < 0.3 or not self._quiet():
            return
        self._set_phase('cancel_reposo', started=False)

    def _poll_cancel_reposo(self):
        self._poll_reposo()

    # ------------------------------------------------------------ final / fallo
    def _halt_motion(self, who):
        self.b.arm.cancel_move(who, quiet=True)
        self.b.arm.servo.stop(who)
        if self.b.nav_running():
            self.b.cancel_goal(who)

    def _fail(self, reason):
        with self._lock:
            if self.state not in RUNNING:
                return
            canceling = self.state == 'canceling'
        if canceling:
            self._finish('canceled', f'cancelada con error: {reason}')
            return
        self._halt_motion(self.who)
        self._to_paused(reason)

    def _to_paused(self, reason):
        with self._lock:
            self.state = 'paused'
            self.phase = None
            self.error = reason
            if self.index < len(self.plan):
                s = self.plan[self.index]
                if self.status.get((s['row'], s['col'])) == 'doing':
                    self.status[(s['row'], s['col'])] = 'pending'
        if reason:
            log.warning('SECUENCIA en pausa por fallo: %s', reason)
            self.b.emit('warning', 'seq', f'Secuencia en pausa: {reason}. «Reanudar» repite el tornillo')
        else:
            log.info('SECUENCIA en pausa')
            self.b.emit('info', 'seq', 'Secuencia en pausa')

    def _finish(self, state, error=None):
        with self._lock:
            self.state = state
            self.phase = None
            self.error = error
            self._ended = time.monotonic()
            done = sum(1 for v in self.status.values() if v == 'done')
            total = len(self.plan)
            elapsed = self._ended - self._started
        if state == 'done':
            log.info('SECUENCIA terminada: %d tornillos en %.0f s', done, elapsed)
            self.b.emit('info', 'seq', f'Secuencia terminada: {done}/{total} tornillos en {elapsed:.0f} s')
        else:
            log.warning('SECUENCIA cancelada (%d/%d hechos)%s', done, total, f': {error}' if error else '')
            self.b.emit('warning', 'seq', f'Secuencia cancelada: {done}/{total} tornillos hechos'
                        + (f' ({error})' if error else ''))
