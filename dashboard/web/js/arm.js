// Panel del brazo: jog articular (mantener pulsado − / +), estado de las 7
// articulaciones con sus límites, pose del TCP, posturas guardadas y, frente a
// un punto de trabajo, la rejilla de tornillos («ir a tornillo»).
// Ir a postura e ir a tornillo los planifica MoveIt en el gateway (5.3), a la
// velocidad elegida en «Velocidad MoveIt».
//
// Jog cartesiano (5.4): {type:'arm_cart', v:[vx,vy,vz,wx,wy,wz], frame} a 10 Hz
// mientras se mantiene pulsado, con MoveIt Servo; mismo selector de velocidad
// que el jog articular, sobre el máximo de config (5 cm/s, 10°/s).
//
// El jog manda {type:'arm_jog', v:[7 x rad/s]} a 10 Hz mientras se mantiene
// pulsado un botón; al soltar manda v=0 una vez. Si se pierde la conexión, el
// watchdog del gateway para el brazo. Se mueve una articulación cada vez.

import { api } from './api.js';

const SEND_PERIOD_MS = 100;
const NEAR_LIMIT = 10 * Math.PI / 180;     // aviso visual a menos de 10° del límite
const deg = (rad) => rad * 180 / Math.PI;
const MODE_TEXT = { jog: 'jog', trajectory: 'trayectoria', switching: 'cambiando…', none: 'sin controlador', unknown: '…' };
const MOVE_TEXT = { pending: 'preparando', sending: 'enviando', planning: 'planificando', active: 'moviendo',
  succeeded: 'alcanzada', aborted: 'FALLIDO', canceled: 'cancelado', rejected: 'RECHAZADO' };
const RUNNING = ['pending', 'sending', 'planning', 'active'];

// Resultado de check_tcp_orientation (move.check)
function checkText(c) {
  if (c.state === 'checking') return 'comprobando TCP…';
  if (c.state === 'error') return `TCP sin comprobar — ${c.message}`;
  const errs = `${(c.pos_error * 1000).toFixed(1)} mm · ${c.angle_error.toFixed(2)}°`;
  return c.state === 'ok' ? `TCP verificado ✓ (${errs})`
    : `TCP FUERA DE TOLERANCIA: ${errs} (máx ${(c.pos_tol * 1000).toFixed(0)} mm · ${c.angle_tol}°)`;
}

export class ArmPanel {
  constructor({ send, notify }) {
    this.send = send;
    this.notify = notify;
    this.$ = (id) => document.getElementById(id);
    this.joints = ['A1', 'A2', 'A3', 'A4', 'A5', 'A6', 'A7'];
    this.jogMax = 15 * Math.PI / 180;
    this.scale = 0.5;
    this.cartFrame = 'tool';
    this.cartMax = { linear: 0.05, angular: 10 * Math.PI / 180 };   // los envía el gateway (state.servo)
    this.enabled = false;
    this.state = null;
    this.poses = [];
    this.held = null;               // { i, dir, btn }
    this.timer = null;
    this.planSpeed = 0.25;          // velocidad de MoveIt (fracción)
    this.workHere = null;           // state.work_here: punto de trabajo frente al que está el robot
    this.safeName = 'reposo';       // nombre de la postura de reposo (state.safe_pose.name)
    this._safeKey = null;

    this._buildRows();
    this._buildCart();
    this._bind();
  }

  // ------------------------------------------------------------ datos
  setConfig(armCfg) {
    if (!armCfg) return;
    this.jogMax = armCfg.jog_max_vel;
    if (armCfg.joints) {
      this.joints = armCfg.joints.map((j) => j.replace(/^.*_(A\d)$/, '$1'));
      this.rows.forEach((r, i) => { r.name.textContent = this.joints[i] || `A${i + 1}`; });
    }
  }

  setEnabled(enabled) {
    if (enabled === this.enabled) return;
    this.enabled = enabled;
    if (!enabled) this._release();
  }

  async reloadPoses() {
    const r = await api('/api/arm/poses');
    if (!r.ok) return;
    this.poses = r.body;
    this._renderPoses();
  }

  setState(arm) {
    this.state = arm;
    const status = this.$('arm-status');
    const modeEl = this.$('arm-mode');
    modeEl.textContent = arm && arm.mode ? `brazo: ${MODE_TEXT[arm.mode] || arm.mode}` : '';
    modeEl.className = 'arm-mode ' + (arm?.mode || '');

    if (!arm || !arm.ok || !arm.positions) {
      status.textContent = 'Sin datos del brazo (joint_states)';
      status.className = 'arm-status bad';
      this._release();
      return;
    }

    const move = arm.move;
    const moving = move && RUNNING.includes(move.state);
    this._renderServo(arm.servo);
    const mi = this.$('arm-moveit');
    mi.textContent = arm.moveit_ready ? 'MoveIt listo' : 'MoveIt no disponible: ir a postura / tornillo desactivados';
    mi.className = 'arm-moveit ' + (arm.moveit_ready ? 'ok' : 'bad');
    if (moving) {
      const t = move.elapsed != null ? ` ${move.elapsed}s` : '';
      const v = move.speed != null ? ` · ${Math.round(move.speed * 100)}%` : '';
      status.textContent = `Yendo a ${move.label || 'postura'} · ${MOVE_TEXT[move.state]}${t}${v}`;
      status.className = 'arm-status moving';
    } else if (arm.jog_active) {
      status.textContent = 'Jog en curso';
      status.className = 'arm-status moving';
    } else if (!arm.limits) {
      status.textContent = 'Esperando límites del brazo (robot_description)';
      status.className = 'arm-status bad';
    } else if (move && move.state === 'succeeded' && move.check) {
      status.textContent = `${move.label || 'Pose'}: ${checkText(move.check)}`;
      status.className = 'arm-status' + (move.check.state === 'ok' ? ' ok'
        : move.check.state === 'checking' ? ' moving' : ' bad');
    } else if (move && move.state !== 'succeeded') {
      const why = move.reason ? ` — ${move.reason}` : '';
      status.textContent = `Último movimiento a ${move.label || 'postura'}: ${MOVE_TEXT[move.state] || move.state}${why}`;
      status.className = 'arm-status' + (move.state === 'aborted' ? ' bad' : '');
    } else {
      status.textContent = 'Mantén pulsado − / + para mover una articulación';
      status.className = 'arm-status';
    }
    this.$('btn-arm-cancel').classList.toggle('hidden', !moving);

    arm.positions.forEach((p, i) => {
      const row = this.rows[i];
      const lim = arm.limits && arm.limits[i];
      row.val.textContent = `${deg(p).toFixed(1)}°`;
      let near = false;
      if (lim) {
        const span = lim.soft_upper - lim.soft_lower;
        row.mark.style.left = `${Math.min(100, Math.max(0, (p - lim.soft_lower) / span * 100))}%`;
        near = p - lim.soft_lower < NEAR_LIMIT || lim.soft_upper - p < NEAR_LIMIT;
        if (moving && move.target) {
          row.target.style.display = '';
          row.target.style.left = `${(move.target[i] - lim.soft_lower) / span * 100}%`;
        } else {
          row.target.style.display = 'none';
        }
        row.bar.title = `Límites: ${deg(lim.soft_lower).toFixed(0)}° … ${deg(lim.soft_upper).toFixed(0)}°`;
      }
      row.mark.classList.toggle('near', near);
      row.val.classList.toggle('near', near);
    });

    const t = arm.tcp;
    const mm = (v) => (v * 1000).toFixed(1);
    this.$('tcp-frame').textContent = t ? `respecto a ${t.frame}` : '';
    this.$('tcp-x').textContent = t ? mm(t.x) : '—';
    this.$('tcp-y').textContent = t ? mm(t.y) : '—';
    this.$('tcp-z').textContent = t ? mm(t.z) : '—';
    this.$('tcp-r').textContent = t ? deg(t.roll).toFixed(1) : '—';
    this.$('tcp-p').textContent = t ? deg(t.pitch).toFixed(1) : '—';
    this.$('tcp-yw').textContent = t ? deg(t.yaw).toFixed(1) : '—';
  }

  // ------------------------------------------------------------ Servo
  _renderServo(sv) {
    const el = this.$('arm-servo');
    if (!sv) { el.textContent = ''; return; }
    if (sv.max_linear) this.cartMax = { linear: sv.max_linear, angular: sv.max_angular };
    let text; let cls;
    if (!sv.ready) {
      text = 'Servo no disponible (servo_node en moveit_sim.launch.py): jog cartesiano y corrección desactivados';
      cls = 'bad';
    } else if (sv.correction) {
      const c = sv.correction;
      text = `Corrigiendo ${c.label} · intento ${c.attempt}/${c.attempts} · ${(c.pos_error * 1000).toFixed(1)} mm · ${c.angle_error.toFixed(2)}°`;
      cls = 'warn';
    } else if (sv.jog_active) {
      text = `Jog cartesiano en curso (${sv.frame === 'tool' ? 'herramienta' : 'base del brazo'})`;
      cls = 'warn';
    } else {
      text = 'Servo listo (en pausa)';
      cls = 'ok';
      if (sv.last_correction) {
        const lc = sv.last_correction;
        text += ` · última corrección: ${lc.ok ? '✓' : '✗'} ${lc.label}`;
      }
    }
    if (sv.status) { text += ` · ${sv.status.text}`; cls = 'bad'; }
    el.textContent = text;
    el.className = `arm-moveit ${cls}`;
  }

  _buildCart() {
    const box = this.$('arm-cart');
    const axes = [['X', 0], ['Y', 1], ['Z', 2], ['Rx', 3], ['Ry', 4], ['Rz', 5]];
    for (const [name, i] of axes) {
      const cell = document.createElement('div');
      cell.className = 'cart-axis';
      const minus = document.createElement('button');
      minus.className = 'aj-btn needs-control';
      minus.textContent = '−';
      minus.title = `${name} negativo`;
      const label = document.createElement('span');
      label.textContent = name;
      const plus = document.createElement('button');
      plus.className = 'aj-btn needs-control';
      plus.textContent = '+';
      plus.title = `${name} positivo`;
      cell.append(minus, label, plus);
      box.append(cell);
      for (const [b, dir] of [[minus, -1], [plus, +1]]) {
        b.addEventListener('pointerdown', (e) => { e.preventDefault(); b.setPointerCapture(e.pointerId); this._pressCart(i, dir, b); });
        b.addEventListener('pointerup', () => this._release());
        b.addEventListener('pointercancel', () => this._release());
        b.addEventListener('lostpointercapture', () => this._release());
        b.addEventListener('contextmenu', (e) => e.preventDefault());
      }
    }
  }

  _pressCart(i, dir, btn) {
    if (!this.enabled || !this.state?.ok) return;
    this._release();
    this.held = { cart: true, i, dir, btn };
    btn.classList.add('held');
    this._tick();
    this.timer = setInterval(() => this._tick(), SEND_PERIOD_MS);
  }

  correctTcp() {
    if (!this.send({ type: 'arm_correct' })) this.notify('warning', 'Sin conexión: la orden no se ha enviado');
  }

  // ------------------------------------------------------------ jog
  _press(i, dir, btn) {
    if (!this.enabled || !this.state?.ok || !this.state?.limits) return;
    this._release();
    this.held = { i, dir, btn };
    btn.classList.add('held');
    this._tick();
    this.timer = setInterval(() => this._tick(), SEND_PERIOD_MS);
  }

  _tick() {
    if (!this.held) return;
    if (this.held.cart) {
      const v = [0, 0, 0, 0, 0, 0];
      const max = this.held.i < 3 ? this.cartMax.linear : this.cartMax.angular;
      v[this.held.i] = this.held.dir * max * this.scale;
      if (!this.send({ type: 'arm_cart', v, frame: this.cartFrame })) this._release();
      return;
    }
    const v = [0, 0, 0, 0, 0, 0, 0];
    v[this.held.i] = this.held.dir * this.jogMax * this.scale;
    if (!this.send({ type: 'arm_jog', v })) this._release();
  }

  _release() {
    if (this.timer) { clearInterval(this.timer); this.timer = null; }
    if (this.held) {
      const cart = this.held.cart;
      this.held.btn.classList.remove('held');
      this.held = null;
      if (cart) this.send({ type: 'arm_cart', v: [0, 0, 0, 0, 0, 0], frame: this.cartFrame });
      else this.send({ type: 'arm_jog', v: [0, 0, 0, 0, 0, 0, 0] });
    }
  }

  // ------------------------------------------------------------ posturas
  async savePose() {
    const name = prompt('Nombre de la postura (se guarda la posición actual del brazo):');
    if (name === null) return;
    const r = await api('/api/arm/poses', { method: 'POST', body: JSON.stringify({ name: name.trim() }) });
    if (!r.ok) this.notify('warning', `No se pudo guardar la postura: ${r.error}`);
  }

  async deletePose(p) {
    if (!confirm(`¿Borrar la postura «${p.name}»?`)) return;
    const r = await api(`/api/arm/poses/${p.id}`, { method: 'DELETE' });
    if (!r.ok) this.notify('warning', `No se pudo borrar: ${r.error}`);
  }

  async overwritePose(p) {
    if (!confirm(`¿Sustituir «${p.name}» por la posición actual del brazo?`)) return;
    const pos = this.state?.positions;
    if (!pos) { this.notify('warning', 'Sin posición actual del brazo'); return; }
    const r = await api(`/api/arm/poses/${p.id}`, { method: 'PATCH', body: JSON.stringify({ positions: pos }) });
    if (!r.ok) this.notify('warning', `No se pudo actualizar: ${r.error}`);
  }

  goToPose(p) {
    if (!this.send({ type: 'arm_move', pose_id: p.id, speed: this.planSpeed })) {
      this.notify('warning', 'Sin conexión: la orden no se ha enviado');
    }
  }

  // ------------------------------------------------------------ tornillos (5.3)
  /** state.work_here: {point_id, name, screws: [{row, col, z, u}], columns: [{col, u}],
   *  column: columna frente a la que está el brazo (o null), est: {distance, lateral, angle}} o null. */
  setWorkHere(w) {
    const key = w ? JSON.stringify(w) : null;
    if (key === this._workKey) return;
    this._workKey = key;
    this.workHere = w;
    this.$('arm-screws').classList.toggle('hidden', !w);
    const grid = this.$('screw-grid');
    const colsEl = this.$('screw-cols');
    grid.replaceChildren();
    colsEl.replaceChildren();
    if (!w) return;
    this.$('arm-screws-title').textContent = `Tornillos · ${w.name}`;
    const rows = Math.max(...w.screws.map((s) => s.row)) + 1;
    const cols = Math.max(...w.screws.map((s) => s.col)) + 1;
    grid.style.gridTemplateColumns = `repeat(${cols}, 1fr)`;
    colsEl.style.gridTemplateColumns = `repeat(${cols}, 1fr)`;
    // Fila de arriba primero (como se ve la pared); columna 0 a la izquierda
    for (let r = rows - 1; r >= 0; r--) {
      for (let c = 0; c < cols; c++) {
        const s = w.screws.find((x) => x.row === r && x.col === c);
        const b = document.createElement('button');
        b.dataset.row = r;
        b.dataset.col = c;
        const toolHere = w.tool && w.tool.screw.row === r && w.tool.screw.col === c;
        b.className = 'ghost needs-control' + (c === w.column ? ' col-here' : '')
          + (toolHere ? (w.tool.state === 'advanced' ? ' tool-adv' : ' tool-here') : '');
        b.innerHTML = `F${r + 1}·C${c + 1}<small>${s.z.toFixed(2)} m</small>`;
        b.title = `TCP delante del tornillo (altura ${s.z.toFixed(2)} m, ${s.u >= 0 ? '+' : ''}${(s.u * 100).toFixed(0)} cm del centro)`;
        b.addEventListener('click', () => this.goToScrew(r, c));
        grid.append(b);
      }
    }
    // Botones de desplazamiento de la base: uno por columna. La referencia es
    // el TCP si está delante de un tornillo (se mide por TF, también tras mover
    // el brazo a mano) o la base del brazo si no.
    const lat = w.reference_u ?? (w.est ? w.est.lateral : 0);
    for (const { col, u } of w.columns || []) {
      const b = document.createElement('button');
      const here = col === w.column;
      b.className = 'ghost needs-control' + (here ? ' here' : '');
      // u + = derecha vista desde el robot: la flecha indica hacia dónde se moverá la base
      const arrow = here ? '●' : (u > lat ? '▶' : '◀');
      b.innerHTML = `${arrow} C${col + 1}<small>${here ? 'aquí' : `${((u - lat) * 100).toFixed(0)} cm`}</small>`;
      b.title = here ? (w.reference === 'tcp' ? 'La herramienta ya está frente a esta columna' : 'El brazo ya está frente a esta columna')
        : `Desplazar la base ${(Math.abs(u - lat) * 100).toFixed(0)} cm a la ${u > lat ? 'derecha' : 'izquierda'}`;
      b.disabled = here;
      b.addEventListener('click', () => this.shiftTo(col));
      colsEl.append(b);
    }
    // Herramienta delante de un tornillo: avance / retroceso LIN (5.5)
    const t = w.tool;
    const txt = this.$('arm-tool-txt');
    if (t) {
      const name = `F${t.screw.row + 1}·C${t.screw.col + 1}`;
      txt.textContent = t.state === 'advanced'
        ? `Herramienta AVANZADA en ${name}: punta del tornillo a ${(t.tip_gap * 1000).toFixed(0)} mm de la pared`
        : `Herramienta en ${name}, a ${(t.gap * 100).toFixed(1)} cm de la pared`;
      txt.className = t.state === 'advanced' ? 'adv' : '';
    } else {
      txt.textContent = 'Herramienta: no está delante de ningún tornillo';
      txt.className = '';
    }
    this.$('arm-lin-adv').dataset.allowed = t && t.state === 'standoff' ? '1' : '';
    this.$('arm-correct').dataset.allowed = t ? '1' : '';
    this.$('arm-lin-ret').dataset.allowed = t && t.state === 'advanced' ? '1' : '';
    this._syncLinButtons();
    const e = w.est;
    const ref = w.reference === 'tcp' && w.tcp_screw
      ? `Referencia: herramienta en F${w.tcp_screw.row + 1}·C${w.tcp_screw.col + 1} (al desplazar, queda en el mismo tornillo de la columna destino)`
      : 'Referencia: base del brazo (la herramienta no está delante de ningún tornillo)';
    this.$('arm-screws-ref').textContent = ref;
    this.$('arm-screws-est').textContent = e
      ? `Base medida: d ${(e.distance * 100).toFixed(1)} cm · lateral ${e.lateral >= 0 ? '+' : ''}${(e.lateral * 1000).toFixed(0)} mm · θ ${(e.angle * 180 / Math.PI).toFixed(2)}°`
      : '';
  }

  /** state.safe_pose: {name, defined, id, in_pose}. */
  setSafePose(sp) {
    const el = this.$('arm-safe');
    if (!sp) { el.textContent = ''; return; }
    if (sp.name.toLowerCase() !== this.safeName) { this.safeName = sp.name.toLowerCase(); this._renderPoses(); }
    if (!sp.defined) {
      el.textContent = `Sin postura «${sp.name}»: la base no se moverá (salvo teleop y desplazamiento lateral) hasta que un administrador la guarde`;
      el.className = 'arm-moveit bad';
    } else if (sp.in_pose) {
      el.textContent = `Brazo en «${sp.name}»: la base puede moverse`;
      el.className = 'arm-moveit ok';
    } else {
      el.textContent = `Brazo fuera de «${sp.name}»: irá allí (50 %) antes de navegar o ajustar`;
      el.className = 'arm-moveit warn';
    }
  }

  /** state.sequence (secuenciador 6.6). */
  setSequence(sq) {
    const txt = this.$('seq-txt');
    const st = sq?.state || 'idle';
    const here = this.workHere && sq && sq.point_id === this.workHere.point_id;
    const running = ['running', 'pausing', 'canceling'].includes(st);
    // Mientras corre, los botones manuales del panel no hacen nada (el gateway los rechazaría)
    this.$('arm-screws').classList.toggle('seq-running', running);
    document.querySelector('.arm-cart')?.classList.toggle('seq-running', running);
    this.$('seq-start').disabled = running || st === 'paused';
    this.$('seq-pause').disabled = st !== 'running';
    this.$('seq-resume').disabled = !(st === 'paused' || st === 'pausing');
    this.$('seq-cancel').disabled = !(running || st === 'paused');
    // Estado de cada tornillo y orden en la rejilla
    const status = {};
    const order = {};
    if (here) {
      sq.screws.forEach((s) => { status[`${s.row},${s.col}`] = s.status; });
      sq.order.forEach(([r, c], i) => { order[`${r},${c}`] = i + 1; });
    }
    this.$('screw-grid').querySelectorAll('button').forEach((b) => {
      const k = `${b.dataset.row},${b.dataset.col}`;
      b.classList.toggle('seq-done', status[k] === 'done');
      b.classList.toggle('seq-doing', status[k] === 'doing');
      let n = b.querySelector('.seq-n');
      if (order[k] && st !== 'idle') {
        if (!n) { n = document.createElement('span'); n.className = 'seq-n'; b.append(n); }
        n.textContent = `${order[k]}`;
      } else if (n) n.remove();
    });
    if (!sq || st === 'idle' || !here) {
      txt.textContent = 'Secuencia: orden en serpiente; atornillado simulado (espera) hasta tener el atornillador';
      txt.className = 'arm-seq-txt';
      return;
    }
    const cur = sq.current ? ` · F${sq.current.row + 1}·C${sq.current.col + 1}` : '';
    const ph = sq.phase_text ? ` · ${sq.phase_text}` : '';
    const prog = `${sq.done}/${sq.total} · ${Math.round(sq.elapsed)} s`;
    const head = { running: 'En marcha', pausing: 'Pausando al terminar el tornillo', paused: 'En pausa',
      canceling: 'Cancelando', done: 'Terminada', canceled: 'Cancelada' }[st] || st;
    txt.textContent = `${head} · ${prog}${cur}${ph}${sq.error ? ` — ${sq.error}` : ''}`;
    txt.className = 'arm-seq-txt ' + (sq.error ? 'bad' : st === 'done' ? 'ok' : running ? 'run' : '');
  }

  goToScrew(row, col) {
    if (!this.workHere) return;
    const msg = { type: 'arm_screw', point_id: this.workHere.point_id, row, col, speed: this.planSpeed };
    if (!this.send(msg)) this.notify('warning', 'Sin conexión: la orden no se ha enviado');
  }

  /** Los botones LIN dependen del estado de la herramienta y de tener el control. */
  _syncLinButtons() {
    for (const id of ['arm-lin-adv', 'arm-lin-ret', 'arm-correct']) {
      const b = this.$(id);
      b.disabled = !b.dataset.allowed;          // sin control: lo bloquea el CSS (needs-control)
    }
  }

  linMove(action) {
    if (!this.workHere) return;
    if (!this.send({ type: 'arm_lin', point_id: this.workHere.point_id, action })) {
      this.notify('warning', 'Sin conexión: la orden no se ha enviado');
    }
  }

  shiftTo(col) {
    if (!this.workHere) return;
    if (!this.send({ type: 'shift', point_id: this.workHere.point_id, col })) {
      this.notify('warning', 'Sin conexión: la orden no se ha enviado');
    }
  }

  _renderPoses() {
    const ul = this.$('arm-poses-list');
    ul.replaceChildren();
    if (!this.poses.length) {
      const li = document.createElement('li');
      li.className = 'pp-empty';
      li.textContent = 'Sin posturas guardadas. Mueve el brazo y pulsa «+ Guardar actual».';
      ul.append(li);
      return;
    }
    for (const p of this.poses) {
      const li = document.createElement('li');
      li.className = 'pp-item';
      const info = document.createElement('div');
      info.className = 'pp-info';
      const b = document.createElement('b');
      const isSafe = p.name.trim().toLowerCase() === this.safeName;
      b.textContent = isSafe ? `⌂ ${p.name}` : p.name;
      if (isSafe) {
        li.classList.add('safe-pose');
        b.title = 'Postura de reposo: el brazo va aquí antes de cualquier movimiento no lateral de la base. Solo un administrador la modifica';
      }
      const s = document.createElement('small');
      s.textContent = p.positions.map((x) => deg(x).toFixed(0)).join(' · ');
      info.append(b, s);
      const actions = document.createElement('div');
      actions.className = 'pp-actions-row';
      actions.append(
        btn('Ir', 'needs-control', () => this.goToPose(p), 'Mover el brazo a esta postura'),
        btn('↺', isSafe ? 'needs-admin' : 'needs-operator', () => this.overwritePose(p), 'Sustituir por la posición actual'),
        btn('✕', `${isSafe ? 'needs-admin' : 'needs-operator'} danger-text`, () => this.deletePose(p), 'Borrar'),
      );
      li.append(info, actions);
      ul.append(li);
    }
  }

  // ------------------------------------------------------------ construcción
  _buildRows() {
    const box = this.$('arm-joints');
    this.rows = this.joints.map((name, i) => {
      const row = document.createElement('div');
      row.className = 'aj-row';
      const label = document.createElement('span');
      label.className = 'aj-name';
      label.textContent = name;
      const minus = document.createElement('button');
      minus.className = 'aj-btn needs-control';
      minus.textContent = '−';
      minus.title = `${name} negativo`;
      const bar = document.createElement('div');
      bar.className = 'aj-bar';
      const mark = document.createElement('div');
      mark.className = 'aj-mark';
      mark.style.left = '50%';
      const target = document.createElement('div');
      target.className = 'aj-mark target';
      target.style.display = 'none';
      bar.append(target, mark);
      const val = document.createElement('span');
      val.className = 'aj-val';
      val.textContent = '—';
      const plus = document.createElement('button');
      plus.className = 'aj-btn needs-control';
      plus.textContent = '+';
      plus.title = `${name} positivo`;
      row.append(label, minus, bar, val, plus);
      box.append(row);

      for (const [b, dir] of [[minus, -1], [plus, +1]]) {
        b.addEventListener('pointerdown', (e) => { e.preventDefault(); b.setPointerCapture(e.pointerId); this._press(i, dir, b); });
        b.addEventListener('pointerup', () => this._release());
        b.addEventListener('pointercancel', () => this._release());
        b.addEventListener('lostpointercapture', () => this._release());
        b.addEventListener('contextmenu', (e) => e.preventDefault());   // pulsación larga en móvil
      }
      return { name: label, bar, mark, target, val };
    });
  }

  _bind() {
    document.querySelectorAll('[data-arm-speed]').forEach((b) => b.addEventListener('click', () => {
      document.querySelectorAll('[data-arm-speed]').forEach((x) => x.classList.toggle('active', x === b));
      this.scale = parseFloat(b.dataset.armSpeed);
    }));
    document.querySelectorAll('[data-arm-plan-speed]').forEach((b) => b.addEventListener('click', () => {
      document.querySelectorAll('[data-arm-plan-speed]').forEach((x) => x.classList.toggle('active', x === b));
      this.planSpeed = parseFloat(b.dataset.armPlanSpeed);
    }));
    this.$('btn-save-arm-pose').addEventListener('click', () => this.savePose());
    this.$('arm-lin-adv').addEventListener('click', () => this.linMove('advance'));
    this.$('arm-lin-ret').addEventListener('click', () => this.linMove('retreat'));
    this.$('arm-correct').addEventListener('click', () => this.correctTcp());
    const seqSend = (type) => () => {
      const msg = { type };
      if (type === 'seq_start') {
        if (!this.workHere) return;
        msg.point_id = this.workHere.point_id;
      }
      if (!this.send(msg)) this.notify('warning', 'Sin conexión: la orden no se ha enviado');
    };
    this.$('seq-start').addEventListener('click', seqSend('seq_start'));
    this.$('seq-pause').addEventListener('click', seqSend('seq_pause'));
    this.$('seq-resume').addEventListener('click', seqSend('seq_resume'));
    this.$('seq-cancel').addEventListener('click', seqSend('seq_cancel'));
    document.querySelectorAll('[data-cart-frame]').forEach((b) => b.addEventListener('click', () => {
      document.querySelectorAll('[data-cart-frame]').forEach((x) => x.classList.toggle('active', x === b));
      this.cartFrame = b.dataset.cartFrame;
    }));
    this.$('btn-arm-cancel').addEventListener('click', () => this.send({ type: 'arm_cancel' }));
    window.addEventListener('blur', () => this._release());
  }
}

function btn(text, cls, onClick, title) {
  const b = document.createElement('button');
  b.className = `ghost small ${cls}`;
  b.textContent = text;
  b.title = title;
  b.addEventListener('click', (e) => { e.stopPropagation(); onClick(); });
  return b;
}
