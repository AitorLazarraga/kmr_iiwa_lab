// Panel del brazo: jog articular (mantener pulsado − / +), estado de las 7
// articulaciones con sus límites, pose del TCP y posturas guardadas.
//
// El jog manda {type:'arm_jog', v:[7 x rad/s]} a 10 Hz mientras se mantiene
// pulsado un botón; al soltar manda v=0 una vez. Si se pierde la conexión, el
// watchdog del gateway para el brazo. Se mueve una articulación cada vez.

import { api } from './api.js';

const SEND_PERIOD_MS = 100;
const NEAR_LIMIT = 10 * Math.PI / 180;     // aviso visual a menos de 10° del límite
const deg = (rad) => rad * 180 / Math.PI;
const MODE_TEXT = { jog: 'jog', trajectory: 'trayectoria', switching: 'cambiando…', none: 'sin controlador', unknown: '…' };
const MOVE_TEXT = { pending: 'preparando', sending: 'enviando', active: 'moviendo', succeeded: 'alcanzada',
  aborted: 'FALLIDO', canceled: 'cancelado', rejected: 'RECHAZADO' };

export class ArmPanel {
  constructor({ send, notify }) {
    this.send = send;
    this.notify = notify;
    this.$ = (id) => document.getElementById(id);
    this.joints = ['A1', 'A2', 'A3', 'A4', 'A5', 'A6', 'A7'];
    this.jogMax = 15 * Math.PI / 180;
    this.scale = 0.5;
    this.enabled = false;
    this.state = null;
    this.poses = [];
    this.held = null;               // { i, dir, btn }
    this.timer = null;

    this._buildRows();
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
    const moving = move && ['pending', 'sending', 'active'].includes(move.state);
    if (moving) {
      const pct = move.progress != null ? ` ${Math.round(move.progress * 100)}%` : '';
      status.textContent = `Yendo a ${move.label || 'postura'} · ${MOVE_TEXT[move.state]}${pct} (${move.duration}s)`;
      status.className = 'arm-status moving';
    } else if (arm.jog_active) {
      status.textContent = 'Jog en curso';
      status.className = 'arm-status moving';
    } else if (!arm.limits) {
      status.textContent = 'Esperando límites del brazo (robot_description)';
      status.className = 'arm-status bad';
    } else if (move && move.state !== 'succeeded') {
      status.textContent = `Último movimiento a ${move.label || 'postura'}: ${MOVE_TEXT[move.state] || move.state}`;
      status.className = 'arm-status';
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
    const v = [0, 0, 0, 0, 0, 0, 0];
    v[this.held.i] = this.held.dir * this.jogMax * this.scale;
    if (!this.send({ type: 'arm_jog', v })) this._release();
  }

  _release() {
    if (this.timer) { clearInterval(this.timer); this.timer = null; }
    if (this.held) {
      this.held.btn.classList.remove('held');
      this.held = null;
      this.send({ type: 'arm_jog', v: [0, 0, 0, 0, 0, 0, 0] });
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
    if (!this.send({ type: 'arm_move', pose_id: p.id })) {
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
      b.textContent = p.name;
      const s = document.createElement('small');
      s.textContent = p.positions.map((x) => deg(x).toFixed(0)).join(' · ');
      info.append(b, s);
      const actions = document.createElement('div');
      actions.className = 'pp-actions-row';
      actions.append(
        btn('Ir', 'needs-control', () => this.goToPose(p), 'Mover el brazo a esta postura'),
        btn('↺', 'needs-operator', () => this.overwritePose(p), 'Sustituir por la posición actual'),
        btn('✕', 'needs-operator danger-text', () => this.deletePose(p), 'Borrar'),
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
    this.$('btn-save-arm-pose').addEventListener('click', () => this.savePose());
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
