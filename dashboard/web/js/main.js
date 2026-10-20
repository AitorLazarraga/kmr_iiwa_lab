// Punto de entrada: sesión, conexión con el gateway y paneles.

import { Alerts } from './alerts.js';
import { Connection } from './connection.js';
import { MapView } from './mapview.js';
import { Teleop } from './teleop.js';

const $ = (id) => document.getElementById(id);
const STATE_TIMEOUT_MS = 2000;     // sin 'state' en este tiempo = gateway colgado
const ROLE_TEXT = { observer: 'observador', operator: 'operador', admin: 'administrador' };

const alerts = new Alerts({
  bannerEl: $('banner'), listEl: $('alert-list'), clearBtn: $('btn-clear-alerts'),
});

let me = null;               // { username, role }
let hasControl = false;
let lastState = null;
let lastStateAt = 0;

// ------------------------------------------------------------------ sesión
async function api(path, options = {}) {
  const r = await fetch(path, {
    credentials: 'same-origin',
    headers: { 'Content-Type': 'application/json' },
    ...options,
  });
  let body = null;
  try { body = await r.json(); } catch { /* sin cuerpo */ }
  return { ok: r.ok, status: r.status, body };
}

function showLogin(message = '') {
  conn.stop();
  setControl(false);
  $('login-error').textContent = message;
  $('login').hidden = false;
  $('login-pass').value = '';
  $('login-user').focus();
}

function startSession(user) {
  me = user;
  $('user-chip').innerHTML = `<b>${escapeHtml(user.username)}</b> · ${ROLE_TEXT[user.role] || user.role}`;
  $('login').hidden = true;
  conn.start();
}

$('login-form').addEventListener('submit', async (e) => {
  e.preventDefault();
  $('login-btn').disabled = true;
  $('login-error').textContent = '';
  const r = await api('/api/login', {
    method: 'POST',
    body: JSON.stringify({ username: $('login-user').value.trim(), password: $('login-pass').value }),
  });
  $('login-btn').disabled = false;
  if (r.ok) startSession(r.body);
  else $('login-error').textContent = r.body?.detail || `Error ${r.status}`;
});

$('btn-logout').addEventListener('click', async () => {
  await api('/api/logout', { method: 'POST' });
  me = null;
  showLogin();
});

// ------------------------------------------------------------------ conexión
const conn = new Connection({
  onStatus: (connected) => {
    setDot('st-client', connected ? 'on' : 'off');
    alerts.condition('ws_lost', !connected && me !== null, 'emergency',
      'SIN CONEXIÓN con el gateway: el robot no recibe órdenes de este panel');
    if (connected) alerts.push('info', 'Conectado al gateway');
    else { setDot('st-ros', ''); setControl(false); }
  },
  onAuthLost: () => {
    alerts.condition('ws_lost', false);
    showLogin('La sesión ha caducado o ha sido cerrada. Vuelve a entrar.');
  },
  onMessage: (msg) => handlers[msg.type]?.(msg),
});

const handlers = {
  config: (msg) => teleop.setLimits(msg.limits),
  state: (msg) => onState(msg),
  map: (msg) => { map.setMap(msg); $('map-hint').classList.add('hidden'); },
  scan: (msg) => map.setScan(msg),
  event: (msg) => alerts.push(msg.level, msg.message, msg.time),
  notice: (msg) => alerts.push(msg.level, msg.message),
  error: (msg) => alerts.push('warning', msg.message),
};

// ------------------------------------------------------------------ paneles
const map = new MapView($('map'), {
  onGoal: (g) => {
    if (!hasControl) { alerts.push('warning', 'Necesitas el control para enviar goals'); return; }
    if (!conn.send({ type: 'goal', x: g.x, y: g.y, yaw: g.yaw })) {
      alerts.push('warning', 'Sin conexión: el goal no se ha enviado');
    }
  },
});

const teleop = new Teleop({
  joyEl: $('joy-xy'),
  rotEl: $('joy-rot'),
  send: (msg) => conn.send(msg),
  onCommand: (c) => {
    $('cmd-vx').textContent = c.vx.toFixed(2);
    $('cmd-vy').textContent = c.vy.toFixed(2);
    $('cmd-wz').textContent = c.wz.toFixed(2);
  },
});

// ------------------------------------------------------------------ estado
function onState(s) {
  lastState = s;
  lastStateAt = performance.now();

  setDot('st-ros', s.ros_ok ? 'on' : 'off');
  alerts.condition('ros_lost', !s.ros_ok, 'emergency',
    'CONEXIÓN CON ROS PERDIDA: no llega la pose del robot');
  alerts.condition('nav2_down', s.ros_ok && !s.nav2_ready, 'warning',
    'Nav2 no disponible: no se pueden enviar goals');
  alerts.condition('stale', false);

  const p = s.pose;
  $('st-x').textContent = p ? p.x.toFixed(2) : '—';
  $('st-y').textContent = p ? p.y.toFixed(2) : '—';
  $('st-th').textContent = p ? `${deg(p.yaw)}°` : '—';
  if (p) map.setPose(p);

  const nav = s.nav || {};
  setDot('st-route', s.route_running ? 'busy' : '');
  $('st-route-txt').textContent = s.route_running
    ? (nav.distance_remaining != null ? `${nav.distance_remaining.toFixed(1)} m` : 'iniciando')
    : '';

  map.setGoal(nav.goal ? { ...nav.goal, state: nav.state } : null);
  $('goal-info').textContent = nav.goal
    ? `Goal x=${nav.goal.x.toFixed(2)} y=${nav.goal.y.toFixed(2)} ` +
      `θ=${deg(nav.goal.yaw)}° · ${NAV_TEXT[nav.state] || nav.state}`
    : 'Sin goal';

  renderControl(s.control);
}

const NAV_TEXT = {
  idle: 'sin ruta', sending: 'enviando', active: 'en ruta', succeeded: 'alcanzado',
  aborted: 'FALLIDO', canceled: 'cancelado', rejected: 'RECHAZADO',
};

// ------------------------------------------------------------------ control
function renderControl(c) {
  if (!c) return;
  setControl(c.you);
  const btn = $('btn-control');
  const h = c.holder;

  if (c.you) {
    setDot('st-control', 'mine');
    $('control-txt').textContent = 'Tienes el control';
  } else if (h) {
    setDot('st-control', h.connected ? 'other' : 'away');
    $('control-txt').textContent = `Control: ${h.username}${h.connected ? '' : ' (desconectado)'}`;
  } else {
    setDot('st-control', '');
    $('control-txt').textContent = 'Control libre';
  }

  const canOperate = me && me.role !== 'observer';
  btn.hidden = !canOperate;
  btn.classList.toggle('has', c.you);
  btn.textContent = c.you ? 'Liberar control' : 'Tomar control';
  btn.disabled = !c.you && h && !c.can_take;
  btn.title = btn.disabled ? `${h.username} tiene un rango superior` : '';
}

function setControl(value) {
  hasControl = value;
  teleop.setEnabled(value);
  document.body.classList.toggle('no-control', !value);
  $('no-control-note').textContent = me?.role === 'observer'
    ? 'Rol observador: solo visualización (la PARADA sigue disponible).'
    : 'Pulsa «Tomar control» para mover el robot.';
}

$('btn-control').addEventListener('click', () => {
  conn.send({ type: hasControl ? 'release_control' : 'take_control' });
});

// Gateway conectado pero sin enviar estado: algo va mal en el servidor.
setInterval(() => {
  const stale = conn.connected && lastState && performance.now() - lastStateAt > STATE_TIMEOUT_MS;
  alerts.condition('stale', stale, 'emergency', 'El gateway no envía estado: posible bloqueo');
}, 500);

/** Radianes -> grados enteros en (-180, 180]. */
function deg(rad) {
  let d = Math.round(rad * 180 / Math.PI) % 360;
  if (d > 180) d -= 360;
  if (d <= -180) d += 360;
  return d;
}

function setDot(id, cls) {
  $(id).className = 'dot' + (cls ? ' ' + cls : '');
}

function escapeHtml(s) {
  return s.replace(/[&<>"']/g, (ch) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[ch]));
}

// ------------------------------------------------------------------ mandos
function emergencyStop() {
  teleop.releaseAll();
  if (!conn.send({ type: 'stop' })) {
    alerts.push('emergency',
      'Sin conexión: la PARADA no se ha podido enviar. Usa la parada física del robot.');
  }
}
$('btn-stop').addEventListener('click', emergencyStop);
window.addEventListener('keydown', (e) => {
  if (e.code === 'Space' && !e.target.closest('input, textarea, select')) {
    e.preventDefault();
    emergencyStop();
  }
});

$('btn-clear-goal').addEventListener('click', () => conn.send({ type: 'cancel' }));
$('btn-center').addEventListener('click', () => map.centerRobot());
$('btn-fit').addEventListener('click', () => map.fitMap());

document.querySelectorAll('[data-tool]').forEach((btn) => {
  btn.addEventListener('click', () => {
    document.querySelectorAll('[data-tool]').forEach((b) => b.classList.toggle('active', b === btn));
    map.setTool(btn.dataset.tool);
    document.querySelector('.map-panel').classList.toggle('tool-goal', btn.dataset.tool === 'goal');
  });
});

document.querySelectorAll('[data-speed]').forEach((btn) => {
  btn.addEventListener('click', () => {
    document.querySelectorAll('[data-speed]').forEach((b) => b.classList.toggle('active', b === btn));
    teleop.setScale(parseFloat(btn.dataset.speed));
  });
});

// ------------------------------------------------------------------ arranque
setControl(false);
api('/api/me').then((r) => (r.ok ? startSession(r.body) : showLogin()));
