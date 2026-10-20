// Punto de entrada: conecta el gateway con los paneles.

import { Alerts } from './alerts.js';
import { Connection } from './connection.js';
import { MapView } from './mapview.js';
import { Teleop } from './teleop.js';

const $ = (id) => document.getElementById(id);
const STATE_TIMEOUT_MS = 2000;     // sin 'state' en este tiempo = gateway colgado

const alerts = new Alerts({
  bannerEl: $('banner'), listEl: $('alert-list'), clearBtn: $('btn-clear-alerts'),
});

let lastState = null;
let lastStateAt = 0;

// ------------------------------------------------------------------ conexión
const conn = new Connection({
  onStatus: (connected) => {
    setDot('st-client', connected ? 'on' : 'off');
    alerts.condition('ws_lost', !connected, 'emergency',
      'SIN CONEXIÓN con el gateway: el robot no recibe órdenes de este panel');
    if (connected) alerts.push('info', 'Conectado al gateway');
    else { setDot('st-ros', ''); teleop.releaseAll(); }
  },
  onMessage: (msg) => handlers[msg.type]?.(msg),
});

const handlers = {
  config: (msg) => teleop.setLimits(msg.limits),
  state: (msg) => onState(msg),
  map: (msg) => { map.setMap(msg); $('map-hint').classList.add('hidden'); },
  scan: (msg) => map.setScan(msg),
  event: (msg) => alerts.push(msg.level, msg.message, msg.time),
  error: (msg) => alerts.push('warning', msg.message),
};

// ------------------------------------------------------------------ paneles
const map = new MapView($('map'), {
  onGoal: (g) => {
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
  $('st-th').textContent = p ? `${(p.yaw * 180 / Math.PI).toFixed(0)}°` : '—';
  if (p) map.setPose(p);

  const nav = s.nav || {};
  setDot('st-route', s.route_running ? 'busy' : '');
  $('st-route-txt').textContent = s.route_running
    ? (nav.distance_remaining != null ? `${nav.distance_remaining.toFixed(1)} m` : 'iniciando')
    : '';

  map.setGoal(nav.goal ? { ...nav.goal, state: nav.state } : null);
  $('goal-info').textContent = nav.goal
    ? `Goal x=${nav.goal.x.toFixed(2)} y=${nav.goal.y.toFixed(2)} ` +
      `θ=${(nav.goal.yaw * 180 / Math.PI).toFixed(0)}° · ${NAV_TEXT[nav.state] || nav.state}`
    : 'Sin goal';
}

const NAV_TEXT = {
  idle: 'sin ruta', sending: 'enviando', active: 'en ruta', succeeded: 'alcanzado',
  aborted: 'FALLIDO', canceled: 'cancelado', rejected: 'RECHAZADO',
};

// Gateway conectado pero sin enviar estado: algo va mal en el servidor.
setInterval(() => {
  const stale = conn.connected && lastState && performance.now() - lastStateAt > STATE_TIMEOUT_MS;
  alerts.condition('stale', stale, 'emergency', 'El gateway no envía estado: posible bloqueo');
}, 500);

function setDot(id, cls) {
  const el = $(id);
  el.className = 'dot' + (cls ? ' ' + cls : '');
}

// ------------------------------------------------------------------ controles
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
