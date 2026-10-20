// Teleoperación: joystick de traslación (solo hacia delante), deslizador de
// giro y teclado. Envía órdenes a 10 Hz mientras hay algún control activo;
// al soltar todo manda una orden cero y deja de enviar. Si se pierde la
// conexión, el watchdog del gateway para el robot.

const SEND_PERIOD_MS = 100;

export class Teleop {
  constructor({ joyEl, rotEl, send, onCommand }) {
    this.send = send;               // (msg) => bool
    this.onCommand = onCommand;     // ({vx,vy,wz}) => void, para mostrar
    this.limits = { vx_min: 0, vx_max: 0.5, vy_max: 0.5, wz_max: 1.0 };
    this.scale = 0.5;

    this.joy = { x: 0, y: 0, active: false };   // normalizado [-1, 1]
    this.rot = { x: 0, active: false };
    this.keys = new Set();
    this.timer = null;
    this.enabled = false;           // solo con el control del robot

    this._bindJoystick(joyEl);
    this._bindSlider(rotEl);
    this._bindKeyboard();
  }

  setLimits(limits) { this.limits = limits; }

  /** Activa o desactiva los mandos. Al desactivar, suelta todo y manda cero. */
  setEnabled(enabled) {
    if (enabled === this.enabled) return;
    this.enabled = enabled;
    if (!enabled) {
      const wasMoving = this.timer !== null;
      this.releaseAll();
      if (wasMoving) this.send({ type: 'teleop', vx: 0, vy: 0, wz: 0 });
    }
  }
  setScale(scale) { this.scale = scale; }

  /** Suelta todos los controles sin mandar nada (tras una PARADA). */
  releaseAll() {
    this.joy = { x: 0, y: 0, active: false };
    this.rot = { x: 0, active: false };
    this.keys.clear();
    this._renderJoy(); this._renderRot();
    this._stopLoop(false);
  }

  // ------------------------------------------------------------ cálculo
  _command() {
    const L = this.limits, s = this.scale;
    let fwd = -this.joy.y;            // arriba = avanzar
    let lat = -this.joy.x;            // derecha = moverse a la derecha (vy < 0)
    let rot = -this.rot.x;            // izquierda = giro antihorario (wz > 0)

    if (this.keys.has('w')) fwd = 1;
    if (this.keys.has('a')) lat = 1;
    if (this.keys.has('d')) lat = -1;
    if (this.keys.has('q')) rot = 1;
    if (this.keys.has('e')) rot = -1;

    // Sin marcha atrás: el robot no la admite (el gateway también la rechaza).
    const vx = Math.max(L.vx_min, fwd * L.vx_max * s);
    return {
      vx: round(Math.min(vx, L.vx_max)),
      vy: round(clamp(lat, -1, 1) * L.vy_max * s),
      wz: round(clamp(rot, -1, 1) * L.wz_max * s),
    };
  }

  _anyActive() { return this.joy.active || this.rot.active || this.keys.size > 0; }

  _update() {
    if (this._anyActive()) {
      if (!this.timer) {
        this._tick();
        this.timer = setInterval(() => this._tick(), SEND_PERIOD_MS);
      }
    } else {
      this._stopLoop(true);
    }
  }

  _tick() {
    const cmd = this._command();
    this.send({ type: 'teleop', ...cmd });
    this.onCommand(cmd);
  }

  _stopLoop(sendZero) {
    if (this.timer) { clearInterval(this.timer); this.timer = null; }
    if (sendZero) this.send({ type: 'teleop', vx: 0, vy: 0, wz: 0 });
    this.onCommand({ vx: 0, vy: 0, wz: 0 });
  }

  // ------------------------------------------------------------ joystick XY
  _bindJoystick(el) {
    this.joyEl = el;
    this.joyKnob = el.querySelector('.joy-knob');
    const move = (e) => {
      const r = el.getBoundingClientRect();
      const R = r.width / 2;
      let x = (e.clientX - r.left - R) / R;
      let y = (e.clientY - r.top - R) / R;
      y = Math.min(0, y);                         // solo mitad superior (avance)
      const n = Math.hypot(x, y);
      if (n > 1) { x /= n; y /= n; }
      this.joy.x = x; this.joy.y = y;
      this._renderJoy();
    };
    el.addEventListener('pointerdown', (e) => {
      if (!this.enabled) return;
      el.setPointerCapture(e.pointerId);
      this.joy.active = true; el.classList.add('active');
      move(e); this._update();
    });
    el.addEventListener('pointermove', (e) => { if (this.joy.active) move(e); });
    const end = () => {
      if (!this.joy.active) return;
      this.joy = { x: 0, y: 0, active: false };
      el.classList.remove('active');
      this._renderJoy(); this._update();
    };
    el.addEventListener('pointerup', end);
    el.addEventListener('pointercancel', end);
  }

  _renderJoy() {
    const R = this.joyEl.clientWidth / 2 - 28;
    this.joyKnob.style.transform =
      `translate(calc(-50% + ${this.joy.x * R}px), calc(-50% + ${this.joy.y * R}px))`;
  }

  // ------------------------------------------------------------ giro
  _bindSlider(el) {
    this.rotEl = el;
    this.rotKnob = el.querySelector('.slider-knob');
    const move = (e) => {
      const r = el.getBoundingClientRect();
      const half = r.width / 2;
      this.rot.x = clamp((e.clientX - r.left - half) / (half - 24), -1, 1);
      this._renderRot();
    };
    el.addEventListener('pointerdown', (e) => {
      if (!this.enabled) return;
      el.setPointerCapture(e.pointerId);
      this.rot.active = true; el.classList.add('active');
      move(e); this._update();
    });
    el.addEventListener('pointermove', (e) => { if (this.rot.active) move(e); });
    const end = () => {
      if (!this.rot.active) return;
      this.rot = { x: 0, active: false };
      el.classList.remove('active');
      this._renderRot(); this._update();
    };
    el.addEventListener('pointerup', end);
    el.addEventListener('pointercancel', end);
  }

  _renderRot() {
    const R = this.rotEl.clientWidth / 2 - 24;
    this.rotKnob.style.transform = `translate(calc(-50% + ${this.rot.x * R}px), -50%)`;
  }

  // ------------------------------------------------------------ teclado
  _bindKeyboard() {
    const TELEOP_KEYS = ['w', 'a', 'd', 'q', 'e'];
    window.addEventListener('keydown', (e) => {
      if (e.target.closest('input, textarea, select')) return;
      const k = e.key.toLowerCase();
      if (this.enabled && TELEOP_KEYS.includes(k) && !e.repeat) {
        this.keys.add(k); this._update();
      }
    });
    window.addEventListener('keyup', (e) => {
      const k = e.key.toLowerCase();
      if (this.keys.delete(k)) this._update();
    });
    // Si la ventana pierde el foco con una tecla pulsada, se suelta todo.
    window.addEventListener('blur', () => {
      if (this.keys.size) { this.keys.clear(); this._update(); }
    });
  }
}

function clamp(v, a, b) { return Math.min(b, Math.max(a, v)); }
function round(v) { return Math.round(v * 1000) / 1000; }
