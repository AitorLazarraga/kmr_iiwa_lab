// Vista 2D del mapa: mapa de ocupación, robot, scan y goal.
// Coordenadas: todo en el frame "map" (metros, y hacia arriba).

const ROBOT_LENGTH = 1.08;   // m (KMP 200)
const ROBOT_WIDTH = 0.63;    // m

const COLORS = {
  bg: '#15181c',
  unknown: [70, 75, 82],
  free: [217, 214, 207],
  occupied: [27, 29, 32],
  robot: '#f2a93b',
  robotEdge: '#1b1d20',
  scan: '#3fd0e0',
  goalActive: '#3d8bfd',
  goalFailed: '#e5484d',
  goalPreview: '#a9c9ff',
};

export class MapView {
  constructor(canvas, { onGoal }) {
    this.canvas = canvas;
    this.ctx = canvas.getContext('2d');
    this.onGoal = onGoal;

    this.map = null;          // { info, image (canvas) }
    this.pose = null;         // { x, y, yaw }
    this.scan = null;
    this.goal = null;         // { x, y, yaw, state }
    this.preview = null;      // goal que se está dibujando

    this.view = { scale: 40, cx: 0, cy: 0 };   // px/m y centro en el mundo
    this.tool = 'pan';
    this.pointers = new Map();
    this.dirty = true;

    this._bindEvents();
    new ResizeObserver(() => this._resize()).observe(canvas);
    this._resize();
    const loop = () => { if (this.dirty) this._draw(); requestAnimationFrame(loop); };
    requestAnimationFrame(loop);
  }

  // ------------------------------------------------------------ datos
  setMap(msg) {
    const { width, height } = msg;
    const raw = atob(msg.data);
    const off = document.createElement('canvas');
    off.width = width; off.height = height;
    const octx = off.getContext('2d');
    const img = octx.createImageData(width, height);
    // Fila 0 del OccupancyGrid = y mínima -> abajo en la imagen
    for (let row = 0; row < height; row++) {
      const dst = (height - 1 - row) * width;
      for (let col = 0; col < width; col++) {
        let v = raw.charCodeAt(row * width + col);
        if (v > 127) v -= 256;                        // int8
        const c = v < 0 ? COLORS.unknown
          : v >= 65 ? COLORS.occupied
          : v <= 25 ? COLORS.free
          : mix(COLORS.free, COLORS.occupied, (v - 25) / 40);
        const i = (dst + col) * 4;
        img.data[i] = c[0]; img.data[i + 1] = c[1]; img.data[i + 2] = c[2]; img.data[i + 3] = 255;
      }
    }
    octx.putImageData(img, 0, 0);
    const first = !this.map;
    this.map = { info: msg, image: off };
    if (first) this.fitMap();
    this.dirty = true;
  }

  setPose(pose) { this.pose = pose; this.dirty = true; }
  setScan(scan) { this.scan = scan; this.dirty = true; }
  setGoal(goal) {
    const same = JSON.stringify(goal) === JSON.stringify(this.goal);
    if (!same) { this.goal = goal; this.dirty = true; }
  }
  setTool(tool) { this.tool = tool; this.preview = null; this.dirty = true; }

  fitMap() {
    if (!this.map) return;
    const m = this.map.info;
    const w = m.width * m.resolution, h = m.height * m.resolution;
    this.view.cx = m.origin.x + w / 2;
    this.view.cy = m.origin.y + h / 2;
    this.view.scale = 0.92 * Math.min(this.W / w, this.H / h);
    this.dirty = true;
  }

  centerRobot() {
    if (!this.pose) return;
    this.view.cx = this.pose.x; this.view.cy = this.pose.y; this.dirty = true;
  }

  // ------------------------------------------------------------ geometría
  toScreen(x, y) {
    return [this.W / 2 + (x - this.view.cx) * this.view.scale,
            this.H / 2 - (y - this.view.cy) * this.view.scale];
  }
  toWorld(sx, sy) {
    return [this.view.cx + (sx - this.W / 2) / this.view.scale,
            this.view.cy - (sy - this.H / 2) / this.view.scale];
  }

  _resize() {
    const dpr = window.devicePixelRatio || 1;
    const r = this.canvas.getBoundingClientRect();
    this.W = r.width; this.H = r.height;
    this.canvas.width = Math.round(r.width * dpr);
    this.canvas.height = Math.round(r.height * dpr);
    this.ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    this.dirty = true;
  }

  // ------------------------------------------------------------ dibujo
  _draw() {
    this.dirty = false;
    const ctx = this.ctx;
    ctx.fillStyle = COLORS.bg;
    ctx.fillRect(0, 0, this.W, this.H);

    if (this.map) {
      const m = this.map.info;
      const [sx, sy] = this.toScreen(m.origin.x, m.origin.y + m.height * m.resolution);
      const size = m.resolution * this.view.scale;
      ctx.imageSmoothingEnabled = false;
      ctx.drawImage(this.map.image, sx, sy, m.width * size, m.height * size);
    }
    this._drawScan();
    if (this.goal) this._drawGoal(this.goal, this._goalColor(this.goal.state));
    if (this.preview) this._drawGoal(this.preview, COLORS.goalPreview);
    this._drawRobot();
  }

  _drawScan() {
    const { scan, pose } = this;
    if (!scan || !pose) return;
    const ctx = this.ctx;
    ctx.fillStyle = COLORS.scan;
    const c = Math.cos(pose.yaw), s = Math.sin(pose.yaw);
    const r = Math.max(1.5, Math.min(3, this.view.scale * 0.03));
    scan.ranges.forEach((d, i) => {
      if (d === null) return;
      const a = scan.angle_min + i * scan.angle_increment;
      const lx = d * Math.cos(a), ly = d * Math.sin(a);
      const [sx, sy] = this.toScreen(pose.x + c * lx - s * ly, pose.y + s * lx + c * ly);
      ctx.fillRect(sx - r / 2, sy - r / 2, r, r);
    });
  }

  _drawRobot() {
    const p = this.pose;
    if (!p) return;
    const ctx = this.ctx;
    const [sx, sy] = this.toScreen(p.x, p.y);
    const k = this.view.scale;
    ctx.save();
    ctx.translate(sx, sy);
    ctx.rotate(-p.yaw);
    ctx.fillStyle = COLORS.robot + 'cc';
    ctx.strokeStyle = COLORS.robotEdge;
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.rect(-ROBOT_LENGTH / 2 * k, -ROBOT_WIDTH / 2 * k, ROBOT_LENGTH * k, ROBOT_WIDTH * k);
    ctx.fill(); ctx.stroke();
    // flecha de orientación (frente del robot = +x)
    const L = Math.max(14, ROBOT_LENGTH * 0.45 * k);
    ctx.strokeStyle = COLORS.robotEdge; ctx.lineWidth = 3;
    ctx.beginPath(); ctx.moveTo(0, 0); ctx.lineTo(L, 0);
    ctx.lineTo(L - 7, -6); ctx.moveTo(L, 0); ctx.lineTo(L - 7, 6); ctx.stroke();
    ctx.restore();
  }

  _drawGoal(g, color) {
    const ctx = this.ctx;
    const [sx, sy] = this.toScreen(g.x, g.y);
    const R = Math.max(7, 0.15 * this.view.scale);
    const L = Math.max(22, 0.6 * this.view.scale);
    ctx.save();
    ctx.translate(sx, sy);
    ctx.strokeStyle = color; ctx.fillStyle = color; ctx.lineWidth = 3;
    ctx.beginPath(); ctx.arc(0, 0, R, 0, 2 * Math.PI); ctx.stroke();
    ctx.beginPath(); ctx.arc(0, 0, 3, 0, 2 * Math.PI); ctx.fill();
    ctx.rotate(-(g.yaw || 0));
    ctx.beginPath(); ctx.moveTo(R, 0); ctx.lineTo(L, 0);
    ctx.lineTo(L - 8, -6); ctx.moveTo(L, 0); ctx.lineTo(L - 8, 6); ctx.stroke();
    ctx.restore();
  }

  _goalColor(state) {
    return ['aborted', 'rejected'].includes(state) ? COLORS.goalFailed : COLORS.goalActive;
  }

  // ------------------------------------------------------------ interacción
  _bindEvents() {
    const c = this.canvas;
    const pos = (e) => { const r = c.getBoundingClientRect(); return [e.clientX - r.left, e.clientY - r.top]; };

    c.addEventListener('pointerdown', (e) => {
      c.setPointerCapture(e.pointerId);
      this.pointers.set(e.pointerId, pos(e));
      if (this.tool === 'goal' && this.pointers.size === 1) {
        const [wx, wy] = this.toWorld(...pos(e));
        this.preview = { x: wx, y: wy, yaw: 0 };
        this.dirty = true;
      }
    });

    c.addEventListener('pointermove', (e) => {
      if (!this.pointers.has(e.pointerId)) return;
      const prev = this.pointers.get(e.pointerId);
      const now = pos(e);
      this.pointers.set(e.pointerId, now);

      if (this.pointers.size === 2) {            // pellizco: zoom
        const [a, b] = [...this.pointers.values()];
        const other = a === now ? b : a;
        const d0 = Math.hypot(prev[0] - other[0], prev[1] - other[1]);
        const d1 = Math.hypot(now[0] - other[0], now[1] - other[1]);
        if (d0 > 0) this._zoomAt((now[0] + other[0]) / 2, (now[1] + other[1]) / 2, d1 / d0);
        this.preview = null;
        return;
      }
      if (this.tool === 'goal' && this.preview) {  // arrastrar: orientación
        const [wx, wy] = this.toWorld(...now);
        const dx = wx - this.preview.x, dy = wy - this.preview.y;
        if (Math.hypot(dx, dy) * this.view.scale > 8) this.preview.yaw = Math.atan2(dy, dx);
        this.dirty = true;
      } else if (this.tool === 'pan') {
        this.view.cx -= (now[0] - prev[0]) / this.view.scale;
        this.view.cy += (now[1] - prev[1]) / this.view.scale;
        this.dirty = true;
      }
    });

    const end = (e) => {
      if (!this.pointers.has(e.pointerId)) return;
      this.pointers.delete(e.pointerId);
      if (this.tool === 'goal' && this.preview && this.pointers.size === 0) {
        const g = this.preview;
        this.preview = null;
        this.dirty = true;
        this.onGoal(g);
      }
    };
    c.addEventListener('pointerup', end);
    c.addEventListener('pointercancel', (e) => { this.preview = null; end(e); });

    c.addEventListener('wheel', (e) => {
      e.preventDefault();
      const [x, y] = pos(e);
      this._zoomAt(x, y, Math.exp(-e.deltaY * 0.0015));
    }, { passive: false });
  }

  _zoomAt(sx, sy, factor) {
    const [wx, wy] = this.toWorld(sx, sy);
    this.view.scale = Math.min(400, Math.max(5, this.view.scale * factor));
    const [nx, ny] = this.toWorld(sx, sy);
    this.view.cx += wx - nx; this.view.cy += wy - ny;
    this.dirty = true;
  }
}

function mix(a, b, t) {
  return [0, 1, 2].map((i) => Math.round(a[i] + (b[i] - a[i]) * t));
}
