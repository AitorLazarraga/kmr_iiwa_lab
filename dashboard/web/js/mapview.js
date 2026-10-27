// Vista 2D del mapa: mapa de ocupación, robot, scan, goal, puntos y rutas.
// Coordenadas: todo en el frame "map" (metros, y hacia arriba).
//
// Herramientas: 'pan' (mover la vista), 'goal' y 'point' (pulsar y arrastrar
// para orientar) y 'work' (arrastrar sobre la cara de una pared, de izquierda a
// derecha vista desde donde se colocará el robot -> onWork([[x1,y1],[x2,y2]])).

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
  point: '#c792ea',
  pointPreview: '#e2c2ff',
  route: '#3d8bfd',
  routeDone: '#6b7480',
  label: '#1b1d20',
  work: '#4fd1a5',          // puntos de trabajo
  workWall: '#ff9f43',      // cara de pared marcada
  screw: '#ffd166',
  workBad: '#e5484d',
};
const OCCUPIED = 65;          // umbral de celda ocupada (igual que el dibujo)

export class MapView {
  constructor(canvas, { onGoal, onPoint, onWork }) {
    this.canvas = canvas;
    this.ctx = canvas.getContext('2d');
    this.onGoal = onGoal;
    this.onPoint = onPoint;      // herramienta "Punto de movimiento"
    this.onWork = onWork;        // herramienta "Punto de trabajo" (marcar pared)
    this.wallDraft = null;       // { p1, p2 } mientras se arrastra sobre la pared
    this.workPreview = null;     // { wall, x, y, yaw, screws, bad } del editor de puntos de trabajo
    this.points = [];            // puntos guardados del mapa activo
    this.route = null;           // ruta en ejecución o terminada (state.nav.route)
    this.highlight = null;       // id de punto resaltado desde el panel

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
    // Valores crudos (fila 0 = y mínima) para comprobar colisiones de una pose
    const grid = new Int8Array(width * height);
    for (let i = 0; i < grid.length; i++) grid[i] = raw.charCodeAt(i);
    const first = !this.map;
    this.map = { info: msg, image: off, grid };
    if (first) this.fitMap();
    this.dirty = true;
  }

  setPose(pose) { this.pose = pose; this.dirty = true; }
  setScan(scan) { this.scan = scan; this.dirty = true; }
  setGoal(goal) {
    const same = JSON.stringify(goal) === JSON.stringify(this.goal);
    if (!same) { this.goal = goal; this.dirty = true; }
  }
  setTool(tool) { this.tool = tool; this.preview = null; this.wallDraft = null; this.dirty = true; }
  setWorkPreview(preview) { this.workPreview = preview; this.dirty = true; }

  /**
   * Celdas ocupadas del mapa bajo la huella del robot en una pose (0 = libre).
   * Celdas desconocidas no cuentan. Devuelve null si no hay mapa.
   */
  footprintHits(x, y, yaw, margin = 0.05) {
    if (!this.map) return null;
    const m = this.map.info;
    const res = m.resolution;
    const c = Math.cos(yaw), s = Math.sin(yaw);
    const hl = ROBOT_LENGTH / 2 + margin, hw = ROBOT_WIDTH / 2 + margin;
    const step = res / 2;
    const seen = new Set();
    let hits = 0;
    for (let a = -hl; a <= hl; a += step) {
      for (let b = -hw; b <= hw; b += step) {
        const wx = x + c * a - s * b, wy = y + s * a + c * b;
        const col = Math.floor((wx - m.origin.x) / res);
        const row = Math.floor((wy - m.origin.y) / res);
        if (col < 0 || row < 0 || col >= m.width || row >= m.height) continue;
        const k = row * m.width + col;
        if (seen.has(k)) continue;
        seen.add(k);
        if (this.map.grid[k] >= OCCUPIED) hits++;
      }
    }
    return hits;
  }
  setPoints(points) { this.points = points || []; this.dirty = true; }
  setRoute(route) {
    const same = JSON.stringify(route) === JSON.stringify(this.route);
    if (!same) { this.route = route; this.dirty = true; }
  }
  setHighlight(pointId) { this.highlight = pointId; this.dirty = true; }

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
    this._drawPoints();
    this._drawRoute();
    if (this.workPreview) this._drawWorkPreview(this.workPreview);
    if (this.wallDraft) this._drawWallDraft(this.wallDraft);
    if (this.goal) this._drawGoal(this.goal, this._goalColor(this.goal.state));
    if (this.preview) {
      this._drawGoal(this.preview, this.tool === 'point' ? COLORS.pointPreview : COLORS.goalPreview);
    }
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

  _drawPoints() {
    const ctx = this.ctx;
    const showLabels = this.view.scale > 18;
    ctx.font = '12px system-ui, sans-serif';
    for (const p of this.points) {
      const work = p.kind === 'work';
      const hl = p.id === this.highlight;
      if (work && p.work) this._drawWall(p.work.wall, p.screws, hl ? 4 : 3, hl ? 1 : 0.75);
      const [sx, sy] = this.toScreen(p.x, p.y);
      const r = hl ? 9 : 6;
      const color = work ? COLORS.work : COLORS.point;
      ctx.save();
      ctx.translate(sx, sy);
      ctx.fillStyle = color;
      ctx.strokeStyle = '#fff';
      ctx.lineWidth = hl ? 3 : 1.5;
      ctx.beginPath();
      if (work) {                                        // cuadrado: punto de trabajo
        ctx.rect(-r * 0.85, -r * 0.85, r * 1.7, r * 1.7);
      } else {                                           // rombo: punto de movimiento
        ctx.moveTo(0, -r); ctx.lineTo(r, 0); ctx.lineTo(0, r); ctx.lineTo(-r, 0); ctx.closePath();
      }
      ctx.fill(); ctx.stroke();
      ctx.rotate(-p.yaw);                                // orientación
      ctx.strokeStyle = color; ctx.lineWidth = 2;
      ctx.beginPath(); ctx.moveTo(r, 0); ctx.lineTo(r + 9, 0); ctx.stroke();
      ctx.restore();
      if (showLabels || p.id === this.highlight) {
        ctx.fillStyle = '#fff';
        ctx.strokeStyle = COLORS.label; ctx.lineWidth = 3;
        ctx.strokeText(p.name, sx + 10, sy - 8);
        ctx.fillText(p.name, sx + 10, sy - 8);
      }
    }
  }

  /** Cara de pared marcada + tornillos (puntos de trabajo y vista previa). */
  _drawWall(wall, screws, width = 3, alpha = 1, color = COLORS.workWall) {
    const ctx = this.ctx;
    const [a, b] = wall.map(([x, y]) => this.toScreen(x, y));
    ctx.save();
    ctx.globalAlpha = alpha;
    ctx.strokeStyle = color; ctx.lineWidth = width; ctx.lineCap = 'round';
    ctx.beginPath(); ctx.moveTo(...a); ctx.lineTo(...b); ctx.stroke();
    if (screws) {
      const r = Math.max(2, Math.min(4, 0.03 * this.view.scale));
      ctx.fillStyle = COLORS.screw; ctx.strokeStyle = COLORS.label; ctx.lineWidth = 1;
      const seen = new Set();                  // varias filas caen en el mismo punto en planta
      for (const s of screws) {
        const key = `${s.x},${s.y}`;
        if (seen.has(key)) continue;
        seen.add(key);
        const [sx, sy] = this.toScreen(s.x, s.y);
        ctx.beginPath(); ctx.arc(sx, sy, r, 0, 2 * Math.PI); ctx.fill(); ctx.stroke();
      }
    }
    ctx.restore();
  }

  /** Huella del robot (rectángulo) en una pose, solo contorno. */
  _drawFootprint(x, y, yaw, color, dashed = true) {
    const ctx = this.ctx;
    const [sx, sy] = this.toScreen(x, y);
    const k = this.view.scale;
    ctx.save();
    ctx.translate(sx, sy);
    ctx.rotate(-yaw);
    ctx.strokeStyle = color; ctx.lineWidth = 2;
    if (dashed) ctx.setLineDash([6, 4]);
    ctx.strokeRect(-ROBOT_LENGTH / 2 * k, -ROBOT_WIDTH / 2 * k, ROBOT_LENGTH * k, ROBOT_WIDTH * k);
    ctx.setLineDash([]);
    const L = Math.max(14, ROBOT_LENGTH * 0.45 * k);
    ctx.beginPath(); ctx.moveTo(0, 0); ctx.lineTo(L, 0);
    ctx.lineTo(L - 7, -6); ctx.moveTo(L, 0); ctx.lineTo(L - 7, 6); ctx.stroke();
    ctx.restore();
  }

  _drawWorkPreview(w) {
    const color = w.bad ? COLORS.workBad : COLORS.work;
    this._drawWall(w.wall, w.screws, 4, 1, w.error ? COLORS.workBad : COLORS.workWall);
    if (!w.error && w.x !== undefined) this._drawFootprint(w.x, w.y, w.yaw, color);
  }

  /** Pared que se está marcando: línea + "1"/"2" + flecha hacia el lado del robot. */
  _drawWallDraft(d) {
    const ctx = this.ctx;
    const [a, b] = [d.p1, d.p2].map(([x, y]) => this.toScreen(x, y));
    ctx.save();
    ctx.strokeStyle = COLORS.workWall; ctx.lineWidth = 3; ctx.setLineDash([8, 5]);
    ctx.beginPath(); ctx.moveTo(...a); ctx.lineTo(...b); ctx.stroke();
    ctx.setLineDash([]);
    const len = Math.hypot(b[0] - a[0], b[1] - a[1]);
    if (len > 12) {
      // El robot queda a la derecha de 1->2 en el mundo: (t.y, -t.x). En pantalla
      // la y está invertida, así que en píxeles es (-uy, ux).
      const ux = (b[0] - a[0]) / len, uy = (b[1] - a[1]) / len;
      const nx = -uy, ny = ux;
      const mx = (a[0] + b[0]) / 2, my = (a[1] + b[1]) / 2;
      const L = 26;
      ctx.strokeStyle = COLORS.work; ctx.lineWidth = 2.5;
      ctx.beginPath(); ctx.moveTo(mx + nx * 4, my + ny * 4); ctx.lineTo(mx + nx * L, my + ny * L);
      ctx.stroke();
      ctx.fillStyle = COLORS.work; ctx.font = 'bold 11px system-ui, sans-serif';
      ctx.fillText('robot', mx + nx * (L + 4) - 14, my + ny * (L + 4) + 4);
    }
    ctx.font = 'bold 12px system-ui, sans-serif';
    for (const [[sx, sy], label] of [[a, '1'], [b, '2']]) {
      ctx.fillStyle = COLORS.workWall;
      ctx.beginPath(); ctx.arc(sx, sy, 8, 0, 2 * Math.PI); ctx.fill();
      ctx.fillStyle = COLORS.label; ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
      ctx.fillText(label, sx, sy + 0.5);
    }
    ctx.restore();
  }

  _drawRoute() {
    const r = this.route;
    if (!r || !r.stops || !r.stops.length) return;
    const ctx = this.ctx;
    const pts = r.stops.map((s) => this.toScreen(s.x, s.y));
    const active = r.phase !== 'done';

    // Parada i alcanzada: ya pasada, o es la actual y se está esperando en ella
    const reached = (i) => !active || i < r.current || (i === r.current && r.phase === 'waiting');

    // tramos: recorridos en gris, pendientes en azul discontinuo
    ctx.lineWidth = 3;
    for (let i = 1; i < pts.length; i++) {
      const done = reached(i);
      ctx.strokeStyle = done ? COLORS.routeDone : COLORS.route;
      ctx.setLineDash(done ? [] : [8, 6]);
      ctx.beginPath(); ctx.moveTo(...pts[i - 1]); ctx.lineTo(...pts[i]); ctx.stroke();
    }
    ctx.setLineDash([]);

    // paradas numeradas
    ctx.font = 'bold 11px system-ui, sans-serif';
    ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
    pts.forEach(([sx, sy], i) => {
      const isCurrent = active && i === r.current;
      const visited = reached(i);
      ctx.fillStyle = isCurrent ? COLORS.route : visited ? COLORS.routeDone : '#fff';
      ctx.strokeStyle = COLORS.route; ctx.lineWidth = 2;
      ctx.beginPath(); ctx.arc(sx, sy, isCurrent ? 11 : 9, 0, 2 * Math.PI); ctx.fill(); ctx.stroke();
      ctx.fillStyle = isCurrent || visited ? '#fff' : COLORS.route;
      ctx.fillText(String(i + 1), sx, sy + 0.5);
    });
    ctx.textAlign = 'start'; ctx.textBaseline = 'alphabetic';
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
      if ((this.tool === 'goal' || this.tool === 'point') && this.pointers.size === 1) {
        const [wx, wy] = this.toWorld(...pos(e));
        this.preview = { x: wx, y: wy, yaw: 0 };
        this.dirty = true;
      } else if (this.tool === 'work' && this.pointers.size === 1) {
        const p = this.toWorld(...pos(e));
        this.wallDraft = { p1: p, p2: p };
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
        this.wallDraft = null;
        return;
      }
      if (this.tool === 'work' && this.wallDraft) {           // arrastrar: segundo extremo
        this.wallDraft.p2 = this.toWorld(...now);
        this.dirty = true;
        return;
      }
      if ((this.tool === 'goal' || this.tool === 'point') && this.preview) {  // arrastrar: orientación
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
      if ((this.tool === 'goal' || this.tool === 'point') && this.preview && this.pointers.size === 0) {
        const g = this.preview;
        this.preview = null;
        this.dirty = true;
        (this.tool === 'goal' ? this.onGoal : this.onPoint)(g);
      } else if (this.tool === 'work' && this.wallDraft && this.pointers.size === 0) {
        const { p1, p2 } = this.wallDraft;
        this.wallDraft = null;
        this.dirty = true;
        // Ignora toques sin arrastre (< 10 px en pantalla)
        if (Math.hypot(p2[0] - p1[0], p2[1] - p1[1]) * this.view.scale >= 10) {
          this.onWork([[p1[0], p1[1]], [p2[0], p2[1]]]);
        }
      }
    };
    c.addEventListener('pointerup', end);
    c.addEventListener('pointercancel', (e) => { this.preview = null; this.wallDraft = null; end(e); });

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
