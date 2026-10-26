// Panel "Puntos y rutas": lista de puntos, lista de rutas, editor de rutas.
// Los datos se piden por la API REST y se recargan cuando el gateway avisa
// con un evento 'points_changed' (lo hace main.js llamando a reload()).

import { api } from './api.js';

const fmt = (v) => Number(v).toFixed(2);
const deg = (rad) => Math.round(rad * 180 / Math.PI);

export class PointsPanel {
  /**
   * @param {object} deps
   *   map       MapView (para dibujar y resaltar puntos)
   *   send      (msg) => bool, envío por WebSocket
   *   notify    (level, text) => void, barra de avisos
   *   getPose   () => {x, y, yaw} | null, pose actual del robot
   */
  constructor({ map, send, notify, getPose }) {
    this.map = map;
    this.send = send;
    this.notify = notify;
    this.getPose = getPose;
    this.points = [];
    this.routes = [];
    this.nav = null;            // state.nav, para marcar la ruta en curso
    this.editing = null;        // { id|null, name, stops: [{point_id, wait_s}] }

    this.$ = (id) => document.getElementById(id);
    this._bind();
  }

  // ------------------------------------------------------------ datos
  async reload() {
    const [p, r] = await Promise.all([api('/api/points'), api('/api/routes')]);
    if (!p.ok || !r.ok) {
      if (p.status !== 401 && r.status !== 401) {
        this.notify('warning', `No se pudieron cargar puntos/rutas: ${p.error || r.error}`);
      }
      return;
    }
    this.points = p.body;
    this.routes = r.body;
    this.map.setPoints(this.points);
    this._renderPoints();
    this._renderRoutes();
    if (this.editing) this._renderEditor();
  }

  setNav(nav) {
    const runningId = nav && nav.mode === 'route' && ['sending', 'active'].includes(nav.state)
      ? nav.route.id : null;
    const prev = this.nav && this.nav.mode === 'route' && ['sending', 'active'].includes(this.nav.state)
      ? this.nav.route.id : null;
    this.nav = nav;
    if (runningId !== prev) this._renderRoutes();
  }

  pointName(id) {
    const p = this.points.find((x) => x.id === id);
    return p ? p.name : `#${id}`;
  }

  // ------------------------------------------------------------ acciones de puntos
  async createPoint(x, y, yaw) {
    const name = prompt('Nombre del punto:');
    if (name === null) return;
    const r = await api('/api/points', {
      method: 'POST', body: JSON.stringify({ name: name.trim(), x, y, yaw }),
    });
    if (!r.ok) this.notify('warning', `No se pudo crear el punto: ${r.error}`);
  }

  async saveCurrentPose() {
    const p = this.getPose();
    if (!p) { this.notify('warning', 'No hay pose del robot todavía'); return; }
    await this.createPoint(p.x, p.y, p.yaw);
  }

  async renamePoint(pt) {
    const name = prompt('Nuevo nombre:', pt.name);
    if (name === null || name.trim() === pt.name) return;
    const r = await api(`/api/points/${pt.id}`, {
      method: 'PATCH', body: JSON.stringify({ name: name.trim() }),
    });
    if (!r.ok) this.notify('warning', `No se pudo renombrar: ${r.error}`);
  }

  async deletePoint(pt) {
    const used = this.routes.filter((r) => r.stops.some((s) => s.point_id === pt.id));
    const warn = used.length
      ? `\n\nSe quitará de las rutas: ${used.map((r) => r.name).join(', ')}` : '';
    if (!confirm(`¿Borrar el punto «${pt.name}»?${warn}`)) return;
    const r = await api(`/api/points/${pt.id}`, { method: 'DELETE' });
    if (!r.ok) this.notify('warning', `No se pudo borrar: ${r.error}`);
  }

  goToPoint(pt) {
    if (!this.send({ type: 'goal', x: pt.x, y: pt.y, yaw: pt.yaw })) {
      this.notify('warning', 'Sin conexión: el goal no se ha enviado');
    }
  }

  // ------------------------------------------------------------ acciones de rutas
  runRoute(route) {
    if (!this.send({ type: 'run_route', route_id: route.id })) {
      this.notify('warning', 'Sin conexión: la ruta no se ha enviado');
    }
  }

  async deleteRoute(route) {
    if (!confirm(`¿Borrar la ruta «${route.name}»? (los puntos no se borran)`)) return;
    const r = await api(`/api/routes/${route.id}`, { method: 'DELETE' });
    if (!r.ok) this.notify('warning', `No se pudo borrar: ${r.error}`);
  }

  editRoute(route) {
    this.editing = route
      ? { id: route.id, name: route.name,
          stops: route.stops.map((s) => ({ point_id: s.point_id, wait_s: s.wait_s })) }
      : { id: null, name: '', stops: [] };
    this._showTab('routes');
    this._renderEditor();
  }

  async saveRoute() {
    const e = this.editing;
    e.name = this.$('re-name').value.trim();
    if (!e.name) { this.notify('warning', 'La ruta necesita un nombre'); return; }
    if (!e.stops.length) { this.notify('warning', 'La ruta necesita al menos una parada'); return; }
    const body = JSON.stringify({ name: e.name, stops: e.stops });
    const r = e.id === null
      ? await api('/api/routes', { method: 'POST', body })
      : await api(`/api/routes/${e.id}`, { method: 'PATCH', body });
    if (!r.ok) { this.notify('warning', `No se pudo guardar la ruta: ${r.error}`); return; }
    this.editing = null;
    this._renderEditor();
  }

  // ------------------------------------------------------------ renderizado
  _renderPoints() {
    const ul = this.$('points-list');
    ul.replaceChildren();
    if (!this.points.length) {
      ul.append(li('pp-empty', 'Sin puntos en este mapa.'));
      return;
    }
    for (const pt of this.points) {
      const item = li('pp-item');
      item.append(
        info(pt.name, `x ${fmt(pt.x)} · y ${fmt(pt.y)} · θ ${deg(pt.yaw)}°`),
        actions(
          button('Ir', 'needs-control', () => this.goToPoint(pt), 'Navegar a este punto'),
          button('✎', 'needs-operator', () => this.renamePoint(pt), 'Renombrar'),
          button('✕', 'needs-operator danger-text', () => this.deletePoint(pt), 'Borrar'),
        ),
      );
      item.addEventListener('pointerenter', () => this.map.setHighlight(pt.id));
      item.addEventListener('pointerleave', () => this.map.setHighlight(null));
      ul.append(item);
    }
  }

  _renderRoutes() {
    const ul = this.$('routes-list');
    ul.replaceChildren();
    if (!this.routes.length) {
      ul.append(li('pp-empty', 'Sin rutas en este mapa.'));
      return;
    }
    const runningId = this.nav && this.nav.mode === 'route'
      && ['sending', 'active'].includes(this.nav.state) ? this.nav.route.id : null;
    for (const route of this.routes) {
      const n = route.stops.length;
      const sub = route.executable
        ? route.stops.map((s) => s.wait_s > 0 ? `${s.name} (${s.wait_s}s)` : s.name).join(' → ')
        : 'VACÍA: sin paradas';
      const item = li('pp-item' + (route.id === runningId ? ' running' : '')
        + (route.executable ? '' : ' empty'));
      const run = button('▶', 'needs-control', () => this.runRoute(route), 'Ejecutar');
      run.disabled = !route.executable;
      item.append(
        info(`${route.name} · ${n} parada${n === 1 ? '' : 's'}`, sub),
        actions(
          run,
          button('✎', 'needs-operator', () => this.editRoute(route), 'Editar'),
          button('✕', 'needs-operator danger-text', () => this.deleteRoute(route), 'Borrar'),
        ),
      );
      ul.append(item);
    }
  }

  _renderEditor() {
    const box = this.$('route-editor');
    const e = this.editing;
    box.classList.toggle('hidden', !e);
    this.$('routes-list').classList.toggle('hidden', !!e);
    this.$('btn-new-route').classList.toggle('hidden', !!e);
    if (!e) return;

    this.$('re-title').textContent = e.id === null ? 'Nueva ruta' : 'Editar ruta';
    if (document.activeElement !== this.$('re-name')) this.$('re-name').value = e.name;

    const ol = this.$('re-stops');
    ol.replaceChildren();
    if (!e.stops.length) ol.append(li('pp-empty', 'Añade paradas con el selector de abajo.'));
    e.stops.forEach((s, i) => {
      const item = li('re-stop');
      const wait = document.createElement('input');
      Object.assign(wait, { type: 'number', min: 0, max: 600, step: 0.5, value: s.wait_s,
        title: 'Espera en la parada (s)' });
      wait.addEventListener('change', () => {
        const v = Math.min(600, Math.max(0, parseFloat(wait.value) || 0));
        s.wait_s = v; wait.value = v;
      });
      const name = document.createElement('span');
      name.className = 're-name';
      name.textContent = `${i + 1}. ${this.pointName(s.point_id)}`;
      const unit = document.createElement('small');
      unit.textContent = 's';
      item.append(
        name, wait, unit,
        button('↑', '', () => this._moveStop(i, -1), 'Subir'),
        button('↓', '', () => this._moveStop(i, +1), 'Bajar'),
        button('✕', 'danger-text', () => { e.stops.splice(i, 1); this._renderEditor(); }, 'Quitar'),
      );
      item.querySelector('button').disabled = i === 0;
      item.querySelectorAll('button')[1].disabled = i === e.stops.length - 1;
      ol.append(item);
    });

    const sel = this.$('re-point');
    sel.replaceChildren(...this.points.map((p) => {
      const o = document.createElement('option');
      o.value = p.id; o.textContent = p.name;
      return o;
    }));
    this.$('re-add-btn').disabled = !this.points.length;
  }

  _moveStop(i, d) {
    const s = this.editing.stops;
    const j = i + d;
    if (j < 0 || j >= s.length) return;
    [s[i], s[j]] = [s[j], s[i]];
    this._renderEditor();
  }

  _showTab(tab) {
    document.querySelectorAll('[data-pp-tab]').forEach((b) =>
      b.classList.toggle('active', b.dataset.ppTab === tab));
    this.$('pp-points').classList.toggle('hidden', tab !== 'points');
    this.$('pp-routes').classList.toggle('hidden', tab !== 'routes');
  }

  _bind() {
    document.querySelectorAll('[data-pp-tab]').forEach((b) =>
      b.addEventListener('click', () => this._showTab(b.dataset.ppTab)));
    this.$('btn-save-pose').addEventListener('click', () => this.saveCurrentPose());
    this.$('btn-new-route').addEventListener('click', () => this.editRoute(null));
    this.$('re-name').addEventListener('input', (ev) => { if (this.editing) this.editing.name = ev.target.value; });
    this.$('re-add-btn').addEventListener('click', () => {
      const id = parseInt(this.$('re-point').value, 10);
      if (!Number.isNaN(id)) { this.editing.stops.push({ point_id: id, wait_s: 0 }); this._renderEditor(); }
    });
    this.$('re-save').addEventListener('click', () => this.saveRoute());
    this.$('re-cancel').addEventListener('click', () => { this.editing = null; this._renderEditor(); });
  }
}

// ------------------------------------------------------------ utilidades DOM
function li(cls, text) {
  const el = document.createElement('li');
  el.className = cls;
  if (text) el.textContent = text;
  return el;
}

function info(title, sub) {
  const box = document.createElement('div');
  box.className = 'pp-info';
  const b = document.createElement('b');
  b.textContent = title;
  const s = document.createElement('small');
  s.textContent = sub;
  box.append(b, s);
  return box;
}

function actions(...buttons) {
  const box = document.createElement('div');
  box.className = 'pp-actions-row';
  box.append(...buttons);
  return box;
}

function button(text, cls, onClick, title = '') {
  const b = document.createElement('button');
  b.className = `ghost small ${cls}`.trim();
  b.textContent = text;
  b.title = title;
  b.addEventListener('click', (e) => { e.stopPropagation(); onClick(); });
  return b;
}
