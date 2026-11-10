// Pestaña «Mapas» (2.6): modo y mapa con los que está arrancado el robot,
// «Guardar mapa» (solo en modo mapeo) y la lista de mapas guardados con el
// comando para arrancar con cada uno.
//
// El gateway no cambia de mapa ni de modo (no tiene acceso a Docker): se
// reinicia todo con start_sim.sh en el PC del robot. Por eso aquí solo se
// muestra el comando, listo para copiar.
//
// Sobrescribir un mapa que ya existe: solo un administrador y con dos
// confirmaciones; el gateway comprueba el rol y exige overwrite=true.

import { api } from './api.js';

const NAME_RE = /^[a-z0-9][a-z0-9_-]{0,39}$/;
const MODE_TEXT = { mapping: 'mapeo', localization: 'localización' };
const STEP_TEXT = { serialize: 'grafo de poses', image: 'imagen .pgm' };

export class MapsPanel {
  /**
   * @param {object} deps
   *   send       (msg) => bool, envío por WebSocket
   *   notify     (level, text) => void, barra de avisos
   *   getRole    () => 'observer' | 'operator' | 'admin'
   */
  constructor({ send, notify, getRole }) {
    this.send = send;
    this.notify = notify;
    this.getRole = getRole;
    this.data = null;          // respuesta de GET /api/maps
    this.slam = null;          // state.slam: {mode, map, saving, last}
    this.selected = null;      // nombre del mapa elegido en la lista
    this._slamKey = null;
    this.$ = (id) => document.getElementById(id);
    this._bind();
  }

  // ------------------------------------------------------------ datos
  async reload() {
    const r = await api('/api/maps');
    if (!r.ok) {
      if (r.status !== 401) this.notify('warning', `No se pudo cargar la lista de mapas: ${r.error}`);
      return;
    }
    this.data = r.body;
    this._render();
  }

  /** state.slam (cada 100 ms): solo se redibuja si cambia algo. */
  setSlam(slam) {
    const key = JSON.stringify(slam ? { ...slam, saving: slam.saving && { ...slam.saving, elapsed: 0 } } : null);
    const savingChanged = JSON.stringify(slam?.saving) !== JSON.stringify(this.slam?.saving);
    this.slam = slam;
    if (key !== this._slamKey) {
      this._slamKey = key;
      this._renderStatus();
      this._renderSave();
    } else if (savingChanged) {
      this._renderSaveMsg();          // solo el contador de segundos
    }
  }

  get mode() { return this.slam?.mode || this.data?.mode || null; }
  get active() { return this.slam?.map || this.data?.active || null; }

  // ------------------------------------------------------------ guardar
  save() {
    const input = this.$('mp-name');
    const name = input.value.trim();
    if (!NAME_RE.test(name)) {
      this.notify('warning', 'Nombre de mapa no válido: minúsculas, números, «-» y «_», empezando por letra o número');
      input.focus();
      return;
    }
    const existing = this.data?.maps?.find((m) => m.name === name);
    let overwrite = false;
    if (existing) {
      if (this.getRole() !== 'admin') {
        this.notify('warning', `El mapa «${name}» ya existe: elige otro nombre (solo un administrador puede sobrescribirlo)`);
        return;
      }
      const what = `${existing.points} punto${existing.points === 1 ? '' : 's'} y ` +
        `${existing.routes} ruta${existing.routes === 1 ? '' : 's'}`;
      if (!confirm(`El mapa «${name}» ya existe (guardado el ${dateText(existing.modified)}, ${what}).\n\n` +
        '¿Sobrescribirlo con el mapa que se está construyendo?')) return;
      if (!confirm(`SEGUNDA CONFIRMACIÓN\n\nEl mapa «${name}» anterior se perderá. Si el nuevo no coincide ` +
        `con él, sus ${what} dejarán de estar donde deben.\n\n¿Sobrescribir «${name}» definitivamente?`)) return;
      overwrite = true;
    }
    if (!this.send({ type: 'map_save', name, overwrite })) {
      this.notify('warning', 'Sin conexión: el mapa no se ha guardado');
    }
  }

  // ------------------------------------------------------------ dibujo
  _render() {
    this._renderStatus();
    this._renderSave();
    this._renderList();
    this._renderCmd();
  }

  _renderStatus() {
    const el = this.$('mp-status');
    if (!this.mode) { el.textContent = '—'; return; }
    el.innerHTML = `Arrancado en modo <b>${MODE_TEXT[this.mode] || esc(this.mode)}</b> ` +
      `con el mapa <b>${esc(this.active || '?')}</b>` +
      (this.mode === 'mapping'
        ? '<br><small>Mueve el robot por toda la zona y guarda el mapa antes de parar: '
          + 'lo que no se guarde se pierde al reiniciar.</small>'
        : '');
    el.className = 'mp-status' + (this.mode === 'mapping' ? ' mapping' : '');
    const cmd = this.data?.start_cmd || './start_sim.sh';
    this.$('mp-new').innerHTML = `Mapa nuevo: <code>${esc(cmd)} mapping &lt;nombre&gt;</code>`;
  }

  _renderSave() {
    const mapping = this.mode === 'mapping';
    this.$('mp-save').classList.toggle('hidden', !mapping);
    if (!mapping) return;
    const input = this.$('mp-name');
    if (!input.value && this.active) input.value = this.active;    // por defecto, el nombre del arranque
    this._renderSaveMsg();
  }

  _renderSaveMsg() {
    const msg = this.$('mp-msg');
    const s = this.slam?.saving;
    const last = this.slam?.last;
    this.$('mp-save-btn').disabled = !!s;
    if (s) {
      msg.textContent = `${s.overwrite ? 'Sobrescribiendo' : 'Guardando'} «${s.name}» · ` +
        `${STEP_TEXT[s.step] || s.step} · ${s.elapsed.toFixed(0)} s`;
      msg.className = 'mp-msg run';
    } else if (last) {
      msg.textContent = `${last.message} (${timeText(last.time)})`;
      msg.className = 'mp-msg ' + (last.ok ? 'ok' : 'bad');
    } else {
      msg.textContent = '';
      msg.className = 'mp-msg';
    }
  }

  _renderList() {
    const ul = this.$('maps-list');
    ul.replaceChildren();
    const d = this.data;
    if (!d) return;
    if (!d.dir_ok) {
      ul.append(li('pp-empty', 'No se encuentra la carpeta de mapas del gateway (MAPS_DIR)'));
      return;
    }
    if (!d.maps.length) {
      ul.append(li('pp-empty', 'Todavía no hay mapas guardados'));
      return;
    }
    for (const m of d.maps) {
      const item = document.createElement('li');
      item.className = 'pp-item mp-item' + (m.name === this.selected ? ' selected' : '') +
        (m.active ? ' active' : '') + (m.complete ? '' : ' empty');
      const info = document.createElement('div');
      info.className = 'pp-info';
      const b = document.createElement('b');
      b.textContent = m.name;
      if (m.active) {
        const tag = document.createElement('span');
        tag.className = 'mp-tag';
        tag.textContent = 'en uso';
        b.append(' ', tag);
      }
      const small = document.createElement('small');
      small.textContent = m.complete
        ? `${dateText(m.modified)} · ${mb(m.size)} · ${m.points} punto${m.points === 1 ? '' : 's'} · `
          + `${m.routes} ruta${m.routes === 1 ? '' : 's'}`
        : 'INCOMPLETO: falta el .data, no se puede cargar';
      info.append(b, small);
      item.append(info);
      item.addEventListener('click', () => {
        this.selected = this.selected === m.name ? null : m.name;
        this._renderList();
        this._renderCmd();
      });
      ul.append(item);
    }
  }

  _renderCmd() {
    const box = this.$('mp-cmd');
    const m = this.data?.maps?.find((x) => x.name === this.selected);
    box.classList.toggle('hidden', !m);
    if (!m) return;
    const cmd = `${this.data.start_cmd || './start_sim.sh'} localization ${m.name}`;
    this.$('mp-cmd-text').textContent = cmd;
    const inUse = m.active && this.mode === 'localization';
    this.$('mp-cmd-title').textContent = !m.complete
      ? `«${m.name}» está incompleto: no arrancará`
      : inUse ? `«${m.name}» ya está en uso. Para reiniciar con él:`
        : `Para arrancar con «${m.name}» (localización):`;
  }

  async copy() {
    const text = this.$('mp-cmd-text').textContent;
    let ok = false;
    // navigator.clipboard solo existe en contextos seguros (https o localhost);
    // desde otro equipo por http, o si el navegador lo deniega, el método antiguo
    try {
      if (navigator.clipboard && window.isSecureContext) {
        await navigator.clipboard.writeText(text);
        ok = true;
      }
    } catch { ok = false; }
    if (!ok) {
      try {
        const ta = document.createElement('textarea');
        ta.value = text;
        ta.style.position = 'fixed'; ta.style.opacity = '0';
        document.body.append(ta);
        ta.select();
        ok = document.execCommand('copy');
        ta.remove();
      } catch { ok = false; }
    }
    if (ok) {
      this.notify('info', 'Comando copiado');
    } else {                           // al menos dejarlo seleccionado para Ctrl+C
      const sel = window.getSelection();
      sel.removeAllRanges();
      const range = document.createRange();
      range.selectNodeContents(this.$('mp-cmd-text'));
      sel.addRange(range);
      this.notify('warning', 'No se pudo copiar automáticamente: el comando queda seleccionado (Ctrl+C)');
    }
  }

  _bind() {
    this.$('mp-save-btn').addEventListener('click', () => this.save());
    this.$('mp-name').addEventListener('keydown', (e) => { if (e.key === 'Enter') this.save(); });
    this.$('mp-copy').addEventListener('click', () => this.copy());
  }
}

function li(cls, text) {
  const el = document.createElement('li');
  el.className = cls;
  el.textContent = text;
  return el;
}

function esc(s) {
  return String(s).replace(/[&<>"']/g, (ch) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[ch]));
}

const pad = (n) => String(n).padStart(2, '0');
function dateText(t) {
  const d = new Date(t * 1000);
  return `${pad(d.getDate())}/${pad(d.getMonth() + 1)}/${d.getFullYear()} ${pad(d.getHours())}:${pad(d.getMinutes())}`;
}
function timeText(t) {
  const d = new Date(t * 1000);
  return `${pad(d.getHours())}:${pad(d.getMinutes())}`;
}
function mb(bytes) {
  return bytes >= 1e6 ? `${(bytes / 1e6).toFixed(1)} MB` : `${Math.max(1, Math.round(bytes / 1e3))} kB`;
}
