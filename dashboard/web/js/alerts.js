// Barra de emergencias y avisos.
//  - Banner: condiciones que siguen activas (sin conexión, ROS perdido...).
//    Se muestra la más grave y desaparece sola cuando la condición se resuelve.
//  - Lista: historial de avisos puntuales (eventos del gateway y locales).

const MAX_ALERTS = 40;
const SEVERITY = { emergency: 2, warning: 1, info: 0 };

export class Alerts {
  constructor({ bannerEl, listEl, clearBtn }) {
    this.bannerEl = bannerEl;
    this.listEl = listEl;
    this.conditions = new Map();   // code -> { level, message }
    clearBtn.addEventListener('click', () => { this.listEl.innerHTML = ''; });
  }

  /** Aviso puntual en el historial. level: info | warning | emergency */
  push(level, message, time = Date.now() / 1000) {
    const li = document.createElement('li');
    li.className = level;
    const t = document.createElement('time');
    t.textContent = new Date(time * 1000).toLocaleTimeString('es-ES', { hour12: false });
    const span = document.createElement('span');
    span.textContent = message;
    li.append(t, span);
    this.listEl.prepend(li);
    while (this.listEl.children.length > MAX_ALERTS) this.listEl.lastChild.remove();
  }

  /** Activa o desactiva una condición persistente del banner. */
  condition(code, active, level = 'emergency', message = '') {
    const had = this.conditions.has(code);
    if (active && !had) {
      this.conditions.set(code, { level, message });
      this.push(level, message);
    } else if (!active && had) {
      this.conditions.delete(code);
    } else if (active) {
      this.conditions.set(code, { level, message });
    }
    this._renderBanner();
  }

  _renderBanner() {
    let worst = null;
    for (const c of this.conditions.values()) {
      if (!worst || SEVERITY[c.level] > SEVERITY[worst.level]) worst = c;
    }
    if (!worst) { this.bannerEl.classList.add('hidden'); return; }
    const extra = this.conditions.size > 1 ? `  (+${this.conditions.size - 1})` : '';
    this.bannerEl.textContent = worst.message + extra;
    this.bannerEl.classList.toggle('warn', worst.level !== 'emergency');
    this.bannerEl.classList.remove('hidden');
  }
}
