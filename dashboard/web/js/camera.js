// Cámara del brazo: vídeo MJPEG en un <img> superpuesto al mapa.
// Modos: 'off' (no se descarga nada), 'pip' (esquina del mapa), 'full' (ocupa
// el área del mapa). El vídeo solo se pide mientras está visible.

const URL_MJPEG = '/api/camera/arm.mjpg';
const RETRY_MS = 2000;
const STORE_KEY = 'kmr.camera.mode';

export class ArmCamera {
  constructor({ box, img, msg, info, toggleBtn, sizeBtn }) {
    Object.assign(this, { box, img, msg, info, toggleBtn, sizeBtn });
    this.mode = 'off';
    this.status = null;            // state.camera del gateway
    this.retry = null;

    this.img.addEventListener('load', () => this._setMessage(''));
    this.img.addEventListener('error', () => this._onError());
    toggleBtn.addEventListener('click', () => this.setMode(this.mode === 'off' ? 'pip' : 'off'));
    sizeBtn.addEventListener('click', () => this.setMode(this.mode === 'full' ? 'pip' : 'full'));

    let saved = 'off';
    try { saved = localStorage.getItem(STORE_KEY) || 'off'; } catch { /* sin almacenamiento */ }
    this.savedMode = ['off', 'pip', 'full'].includes(saved) ? saved : 'off';
  }

  /** Arranca en el último modo usado (tras iniciar sesión). */
  start() { this.setMode(this.savedMode); }

  /** Corta el vídeo (al cerrar sesión o perder la conexión). */
  stop() { this._closeStream(); }

  setMode(mode) {
    this.mode = mode;
    this.box.classList.toggle('hidden', mode === 'off');
    this.box.classList.toggle('full', mode === 'full');
    this.toggleBtn.classList.toggle('active', mode !== 'off');
    this.sizeBtn.textContent = mode === 'full' ? '⤡' : '⤢';
    this.sizeBtn.title = mode === 'full' ? 'Reducir' : 'Ampliar';
    try { localStorage.setItem(STORE_KEY, mode); } catch { /* sin almacenamiento */ }
    if (mode === 'off') this._closeStream();
    else if (!this.img.getAttribute('src')) this._openStream();
  }

  /** Estado que manda el gateway en cada 'state'. */
  setStatus(cam) {
    this.status = cam;
    if (this.mode === 'off' || !cam) return;
    if (!cam.available) this._setMessage('Cámara no disponible en el gateway (falta OpenCV)');
    else if (!cam.ok) this._setMessage('Sin imagen de la cámara');
    else if (!this.img.getAttribute('src')) this._openStream();   // vuelve la imagen
    else if (this.msg.dataset.kind === 'nocam') this._setMessage('');
    this.info.textContent = cam.ok && cam.size ? `${cam.size[0]}×${cam.size[1]} · ${cam.fps} fps` : '';
  }

  _openStream() {
    clearTimeout(this.retry);
    this._setMessage('Conectando…');
    this.img.src = `${URL_MJPEG}?t=${Date.now()}`;   // evita reutilizar una conexión cerrada
  }

  _closeStream() {
    clearTimeout(this.retry);
    this.img.removeAttribute('src');                 // el navegador cierra la conexión
  }

  _onError() {
    if (this.mode === 'off') return;
    this._closeStream();
    this._setMessage(this.status && !this.status.ok ? 'Sin imagen de la cámara' : 'Reconectando vídeo…');
    this.retry = setTimeout(() => { if (this.mode !== 'off') this._openStream(); }, RETRY_MS);
  }

  _setMessage(text) {
    this.msg.textContent = text;
    this.msg.dataset.kind = text.startsWith('Sin imagen') || text.startsWith('Cámara no') ? 'nocam' : '';
    this.msg.classList.toggle('hidden', !text);
  }
}
