// Conexión WebSocket con el gateway, con reconexión automática.
// Si el gateway cierra con 4401 (sesión no válida), no se reintenta: se avisa
// para volver a pedir login.

const WS_CLOSE_AUTH = 4401;

export class Connection {
  constructor({ onMessage, onStatus, onAuthLost }) {
    this.onMessage = onMessage;
    this.onStatus = onStatus;          // (connected: bool) => void
    this.onAuthLost = onAuthLost;      // () => void
    this.ws = null;
    this.connected = false;
    this.active = false;
    this.retryMs = 2000;
    const proto = location.protocol === 'https:' ? 'wss' : 'ws';
    this.url = `${proto}://${location.host}/ws`;
  }

  start() {
    if (this.active) return;
    this.active = true;
    this._open();
  }

  stop() {
    this.active = false;
    if (this.ws) this.ws.close();
  }

  _open() {
    const ws = new WebSocket(this.url);
    this.ws = ws;
    ws.onopen = () => this._setConnected(true);
    ws.onmessage = (ev) => {
      let msg;
      try { msg = JSON.parse(ev.data); } catch { return; }
      this.onMessage(msg);
    };
    ws.onclose = (ev) => {
      this._setConnected(false);
      if (ev.code === WS_CLOSE_AUTH) {
        this.active = false;
        this.onAuthLost();
        return;
      }
      if (this.active) setTimeout(() => this.active && this._open(), this.retryMs);
    };
    ws.onerror = () => ws.close();
  }

  _setConnected(value) {
    if (this.connected === value) return;
    this.connected = value;
    this.onStatus(value);
  }

  /** Envía un objeto. Devuelve false si no hay conexión. */
  send(obj) {
    if (!this.ws || this.ws.readyState !== WebSocket.OPEN) return false;
    this.ws.send(JSON.stringify(obj));
    return true;
  }
}
