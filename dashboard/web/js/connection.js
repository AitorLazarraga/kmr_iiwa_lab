// Conexión WebSocket con el gateway, con reconexión automática.

export class Connection {
  constructor({ onMessage, onStatus }) {
    this.onMessage = onMessage;
    this.onStatus = onStatus;          // (connected: bool) => void
    this.ws = null;
    this.connected = false;
    this.retryMs = 2000;
    const proto = location.protocol === 'https:' ? 'wss' : 'ws';
    this.url = `${proto}://${location.host}/ws`;
    this.open();
  }

  open() {
    const ws = new WebSocket(this.url);
    this.ws = ws;
    ws.onopen = () => this._setConnected(true);
    ws.onmessage = (ev) => {
      let msg;
      try { msg = JSON.parse(ev.data); } catch { return; }
      this.onMessage(msg);
    };
    ws.onclose = () => {
      this._setConnected(false);
      setTimeout(() => this.open(), this.retryMs);
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
