// Llamadas a la API HTTP del gateway (misma sesión por cookie).

export async function api(path, options = {}) {
  const r = await fetch(path, {
    credentials: 'same-origin',
    headers: { 'Content-Type': 'application/json' },
    ...options,
  });
  let body = null;
  try { body = await r.json(); } catch { /* sin cuerpo */ }
  return { ok: r.ok, status: r.status, body, error: errorText(r.status, body) };
}

/** Mensaje legible a partir de la respuesta de error de FastAPI. */
function errorText(status, body) {
  if (!body || status < 400) return null;
  if (typeof body.detail === 'string') return body.detail;
  if (Array.isArray(body.detail)) {          // 422: errores de validación
    return body.detail.map((d) => `${(d.loc || []).slice(1).join('.')}: ${d.msg}`).join('; ');
  }
  return `Error ${status}`;
}
