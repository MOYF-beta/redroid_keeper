from __future__ import annotations

import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

from loguru import logger


def _monitor_html(ws_port: int) -> str:
    return f"""<!doctype html>
<html><head><meta charset='utf-8'><title>Redroid WS Monitor</title>
<style>
body{{font-family:Arial,sans-serif;margin:16px;background:#f7f7f9}}
.panel{{background:white;border:1px solid #ddd;border-radius:8px;padding:12px;margin-bottom:12px}}
.row{{display:flex;gap:12px;align-items:flex-start}}
.col{{flex:1;min-width:0}}
pre{{background:#111;color:#0f0;padding:8px;overflow:auto;max-height:320px;border-radius:6px}}
#canvasWrap{{position:relative;display:inline-block;border:1px solid #ccc;background:#000}}
#screen{{max-width:640px;display:block}}
#overlay{{position:absolute;left:0;top:0;pointer-events:none}}
#historyList button{{display:block;width:100%;text-align:left;margin:4px 0;padding:6px;border-radius:6px;border:1px solid #ddd;background:#fff;cursor:pointer}}
#historyList button.active{{background:#eaf3ff;border-color:#88b6ff}}
.small{{font-size:12px;color:#666}}
</style>
</head><body>
<h2>Redroid WS Monitor</h2>
<div class='panel'>
  <label>Device: <select id='dev'></select></label>
  <button onclick='forceSync()'>Sync now</button>
  <button onclick='clearHistory()'>Clear session history</button>
  <button onclick='prevFrame()'>Prev</button>
  <button onclick='nextFrame()'>Next</button>
  <span id='wsStatus' class='small'></span>
</div>

<div class='row'>
  <div class='col panel'>
    <h3>Screen + Operation Overlay</h3>
    <div id='canvasWrap'>
      <img id='screen' />
      <canvas id='overlay'></canvas>
    </div>
    <div id='frameMeta' class='small'></div>
  </div>
  <div class='col panel'>
    <h3>Session Screenshot History</h3>
    <div id='historyList' style='max-height:460px;overflow:auto'></div>
  </div>
</div>

<div class='row'>
  <div class='col panel'><h3>Latest Observation</h3><pre id='obs'></pre></div>
  <div class='col panel'><h3>Latest Request</h3><pre id='req'></pre></div>
</div>
<div class='row'>
  <div class='col panel'><h3>Latest Info</h3><pre id='info'></pre></div>
  <div class='col panel'><h3>Recent Events (Server)</h3><pre id='events'></pre></div>
</div>

<script>
const WS_PORT = {ws_port};
let ws = null;
let snapshot = {{}};
const selectedIndexByDevice = {{}};

function nowId() {{ return String(Date.now()) + '_' + Math.random().toString(36).slice(2); }}
function monitorWsUrl() {{ const p = location.protocol === 'https:' ? 'wss' : 'ws'; return `${{p}}://${{location.hostname}}:${{WS_PORT}}`; }}
function setWsStatus(t) {{ document.getElementById('wsStatus').textContent = t; }}

function storageKey(dev) {{ return `redroid_monitor_history_${{dev}}`; }}
function loadHistory(dev) {{
  try {{ return JSON.parse(sessionStorage.getItem(storageKey(dev)) || '[]'); }}
  catch {{ return []; }}
}}
function saveHistory(dev, arr) {{
  const keep = arr.slice(-500);
  sessionStorage.setItem(storageKey(dev), JSON.stringify(keep));
}}

function ensureDeviceOptions() {{
  const sel = document.getElementById('dev');
  const cur = sel.value;
  const ids = (snapshot.allowed_device_ids || snapshot.devices || []).map(x => String(x));
  sel.innerHTML = '';
  ids.forEach(d => {{ const o = document.createElement('option'); o.value = d; o.text = d; sel.appendChild(o); }});
  if (cur && ids.includes(cur)) sel.value = cur;
  else if (ids.length > 0) sel.value = ids[0];
}}

function appendFrameFromState(dev, st) {{
  const lr = (st && st.last_response) || {{}};
  const info = (lr && lr.info) || {{}};
  const b64 = lr.image_b64 || '';
  if (!b64) return;

  const frame = {{
    key: `${{st.updated_at||0}}|${{st.last_op||''}}|${{(info.executor_status||'')}}|${{b64.length}}`,
    ts: st.updated_at || 0,
    op: st.last_op || '',
    status: info.executor_status || '',
    observation_text: lr.observation_text || '',
    request: st.last_request || {{}},
    info: info,
    image_b64: b64,
    tool_returns: Array.isArray(info.tool_returns) ? info.tool_returns : [],
  }};

  const hist = loadHistory(dev);
  if (hist.length > 0 && hist[hist.length - 1].key === frame.key) return;
  hist.push(frame);
  saveHistory(dev, hist);
}}

function parseDirection(label) {{
  const m = String(label || '').match(/direction\\s*=\\s*(-?\\d+(?:\\.\\d+)?)/i);
  if (!m) return null;
  return Number(m[1]);
}}

function drawArrow(ctx, x1, y1, x2, y2, color) {{
  ctx.strokeStyle = color; ctx.fillStyle = color; ctx.lineWidth = 2;
  ctx.beginPath(); ctx.moveTo(x1, y1); ctx.lineTo(x2, y2); ctx.stroke();
  const ang = Math.atan2(y2 - y1, x2 - x1);
  const hl = 8;
  ctx.beginPath();
  ctx.moveTo(x2, y2);
  ctx.lineTo(x2 - hl*Math.cos(ang - Math.PI/6), y2 - hl*Math.sin(ang - Math.PI/6));
  ctx.lineTo(x2 - hl*Math.cos(ang + Math.PI/6), y2 - hl*Math.sin(ang + Math.PI/6));
  ctx.closePath(); ctx.fill();
}}

function drawToolOverlay(ctx, tr, sx, sy) {{
  const action = (tr && tr.action) || {{}};
  const label = String(action.label || '');
  const rawB = action.bbox_2d || tr.bbox_2d || null;
  let bbox = null;
  if (Array.isArray(rawB) && rawB.length === 4) {{
    const nums = rawB.map(v => Number(v));
    if (nums.every(Number.isFinite)) bbox = nums;
  }}

  let cx = 40, cy = 40;
  if (bbox) {{
    const [x1,y1,x2,y2] = bbox;
    const rx = x1 * sx, ry = y1 * sy, rw = (x2-x1) * sx, rh = (y2-y1) * sy;
    ctx.strokeStyle = '#00e0ff';
    ctx.lineWidth = 2;
    ctx.strokeRect(rx, ry, rw, rh);
    cx = rx + rw / 2;
    cy = ry + rh / 2;
  }}

  if (label.includes('touch.tap')) {{
    ctx.beginPath(); ctx.strokeStyle = '#2ecc71'; ctx.lineWidth = 3; ctx.arc(cx, cy, 12, 0, Math.PI*2); ctx.stroke();
  }} else if (label.includes('touch.long_tap')) {{
    ctx.beginPath(); ctx.strokeStyle = '#f1c40f'; ctx.lineWidth = 3; ctx.arc(cx, cy, 16, 0, Math.PI*2); ctx.stroke();
  }} else if (label.includes('touch.swipe')) {{
    const dir = parseDirection(label);
    const deg = (dir == null ? 90 : dir) * Math.PI / 180;
    const len = 80;
    const x2 = cx + Math.cos(deg) * len;
    const y2 = cy - Math.sin(deg) * len;
    drawArrow(ctx, cx, cy, x2, y2, '#ff4d4f');
  }}

  ctx.fillStyle = 'rgba(0,0,0,0.7)';
  ctx.fillRect(8, 8 + (drawToolOverlay._line||0)*18, Math.min(620, 12 + label.length * 7), 16);
  ctx.fillStyle = '#fff';
  ctx.font = '12px sans-serif';
  ctx.fillText(label || (tr && tr.func_name) || 'action', 12, 20 + (drawToolOverlay._line||0)*18);
  drawToolOverlay._line = (drawToolOverlay._line || 0) + 1;
}}

drawToolOverlay._line = 0;

function renderFrame(frame) {{
  const img = document.getElementById('screen');
  const canvas = document.getElementById('overlay');
  const ctx = canvas.getContext('2d');

  if (!frame || !frame.image_b64) {{
    img.removeAttribute('src');
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    document.getElementById('frameMeta').textContent = 'No frame';
    return;
  }}

  img.onload = () => {{
    canvas.width = img.clientWidth;
    canvas.height = img.clientHeight;
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    const sx = img.clientWidth / img.naturalWidth;
    const sy = img.clientHeight / img.naturalHeight;
    drawToolOverlay._line = 0;

    const trs = Array.isArray(frame.tool_returns) ? frame.tool_returns : [];
    for (const tr of trs) drawToolOverlay(ctx, tr, sx, sy);
  }};
  img.src = 'data:image/jpeg;base64,' + frame.image_b64;

  document.getElementById('frameMeta').textContent =
    `ts=${{new Date((frame.ts||0)*1000).toLocaleString()}} | op=${{frame.op}} | status=${{frame.status}} | actions=${{(frame.tool_returns||[]).length}}`;
  document.getElementById('obs').textContent = frame.observation_text || '';
  document.getElementById('req').textContent = JSON.stringify(frame.request || {{}}, null, 2);
  document.getElementById('info').textContent = JSON.stringify(frame.info || {{}}, null, 2);
}}

function renderHistory() {{
  const dev = document.getElementById('dev').value;
  const list = document.getElementById('historyList');
  const hist = loadHistory(dev);
  const selected = selectedIndexByDevice[dev] ?? (hist.length - 1);
  selectedIndexByDevice[dev] = Math.max(0, Math.min(selected, hist.length - 1));

  list.innerHTML = '';
  hist.forEach((f, idx) => {{
    const b = document.createElement('button');
    if (idx === selectedIndexByDevice[dev]) b.className = 'active';
    const t = new Date((f.ts||0)*1000).toLocaleTimeString();
    b.textContent = `${{idx+1}}. [${{t}}] ${{f.op}} | ${{f.status}}`;
    b.onclick = () => {{ selectedIndexByDevice[dev] = idx; renderHistory(); renderSelectedFrame(); }};
    list.appendChild(b);
  }});
}}

function renderSelectedFrame() {{
  const dev = document.getElementById('dev').value;
  const hist = loadHistory(dev);
  const idx = selectedIndexByDevice[dev] ?? (hist.length - 1);
  renderFrame(hist[idx]);
}}

function prevFrame() {{
  const dev = document.getElementById('dev').value;
  const hist = loadHistory(dev);
  if (!hist.length) return;
  const cur = selectedIndexByDevice[dev] ?? (hist.length - 1);
  selectedIndexByDevice[dev] = Math.max(0, cur - 1);
  renderHistory(); renderSelectedFrame();
}}

function nextFrame() {{
  const dev = document.getElementById('dev').value;
  const hist = loadHistory(dev);
  if (!hist.length) return;
  const cur = selectedIndexByDevice[dev] ?? (hist.length - 1);
  selectedIndexByDevice[dev] = Math.min(hist.length - 1, cur + 1);
  renderHistory(); renderSelectedFrame();
}}

function clearHistory() {{
  const dev = document.getElementById('dev').value;
  sessionStorage.removeItem(storageKey(dev));
  selectedIndexByDevice[dev] = 0;
  renderHistory(); renderSelectedFrame();
}}

function renderEvents() {{
  const dev = document.getElementById('dev').value;
  const events = (snapshot.events || []).filter(e => String(e.device_id) === String(dev)).slice(-80).reverse();
  document.getElementById('events').textContent = JSON.stringify(events, null, 2);
}}

function consumeSnapshot(s) {{
  snapshot = s || {{}};
  ensureDeviceOptions();
  const dev = document.getElementById('dev').value;
  const st = ((snapshot.states || {{}})[String(dev)] || {{}});
  appendFrameFromState(dev, st);
  renderHistory();
  renderSelectedFrame();
  renderEvents();
}}

function requestSnapshot() {{
  if (!ws || ws.readyState !== WebSocket.OPEN) return;
  ws.send(JSON.stringify({{ request_id: nowId(), op: 'monitor_state' }}));
}}

function connectWs() {{
  setWsStatus('connecting...');
  ws = new WebSocket(monitorWsUrl());
  ws.onopen = () => {{ setWsStatus('connected'); requestSnapshot(); }};
  ws.onmessage = (ev) => {{
    try {{
      const msg = JSON.parse(ev.data);
      if (msg && msg.ok && msg.monitor_snapshot) consumeSnapshot(msg.monitor_snapshot);
    }} catch (e) {{ console.error(e); }}
  }};
  ws.onerror = () => setWsStatus('error');
  ws.onclose = () => {{ setWsStatus('closed; reconnecting...'); setTimeout(connectWs, 1500); }};
}}

function forceSync() {{ requestSnapshot(); }}

setInterval(requestSnapshot, 1200);
document.getElementById('dev').addEventListener('change', () => {{ renderHistory(); renderSelectedFrame(); renderEvents(); }});
connectWs();
</script>
</body></html>"""


def run_monitor_ui(host: str, port: int, ws_port: int, service=None) -> ThreadingHTTPServer:
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            parsed = urlparse(self.path)
            path = parsed.path
            query = parse_qs(parsed.query or "")

            def _q_bool(name: str, default: bool = False) -> bool:
                raw = query.get(name, [str(default)])[0].strip().lower()
                return raw in {"1", "true", "yes", "y", "on"}

            def _q_int(name: str):
                raw = query.get(name, [""])[0].strip()
                if not raw:
                    return None
                try:
                    return int(raw)
                except Exception:
                    return None

            if path == "/" or path == "/index.html":
                body = _monitor_html(ws_port=ws_port).encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return
            if path == "/api/ping":
                # Backward-compatible default ping, plus optional richer checks via query params:
                # - ?deep=1
                # - ?device_id=1
                # - ?screenshot=1
                # - ?mock=1
                # - ?ws=1
                result = {"ok": True, "ui": "ok"}

                deep = _q_bool("deep", False)
                include_ws = _q_bool("ws", False)

                if deep and service is not None:
                    payload = {
                        "op": "health_check",
                        "device_id": _q_int("device_id"),
                        "include_screenshot": _q_bool("screenshot", False),
                        "include_mock_ops": _q_bool("mock", False),
                    }
                    result["health"] = service.health_check(payload)
                    result["ok"] = bool(result["ok"] and result["health"].get("ok", False))

                if include_ws:
                    try:
                        from websockets.sync.client import connect

                        ws_url = f"ws://127.0.0.1:{ws_port}"
                        with connect(ws_url, open_timeout=5, close_timeout=5) as ws:
                            ws.send(json.dumps({"request_id": "http-ping", "op": "monitor_state"}))
                            resp = ws.recv(timeout=8)
                        ws_resp = json.loads(resp) if isinstance(resp, str) else {}
                        result["ws_bridge"] = {
                            "ok": bool(ws_resp.get("ok", False)),
                            "url": ws_url,
                        }
                        result["ok"] = bool(result["ok"] and result["ws_bridge"]["ok"])
                    except Exception as e:
                        result["ws_bridge"] = {"ok": False, "error": str(e)}
                        result["ok"] = False

                body = json.dumps(result, ensure_ascii=False).encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return
            if path == "/api/health":
                if service is None:
                    body = b'{"ok": false, "error": "service_not_available"}'
                    self.send_response(500)
                    self.send_header("Content-Type", "application/json")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                    return

                payload = {
                    "op": "health_check",
                    "device_id": _q_int("device_id"),
                    "include_screenshot": _q_bool("screenshot", True),
                    "include_mock_ops": _q_bool("mock", True),
                }
                result = service.health_check(payload)
                body = json.dumps(result, ensure_ascii=False).encode("utf-8")
                self.send_response(200 if result.get("ok", False) else 503)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return
            self.send_response(404)
            self.end_headers()

        def log_message(self, fmt, *args):
            logger.debug("UI {} - {}", self.client_address[0], fmt % args)

    server = ThreadingHTTPServer((host, port), Handler)
    th = threading.Thread(target=server.serve_forever, daemon=True)
    th.start()
    logger.info(f"Redroid monitor UI started at http://{host}:{port}")
    return server
