/* Pinky Security Patrol dashboard.
 *
 * Talks to ROS 2 through rosbridge (ws://<host>:9090 by default, or ?ws=ws://ip:9090).
 *   /fleet/robot_state      RobotState            robot cards, map
 *   /fleet/objects          TrackedObjectArray    registry, map, alert
 *   /fleet/events           std_msgs/String JSON  event log (live)
 *   /fleet/event_history    std_msgs/String JSON  last 500 events (for pages opened later)
 *   /traffic/waypoints      PoseArray             map
 *   /map                    OccupancyGrid         map background
 *   /perception/stream_url                        robot MJPEG stream for the live camera view
 *   /perception/debug_image/compressed            Detections view (annotated, only while shown)
 *   /fake_pinky/status      std_msgs/String JSON  simulation panel (fake robot only)
 */
(() => {
  'use strict';

  // ------------------------------------------------------------------ state
  const state = {
    connected: false,
    robot: null,
    robotTime: 0,
    objects: new Map(),     // marker_id -> TrackedObject
    events: [],             // newest first
    waypoints: [],
    map: null,              // {canvas, info}
    fake: null,             // fake_pinky status
    battery: null,          // /fleet/battery JSON
    charger: null,          // {x, y} from /traffic/charger
    tab: 'overview',
    objFilter: 'all',
    logFilter: 'all',
    selected: null,
    dismissedAlert: new Set(),
  };
  const MAX_EVENTS = 1000;

  const $ = (id) => document.getElementById(id);
  const fmt = (n, d = 2) => (typeof n === 'number' && isFinite(n) ? n.toFixed(d) : '—');
  const fmtTime = (t) => {
    if (!t || !t.sec) return '—';
    return new Date(t.sec * 1000).toLocaleTimeString([], { hour12: false });
  };
  const LEVEL_LABEL = ['Info', 'Level 1', 'Level 2', 'Level 3'];

  function el(tag, attrs = {}, children = []) {
    const e = document.createElement(tag);
    for (const [k, v] of Object.entries(attrs)) {
      if (k === 'class') e.className = v;
      else if (k === 'text') e.textContent = v;
      else if (k.startsWith('on')) e.addEventListener(k.slice(2), v);
      else e.setAttribute(k, v);
    }
    for (const c of [].concat(children)) {
      if (c == null) continue;
      e.appendChild(typeof c === 'string' ? document.createTextNode(c) : c);
    }
    return e;
  }

  function levelBadge(level) {
    const lv = Math.max(0, Math.min(3, level | 0));
    return el('span', { class: `lvl lvl-${lv}`, text: LEVEL_LABEL[lv] });
  }

  let toastTimer = null;
  function toast(text, isError = false) {
    const t = $('toast');
    t.textContent = text;
    t.className = 'toast' + (isError ? ' error' : '');
    t.hidden = false;
    clearTimeout(toastTimer);
    toastTimer = setTimeout(() => { t.hidden = true; }, 3500);
  }

  // ------------------------------------------------------------------ ROS
  const params = new URLSearchParams(location.search);
  const wsUrl = params.get('ws') || `ws://${location.hostname || 'localhost'}:9090`;
  let ros = null;

  function connect() {
    if (typeof ROSLIB === 'undefined') {
      setConn(false, 'roslib.js not loaded (no internet?)');
      return;
    }
    ros = new ROSLIB.Ros({ url: wsUrl });
    ros.on('connection', () => { setConn(true); subscribeAll(); });
    ros.on('error', () => setConn(false, 'Connection error'));
    ros.on('close', () => {
      setConn(false, 'Disconnected, retrying…');
      setTimeout(connect, 3000);
    });
  }

  function setConn(ok, text) {
    state.connected = ok;
    $('conn-dot').className = 'dot ' + (ok ? 'dot-on' : 'dot-off');
    $('conn-text').textContent = ok ? `Connected · ${wsUrl.replace('ws://', '')}` : (text || 'Disconnected');
  }

  function topic(name, type, cb, throttle = 0) {
    const t = new ROSLIB.Topic({ ros, name, messageType: type, throttle_rate: throttle, queue_length: 1 });
    t.subscribe(cb);
    return t;
  }

  // Camera: the live view reads the robot's MJPEG stream (camera_server.py)
  // straight from the browser; only the Detections view goes through rosbridge.
  const camera = { url: params.get('cam'), override: !!params.get('cam'), mode: 'live',
    debugTopic: null, subscribed: false, failed: false };

  function showDebugImage(m) {
    const img = $('camera-img');
    img.src = `data:image/jpeg;base64,${m.data}`;
    img.hidden = false;
    $('camera-empty').hidden = true;
  }

  function updateCamera() {
    const img = $('camera-img');
    const live = camera.mode === 'live' && camera.url && !camera.failed;
    $('cam-live').setAttribute('aria-pressed', String(camera.mode === 'live'));
    $('cam-det').setAttribute('aria-pressed', String(camera.mode !== 'live'));
    const wantDebug = !live;
    if (camera.debugTopic) {
      if (wantDebug && !camera.subscribed) { camera.debugTopic.subscribe(showDebugImage); camera.subscribed = true; }
      if (!wantDebug && camera.subscribed) { camera.debugTopic.unsubscribe(); camera.subscribed = false; }
    }
    if (live) {
      if (img.dataset.stream !== camera.url) {
        img.dataset.stream = camera.url;
        img.src = camera.url;
      }
      img.hidden = false;
      $('camera-empty').hidden = true;
      $('camera-src').textContent = `live · ${camera.url}`;
    } else {
      if (img.dataset.stream) { img.removeAttribute('src'); delete img.dataset.stream; img.hidden = true; }
      $('camera-empty').hidden = !img.hidden;
      $('camera-empty').textContent = camera.failed ? 'Live stream not reachable, waiting for detections image' : 'No camera image yet';
      $('camera-src').textContent = camera.failed ? `stream failed · ${camera.url} · showing /perception/debug_image`
        : '/perception/debug_image (annotated, via rosbridge)';
    }
  }

  function subscribeAll() {
    topic('/fleet/robot_state', 'security_interfaces/msg/RobotState', (m) => {
      state.robot = m;
      state.robotTime = Date.now();
      scheduleRender();
    }, 200);
    topic('/fleet/objects', 'security_interfaces/msg/TrackedObjectArray', (m) => {
      state.objects = new Map(m.objects.map((o) => [o.marker_id, o]));
      scheduleRender();
    }, 300);
    const ev = new ROSLIB.Topic({ ros, name: '/fleet/events', messageType: 'std_msgs/msg/String', queue_length: 50 });
    ev.subscribe((m) => {
      try { addEvent(JSON.parse(m.data)); } catch (e) { /* ignore malformed */ }
    });
    topic('/fleet/event_history', 'std_msgs/msg/String', (m) => {
      let list;
      try { list = JSON.parse(m.data); } catch (e) { return; }
      state.events = list.reverse().map((e) => ({ ...e, kind: eventKind(e) }));
      scheduleRender();
    }, 3000);
    topic('/traffic/waypoints', 'geometry_msgs/msg/PoseArray', (m) => {
      state.waypoints = m.poses.map((p) => ({ x: p.position.x, y: p.position.y }));
      scheduleRender();
    }, 2000);
    topic('/fleet/battery', 'std_msgs/msg/String', (m) => {
      try { state.battery = JSON.parse(m.data); } catch (e) { return; }
      scheduleRender();
    }, 1000);
    topic('/traffic/charger', 'geometry_msgs/msg/PoseStamped', (m) => {
      state.charger = { x: m.pose.position.x, y: m.pose.position.y };
    }, 5000);
    topic('/map', 'nav_msgs/msg/OccupancyGrid', (m) => { buildMapImage(m); scheduleRender(); }, 5000);
    topic('/perception/stream_url', 'std_msgs/msg/String', (m) => {
      if (camera.override || camera.url === m.data) return;
      camera.url = m.data;
      updateCamera();
    }, 2000);
    camera.debugTopic = new ROSLIB.Topic({ ros, name: '/perception/debug_image/compressed',
      messageType: 'sensor_msgs/msg/CompressedImage', throttle_rate: 300, queue_length: 1 });
    camera.subscribed = false;
    updateCamera();
    topic('/fake_pinky/status', 'std_msgs/msg/String', (m) => {
      try { state.fake = JSON.parse(m.data); } catch (e) { return; }
      renderSim();
      scheduleRender();
    }, 500);
  }

  function callService(name, type, request, okMsg) {
    if (!state.connected) { toast('Not connected to the robot', true); return; }
    const srv = new ROSLIB.Service({ ros, name, serviceType: type });
    srv.callService(new ROSLIB.ServiceRequest(request), (res) => {
      const ok = res.success !== false;
      toast(res.message || okMsg || (ok ? 'Done' : 'Failed'), !ok);
    }, (err) => toast(`${name}: ${err}`, true));
  }
  const trigger = (name, okMsg) => callService(name, 'std_srvs/srv/Trigger', {}, okMsg);

  function sendFake(cmd) {
    const t = new ROSLIB.Topic({ ros, name: '/fake_pinky/command', messageType: 'std_msgs/msg/String' });
    t.publish({ data: cmd });
    toast(`Simulation: ${cmd}`);
  }

  // ------------------------------------------------------------------ events
  const KIND = {
    warning: new Set(['seen_again', 'moving', 'lost_track', 'hold', 'follow_start', 'emergency_stop', 'robot_lost', 'reacquired',
      'battery_low', 'battery_critical', 'go_charge', 'charger_failed', 'patrol_refused', 'dock_retry']),
    detection: new Set(['detected', 'relocated', 'removed', 'reappeared', 'resighted', 'moving_update', 'reacquired']),
    operator: new Set(['acknowledged', 'cleared', 'paused', 'resumed', 'patrol_stopped', 'emergency_stop', 'charge_ended']),
  };
  function eventKind(e) {
    if (KIND.operator.has(e.event)) return 'operator';
    if (KIND.warning.has(e.event)) return 'warning';
    if (KIND.detection.has(e.event)) return 'detection';
    return 'robot';
  }

  function addEvent(e) {
    e.kind = eventKind(e);
    state.events.unshift(e);
    if (state.events.length > MAX_EVENTS) state.events.length = MAX_EVENTS;
    if (['seen_again', 'moving', 'lost_track', 'removed'].includes(e.event)) {
      state.dismissedAlert.delete(Number(e.marker_id));
    }
    scheduleRender();
  }

  // ------------------------------------------------------------------ map
  function buildMapImage(m) {
    const { width, height } = m.info;
    if (!width || !height) return;
    const c = document.createElement('canvas');
    c.width = width; c.height = height;
    const ctx = c.getContext('2d');
    const img = ctx.createImageData(width, height);
    for (let y = 0; y < height; y++) {
      for (let x = 0; x < width; x++) {
        const v = m.data[y * width + x];
        // image rows go top-down, map rows bottom-up
        const i = ((height - 1 - y) * width + x) * 4;
        const shade = v < 0 ? 214 : v > 50 ? 43 : 247;
        img.data[i] = img.data[i + 1] = img.data[i + 2] = shade;
        img.data[i + 3] = 255;
      }
    }
    ctx.putImageData(img, 0, 0);
    state.map = { canvas: c, info: m.info };
  }

  let view = null;   // world -> screen transform

  function computeView(w, h) {
    let minX, minY, maxX, maxY;
    if (state.map) {
      const i = state.map.info;
      minX = i.origin.position.x; minY = i.origin.position.y;
      maxX = minX + i.width * i.resolution; maxY = minY + i.height * i.resolution;
    } else {
      const pts = [...state.waypoints, ...[...state.objects.values()].map((o) => o.position)];
      if (state.robot) pts.push(state.robot.pose);
      if (!pts.length) pts.push({ x: 0, y: 0 });
      minX = Math.min(...pts.map((p) => p.x)) - 1; maxX = Math.max(...pts.map((p) => p.x)) + 1;
      minY = Math.min(...pts.map((p) => p.y)) - 1; maxY = Math.max(...pts.map((p) => p.y)) + 1;
    }
    const pad = 16;
    const scale = Math.min((w - 2 * pad) / (maxX - minX), (h - 2 * pad) / (maxY - minY));
    const ox = pad + ((w - 2 * pad) - (maxX - minX) * scale) / 2;
    const oy = pad + ((h - 2 * pad) - (maxY - minY) * scale) / 2;
    return {
      scale, minX, minY, maxX, maxY,
      sx: (x) => ox + (x - minX) * scale,
      sy: (y) => oy + (maxY - y) * scale,
    };
  }

  function objectColor(o) {
    if (o.state === 'REMOVED') return '#9CA3AF';
    return o.warning_level >= 3 ? '#6D28D9' : o.warning_level === 2 ? '#B91C1C' : '#B45309';
  }

  function drawMap() {
    const canvas = $('map');
    const rect = canvas.getBoundingClientRect();
    if (!rect.width) return;
    const dpr = window.devicePixelRatio || 1;
    if (canvas.width !== Math.round(rect.width * dpr) || canvas.height !== Math.round(rect.height * dpr)) {
      canvas.width = Math.round(rect.width * dpr);
      canvas.height = Math.round(rect.height * dpr);
    }
    const ctx = canvas.getContext('2d');
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, rect.width, rect.height);
    view = computeView(rect.width, rect.height);
    const v = view;

    if (state.map) {
      ctx.imageSmoothingEnabled = false;
      ctx.drawImage(state.map.canvas, v.sx(v.minX), v.sy(v.maxY), (v.maxX - v.minX) * v.scale, (v.maxY - v.minY) * v.scale);
    } else {
      ctx.strokeStyle = '#E3E6EB';
      ctx.lineWidth = 1;
      for (let x = Math.ceil(v.minX); x <= v.maxX; x += 0.5) { ctx.beginPath(); ctx.moveTo(v.sx(x), v.sy(v.minY)); ctx.lineTo(v.sx(x), v.sy(v.maxY)); ctx.stroke(); }
      for (let y = Math.ceil(v.minY); y <= v.maxY; y += 0.5) { ctx.beginPath(); ctx.moveTo(v.sx(v.minX), v.sy(y)); ctx.lineTo(v.sx(v.maxX), v.sy(y)); ctx.stroke(); }
    }

    // patrol loop
    if (state.waypoints.length) {
      ctx.strokeStyle = '#2563EB';
      ctx.lineWidth = 2;
      ctx.setLineDash([7, 6]);
      ctx.beginPath();
      state.waypoints.forEach((p, i) => (i ? ctx.lineTo(v.sx(p.x), v.sy(p.y)) : ctx.moveTo(v.sx(p.x), v.sy(p.y))));
      ctx.closePath();
      ctx.stroke();
      ctx.setLineDash([]);
      const current = state.robot ? state.robot.waypoint_index : -1;
      state.waypoints.forEach((p, i) => {
        ctx.beginPath();
        ctx.arc(v.sx(p.x), v.sy(p.y), i === current ? 8 : 6, 0, Math.PI * 2);
        ctx.fillStyle = i === current ? '#2563EB' : '#fff';
        ctx.fill();
        ctx.strokeStyle = '#2563EB';
        ctx.stroke();
        ctx.fillStyle = '#1E3A8A';
        ctx.font = '11px Inter, sans-serif';
        ctx.fillText(`WP${i + 1}`, v.sx(p.x) + 10, v.sy(p.y) - 8);
      });
    }

    // objects
    for (const o of state.objects.values()) {
      const x = v.sx(o.position.x), y = v.sy(o.position.y);
      const color = objectColor(o);
      if (o.warning_level >= 2 && o.state !== 'REMOVED') {
        ctx.beginPath();
        ctx.arc(x, y, 18, 0, Math.PI * 2);
        ctx.fillStyle = color + '22';
        ctx.fill();
        ctx.strokeStyle = color;
        ctx.lineWidth = 2;
        ctx.stroke();
      }
      ctx.lineWidth = 2;
      if (o.state === 'REMOVED') {
        ctx.setLineDash([3, 2]);
        ctx.strokeStyle = color;
        ctx.strokeRect(x - 7, y - 7, 14, 14);
        ctx.setLineDash([]);
      } else {
        ctx.fillStyle = color;
        ctx.fillRect(x - 7, y - 7, 14, 14);
      }
      if (state.selected === o.marker_id) {
        ctx.strokeStyle = '#2563EB';
        ctx.lineWidth = 2;
        ctx.strokeRect(x - 11, y - 11, 22, 22);
      }
      ctx.fillStyle = '#15171A';
      ctx.font = '600 12px Inter, sans-serif';
      ctx.fillText(`#${o.marker_id} · ${o.state === 'REMOVED' ? 'removed' : 'L' + o.warning_level}`, x + 12, y + 4);
    }

    // charger
    if (state.charger) {
      const cx = v.sx(state.charger.x), cy = v.sy(state.charger.y);
      ctx.fillStyle = '#DCFCE7';
      ctx.strokeStyle = '#16A34A';
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.roundRect ? ctx.roundRect(cx - 11, cy - 11, 22, 22, 4) : ctx.rect(cx - 11, cy - 11, 22, 22);
      ctx.fill();
      ctx.stroke();
      ctx.beginPath();                      // lightning bolt
      ctx.moveTo(cx + 2, cy - 7); ctx.lineTo(cx - 4, cy + 1); ctx.lineTo(cx, cy + 1);
      ctx.lineTo(cx - 2, cy + 7); ctx.lineTo(cx + 4, cy - 1); ctx.lineTo(cx, cy - 1); ctx.closePath();
      ctx.fillStyle = '#15803D';
      ctx.fill();
      ctx.fillStyle = '#15803D';
      ctx.font = '600 11px Inter, sans-serif';
      ctx.fillText('home dock', cx + 14, cy + 16);
    }

    // robot
    if (state.robot) {
      const r = state.robot;
      const x = v.sx(r.pose.x), y = v.sy(r.pose.y);
      const target = state.objects.get(r.current_object_id);
      if (target) {
        ctx.strokeStyle = '#6D28D9';
        ctx.lineWidth = 1.5;
        ctx.setLineDash([4, 3]);
        ctx.beginPath();
        ctx.moveTo(x, y);
        ctx.lineTo(v.sx(target.position.x), v.sy(target.position.y));
        ctx.stroke();
        ctx.setLineDash([]);
      }
      const GLOW = { BLINK: ['rgba(220, 38, 38, 0.35)', 400], ON: ['rgba(219, 39, 119, 0.35)', 150],
        LOW: ['rgba(245, 158, 11, 0.4)', 800], CHARGE: ['rgba(22, 163, 74, 0.3)', 0] };
      const glow = GLOW[r.lamp];
      if (glow && (!glow[1] || Math.floor(Date.now() / glow[1]) % 2 === 0)) {
        ctx.beginPath();
        ctx.arc(x, y, 22, 0, Math.PI * 2);
        ctx.fillStyle = glow[0];
        ctx.fill();
      }
      ctx.save();
      ctx.translate(x, y);
      ctx.rotate(-r.pose.theta);
      ctx.beginPath();
      ctx.arc(0, 0, 12, 0, Math.PI * 2);
      ctx.fillStyle = '#15171A';
      ctx.fill();
      ctx.beginPath();
      ctx.moveTo(9, 0); ctx.lineTo(-4, -6); ctx.lineTo(-4, 6); ctx.closePath();
      ctx.fillStyle = '#fff';
      ctx.fill();
      ctx.restore();
      ctx.fillStyle = '#15171A';
      ctx.font = '600 12px Inter, sans-serif';
      ctx.fillText(r.robot_id || 'robot', x + 14, y - 12);
    }

    ctx.fillStyle = '#4B5563';
    ctx.font = '11px "JetBrains Mono", monospace';
    ctx.fillText(state.map ? `frame: map · ${Number(state.map.info.resolution).toFixed(3)} m/px` : 'frame: map · grid 0.5 m (no /map yet)', 10, rect.height - 10);
  }

  $('map').addEventListener('click', (e) => {
    if (!view) return;
    const rect = e.target.getBoundingClientRect();
    const cx = e.clientX - rect.left, cy = e.clientY - rect.top;
    let best = null, bestD = 20;
    for (const o of state.objects.values()) {
      const d = Math.hypot(view.sx(o.position.x) - cx, view.sy(o.position.y) - cy);
      if (d < bestD) { best = o; bestD = d; }
    }
    if (best) { state.selected = best.marker_id; setTab('objects'); }
  });

  // ------------------------------------------------------------------ alert
  function topAlert() {
    let best = null;
    for (const o of state.objects.values()) {
      if (o.state === 'REMOVED' || o.warning_level < 2) continue;
      if (o.acknowledged && o.warning_level < 3) continue;
      if (state.dismissedAlert.has(o.marker_id)) continue;
      if (!best || o.warning_level > best.warning_level) best = o;
    }
    return best;
  }

  function renderAlert() {
    const o = topAlert();
    const box = $('alert');
    if (!o) { box.hidden = true; return; }
    box.hidden = false;
    box.classList.toggle('l3', o.warning_level >= 3);
    const pos = `(${fmt(o.position.x)}, ${fmt(o.position.y)})`;
    if (o.state === 'MOVING') {
      $('alert-title').textContent = `Level 3 · Object #${o.marker_id} is moving`;
      $('alert-body').textContent = `Last position ${pos}. The robot is following at a safe distance.`;
    } else if (o.state === 'LOST_TRACK') {
      $('alert-title').textContent = `Level 3 · Lost track of object #${o.marker_id}`;
      $('alert-body').textContent = `Last known position ${pos}. Check the area.`;
    } else {
      $('alert-title').textContent = `Level 2 · Object #${o.marker_id} still at the same position`;
      $('alert-body').textContent = `Seen on ${o.patrol_passes_seen} patrol passes at ${pos}.`;
    }
    $('alert-view').onclick = () => { state.selected = o.marker_id; setTab('objects'); };
    $('alert-ack').onclick = () => {
      state.dismissedAlert.add(o.marker_id);
      callService('/fleet/acknowledge_object', 'security_interfaces/srv/AcknowledgeObject',
        { marker_id: o.marker_id, clear: false });
      renderAlert();
    };
  }

  // ------------------------------------------------------------------ overview
  function renderOverview() {
    const r = state.robot;
    const objs = [...state.objects.values()];
    const active = objs.filter((o) => o.state !== 'REMOVED');
    const byLevel = (n) => active.filter((o) => o.warning_level === n).length;
    $('s-objects').textContent = String(objs.length);
    $('s-objects-sub').textContent = objs.length
      ? `${active.length} active · L3 ${byLevel(3)} · L2 ${byLevel(2)} · L1 ${byLevel(1)}`
      : 'None yet';

    const age = r ? (Date.now() - state.robotTime) / 1000 : Infinity;
    if (!r || age > 5) {
      $('s-task').textContent = r ? 'NO DATA' : '—';
      $('s-task').style.color = '#B91C1C';
      $('s-task-sub').textContent = state.connected ? 'No /fleet/robot_state (is the task manager running?)' : 'Not connected';
    } else {
      const warn = ['HOLDING', 'FOLLOWING', 'EMERGENCY_STOP', 'ERROR'].includes(r.task_state);
      const COLORS = { FOLLOWING: '#6D28D9', RETURNING: '#B45309', CHARGING: '#15803D' };
      $('s-task').textContent = r.task_state;
      $('s-task').style.color = COLORS[r.task_state] || (warn ? '#B91C1C' : '');
      $('s-task-sub').textContent = `motion ${r.motion_state} · lamp ${r.lamp}`;
    }
    $('s-pass').textContent = r ? String(r.patrol_pass || '—') : '—';
    $('s-pass-sub').textContent = r && state.waypoints.length
      ? `heading to WP${r.waypoint_index + 1} of ${state.waypoints.length}` : '';
    $('s-pose').textContent = r ? `(${fmt(r.pose.x)}, ${fmt(r.pose.y)})` : '—';
    renderBattery();

    $('robot-name').textContent = r ? r.robot_id : 'Robot';
    $('r-task').textContent = r ? r.task_state : '—';
    $('r-motion').textContent = r ? r.motion_state : '—';
    $('r-lamp').textContent = r ? r.lamp : '—';
    $('r-object').textContent = r && r.current_object_id >= 0 ? `#${r.current_object_id}` : '—';
    $('r-wp').textContent = r && state.waypoints.length ? `WP${r.waypoint_index + 1}` : '—';
    $('r-age').textContent = r ? (age < 2 ? 'live' : `${age.toFixed(0)} s ago`) : '—';

    const paused = r && ['PAUSED', 'EMERGENCY_STOP'].includes(r.task_state);
    const estop = r && r.task_state === 'EMERGENCY_STOP';
    $('btn-pause').textContent = paused ? 'Resume robot' : 'Pause robot';
    $('btn-unfollow').disabled = !(r && r.task_state === 'FOLLOWING');
    $('btn-home').disabled = !!(r && ['RETURNING', 'CHARGING'].includes(r.task_state));
    $('estop').textContent = estop ? 'CLEAR STOP' : 'EMERGENCY STOP';
    $('estop').classList.toggle('active', !!estop);
  }

  function renderBattery() {
    const b = state.battery;
    const card = $('battery-card');
    card.classList.remove('low', 'critical', 'charging');
    if (!b || b.voltage == null) {
      $('s-battery').textContent = '—';
      $('s-battery-sub').textContent = b ? 'Waiting for /battery/voltage' : 'No /fleet/battery yet';
      $('battery-fill').style.width = '0';
      return;
    }
    const pct = b.percent != null ? b.percent
      : Math.max(0, Math.min(100, (b.voltage - 6.6) / (8.4 - 6.6) * 100));
    $('s-battery').textContent = `${b.voltage.toFixed(2)} V · ${Math.round(pct)}%`;
    $('battery-fill').style.width = `${pct}%`;
    let sub = 'OK';
    if (b.mode === 'CHARGING') {
      card.classList.add('charging');
      sub = b.auto_resume
        ? (b.charged ? 'Docked · charged' : 'Docked · charging')
        : (b.charged ? 'Charged · unplug, then Start patrol' : 'At the charger · plug in the cable');
    } else if (b.mode === 'RETURNING') {
      if (b.level !== 'OK') card.classList.add(b.level === 'CRITICAL' ? 'critical' : 'low');
      sub = b.level === 'OK' ? 'Going home' : 'Low · going home to charge';
    } else if (b.level === 'CRITICAL') {
      card.classList.add('critical');
      sub = `Critical (≤ ${b.critical_voltage} V)`;
    } else if (b.level === 'LOW') {
      card.classList.add('low');
      sub = b.mode === 'FOLLOWING' ? 'Low · charger after this follow' : `Low (≤ ${b.low_voltage} V)`;
    }
    $('s-battery-sub').textContent = sub;
  }

  function renderSim() {
    const card = $('sim-card');
    if (!state.fake) { card.hidden = true; return; }
    card.hidden = false;
    const list = $('sim-objects');
    list.replaceChildren(...state.fake.objects.map((o) => el('div', { class: 'sim-row' }, [
      el('span', { class: 'label', text: `#${o.id} ${o.carried ? '(carried)' : o.visible ? '' : '(hidden)'}` }),
      el('span', { class: 'row-gap wrap' }, [
        el('button', { type: 'button', class: 'btn btn-sm', text: 'Pick up', onclick: () => sendFake(`pickup ${o.id}`) }),
        el('button', { type: 'button', class: 'btn btn-sm', text: o.visible ? 'Hide' : 'Show', onclick: () => sendFake(`${o.visible ? 'hide' : 'show'} ${o.id}`) }),
        el('button', { type: 'button', class: 'btn btn-sm', text: 'Remove', onclick: () => sendFake(`remove ${o.id}`) }),
      ]),
    ])));
    const fb = state.fake.battery;
    if (!fb) { $('sim-battery').replaceChildren(); return; }
    $('sim-battery').replaceChildren(el('div', { class: 'sim-row' }, [
      el('span', { class: 'label', text: `Battery ${fb.voltage.toFixed(2)} V${fb.plugged ? ' (plugged)' : ''}` }),
      el('span', { class: 'row-gap wrap' }, [
        el('button', { type: 'button', class: 'btn btn-sm', text: 'Low', onclick: () => sendFake('battery 6.95') }),
        el('button', { type: 'button', class: 'btn btn-sm', text: 'Critical', onclick: () => sendFake('battery 6.75') }),
        el('button', { type: 'button', class: 'btn btn-sm', text: fb.plugged ? 'Unplug' : 'Plug in', onclick: () => sendFake(fb.plugged ? 'unplug' : 'plug') }),
        el('button', { type: 'button', class: 'btn btn-sm', text: 'Full', onclick: () => sendFake('battery 8.3') }),
      ]),
    ]));
  }

  // ------------------------------------------------------------------ registry
  const OBJ_FILTERS = [['all', 'All'], ['active', 'Active'], ['l2', 'Level 2+'], ['removed', 'Removed']];

  function renderFilters(containerId, defs, current, onPick) {
    $(containerId).replaceChildren(...defs.map(([id, label]) => el('button', {
      type: 'button', class: 'chip', 'aria-pressed': String(current === id), text: label,
      onclick: () => onPick(id),
    })));
  }

  function renderObjects() {
    renderFilters('obj-filters', OBJ_FILTERS, state.objFilter, (id) => { state.objFilter = id; renderObjects(); });
    const keep = (o) => state.objFilter === 'all'
      || (state.objFilter === 'active' && o.state !== 'REMOVED')
      || (state.objFilter === 'l2' && o.warning_level >= 2 && o.state !== 'REMOVED')
      || (state.objFilter === 'removed' && o.state === 'REMOVED');
    const list = [...state.objects.values()].filter(keep)
      .sort((a, b) => b.warning_level - a.warning_level || a.marker_id - b.marker_id);
    $('obj-rows').replaceChildren(...list.map((o) => {
      const tr = el('tr', { class: (o.marker_id === state.selected ? 'selected ' : '') + (o.state === 'REMOVED' ? 'removed' : '') }, [
        el('td', { text: `#${o.marker_id}` }),
        el('td', { text: o.state }),
        el('td', {}, levelBadge(o.state === 'REMOVED' ? 0 : o.warning_level)),
        el('td', { class: 'mono', text: `${fmt(o.position.x)}, ${fmt(o.position.y)}` }),
        el('td', { text: String(o.sightings) }),
        el('td', { text: String(o.patrol_passes_seen) }),
        el('td', { text: fmtTime(o.first_seen) }),
        el('td', { text: fmtTime(o.last_seen) }),
        el('td', { text: o.acknowledged ? 'Yes' : 'No' }),
        el('td', {}, el('button', {
          type: 'button', class: 'btn btn-sm', text: 'View', 'aria-label': `Show details for object ${o.marker_id}`,
          onclick: () => { state.selected = o.marker_id; renderObjects(); },
        })),
      ]);
      return tr;
    }));
    $('obj-empty').hidden = list.length > 0;
    renderDetail();
  }

  function renderDetail() {
    const o = state.objects.get(state.selected);
    const has = !!o;
    $('d-kv').hidden = !has;
    $('d-actions').hidden = !has;
    $('d-hist-title').hidden = !has;
    if (!has) {
      $('d-title').textContent = 'No object selected';
      $('d-level').replaceChildren();
      $('d-summary').textContent = 'Select an object in the table or on the map.';
      $('d-history').replaceChildren();
      return;
    }
    $('d-title').textContent = `Object #${o.marker_id}`;
    $('d-level').replaceChildren(levelBadge(o.state === 'REMOVED' ? 0 : o.warning_level));
    const summaries = {
      REPORTED: 'Reported on this pass. It escalates if it is still there on a later pass.',
      PERSISTENT: 'Seen at the same place on more than one patrol pass.',
      MOVING: 'Its position keeps changing. The robot follows it.',
      LOST_TRACK: 'The robot lost sight of it while following.',
      REMOVED: 'Its logged place was in view, but it was not detected anymore.',
    };
    $('d-summary').textContent = summaries[o.state] || '';
    $('d-state').textContent = o.state;
    $('d-frame').textContent = `suspicious_object_${o.marker_id}`;
    $('d-pos').textContent = `${fmt(o.position.x)}, ${fmt(o.position.y)}`;
    $('d-ack').textContent = o.acknowledged ? 'Yes' : 'No';
    $('d-sightings').textContent = String(o.sightings);
    $('d-passes').textContent = String(o.patrol_passes_seen);
    const hist = state.events
      .filter((e) => Number(e.marker_id) === o.marker_id && e.x_map !== '' && e.x_map != null)
      .slice(0, 40);
    $('d-history').replaceChildren(...(hist.length ? hist.map((e) => el('li', {}, [
      el('span', { class: 'muted', text: (e.time || '').slice(11) }),
      el('span', { text: `${fmt(Number(e.x_map))}, ${fmt(Number(e.y_map))}` }),
      el('span', { class: 'muted', text: e.event }),
    ])) : [el('li', {}, el('span', { class: 'muted', text: 'No position events in this session yet' }))]));
    $('d-ack-btn').onclick = () => callService('/fleet/acknowledge_object', 'security_interfaces/srv/AcknowledgeObject', { marker_id: o.marker_id, clear: false });
    $('d-clear-btn').onclick = () => {
      if (!confirm(`Clear object #${o.marker_id} from the registry?`)) return;
      callService('/fleet/acknowledge_object', 'security_interfaces/srv/AcknowledgeObject', { marker_id: o.marker_id, clear: true });
      state.selected = null;
    };
    $('d-log-btn').onclick = () => { state.logFilter = `#${o.marker_id}`; setTab('log'); };
  }

  // ------------------------------------------------------------------ log
  const LOG_FILTERS = [['all', 'All'], ['warning', 'Warnings'], ['detection', 'Detections'], ['robot', 'Robot'], ['operator', 'Operator']];

  function filteredEvents() {
    const f = state.logFilter;
    if (f === 'all') return state.events;
    if (f.startsWith('#')) return state.events.filter((e) => `#${e.marker_id}` === f);
    return state.events.filter((e) => e.kind === f);
  }

  function renderLog() {
    const defs = LOG_FILTERS.slice();
    if (state.logFilter.startsWith('#')) defs.push([state.logFilter, `${state.logFilter} only`]);
    renderFilters('log-filters', defs, state.logFilter, (id) => {
      state.logFilter = (id === state.logFilter && id.startsWith('#')) ? 'all' : id;
      renderLog();
    });
    const rows = filteredEvents().slice(0, 300);
    $('log-rows').replaceChildren(...rows.map((e) => {
      const lvl = e.warning_level === '' || e.warning_level == null ? null : Number(e.warning_level);
      const cls = e.kind === 'warning' && lvl >= 2 ? `ev-l${Math.min(lvl, 3)}` : (e.event === 'detected' ? 'ev-l1' : '');
      return el('tr', { class: cls }, [
        el('td', { text: (e.time || '').slice(11) }),
        el('td', { text: e.source === 'traffic_manager' ? 'traffic' : 'task' }),
        el('td', { text: e.marker_id === '' || e.marker_id == null ? '—' : `#${e.marker_id}` }),
        el('td', { text: e.event }),
        el('td', { text: e.state || '—' }),
        el('td', { text: lvl == null ? '—' : (lvl === 0 ? 'info' : `L${lvl}`) }),
        el('td', { text: e.x_map === '' || e.x_map == null ? '—' : `${fmt(Number(e.x_map))}, ${fmt(Number(e.y_map))}` }),
        el('td', { text: `${fmt(Number(e.robot_x))}, ${fmt(Number(e.robot_y))}` }),
        el('td', { text: String(e.patrol_pass ?? '—') }),
        el('td', { class: 'small', text: e.message || '' }),
      ]);
    }));
    $('log-empty').hidden = rows.length > 0;
  }

  $('export').addEventListener('click', () => {
    const cols = ['time', 'source', 'marker_id', 'event', 'state', 'warning_level', 'x_map', 'y_map', 'robot_x', 'robot_y', 'patrol_pass', 'message'];
    const esc = (v) => {
      const s = v == null ? '' : String(v);
      return /[",\n]/.test(s) ? `"${s.replace(/"/g, '""')}"` : s;
    };
    const lines = [cols.join(',')].concat(filteredEvents().slice().reverse().map((e) => cols.map((c) => esc(e[c])).join(',')));
    const blob = new Blob([lines.join('\n')], { type: 'text/csv' });
    const a = el('a', { href: URL.createObjectURL(blob), download: `patrol_events_${new Date().toISOString().slice(0, 19).replace(/[:T]/g, '-')}.csv` });
    document.body.appendChild(a);
    a.click();
    a.remove();
  });

  // ------------------------------------------------------------------ tabs and controls
  function setTab(tab) {
    state.tab = tab;
    document.querySelectorAll('.tab').forEach((b) => b.setAttribute('aria-pressed', String(b.dataset.tab === tab)));
    for (const id of ['overview', 'objects', 'log']) $(`tab-${id}`).hidden = id !== tab;
    render();
  }
  document.querySelectorAll('.tab').forEach((b) => b.addEventListener('click', () => setTab(b.dataset.tab)));

  $('btn-start').addEventListener('click', () => trigger('/fleet/start_patrol'));
  $('btn-stop').addEventListener('click', () => trigger('/fleet/stop_patrol'));
  $('btn-unfollow').addEventListener('click', () => trigger('/fleet/stop_follow'));
  $('btn-home').addEventListener('click', () => trigger('/fleet/go_home'));
  $('cam-live').addEventListener('click', () => { camera.mode = 'live'; camera.failed = false; updateCamera(); });
  $('cam-det').addEventListener('click', () => { camera.mode = 'detections'; updateCamera(); });
  $('camera-img').addEventListener('error', () => {
    if (camera.mode !== 'live' || !camera.url) return;
    camera.failed = true;
    updateCamera();
    setTimeout(() => { if (camera.failed && camera.mode === 'live') { camera.failed = false; updateCamera(); } }, 10000);
  });
  $('btn-pause').addEventListener('click', () => {
    const r = state.robot;
    trigger(r && ['PAUSED', 'EMERGENCY_STOP'].includes(r.task_state) ? '/traffic/resume' : '/traffic/pause');
  });
  $('estop').addEventListener('click', () => {
    const r = state.robot;
    if (r && r.task_state === 'EMERGENCY_STOP') trigger('/traffic/resume', 'Emergency stop cleared');
    else trigger('/traffic/emergency_stop', 'EMERGENCY STOP sent');
  });

  // ------------------------------------------------------------------ render loop
  let renderPending = false;
  function scheduleRender() {
    if (renderPending) return;
    renderPending = true;
    requestAnimationFrame(() => { renderPending = false; render(); });
  }

  function render() {
    $('badge-objects').textContent = String(state.objects.size);
    $('badge-log').textContent = String(state.events.length);
    renderAlert();
    if (state.tab === 'overview') { renderOverview(); drawMap(); }
    if (state.tab === 'objects') renderObjects();
    if (state.tab === 'log') renderLog();
  }

  setInterval(() => {
    $('clock').textContent = new Date().toLocaleTimeString([], { hour12: false });
    if (state.tab === 'overview') { renderOverview(); drawMap(); }   // keeps "last update" and the blinking lamp fresh
  }, 400);
  window.addEventListener('resize', scheduleRender);

  connect();
  render();
})();
