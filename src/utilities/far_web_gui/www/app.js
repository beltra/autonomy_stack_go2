/**
 * FAR Planner Web GUI — app.js
 * Single-file frontend that connects to rosbridge via roslibjs,
 * displays the obstacle graph and robot pose on a 2D canvas,
 * and provides goalpoint, teleop, and VGH save/load controls.
 */

'use strict';

/* ====================================================================
   GLOBALS
   ==================================================================== */

let ros = null;               // ROSLIB.Ros connection
let connected = false;

// ROS topic handles (created on connect)
let subOdom = null;
let subGoalStatus = null;
let subVizGraph = null;
let subVizPath = null;
let subVizNode = null;

let pubGoalPoint = null;
let pubJoy = null;
let pubAttemptable = null;
let pubUpdateVGraph = null;
let pubResetVGraph = null;
let pubUploadGraph = null;

let srvDownloadGraph = null;

// Robot state
const robotState = { x: 0, y: 0, z: 0, yaw: 0 };
let goalReached = null;

// Map data
let graphEdges = [];          // [{ x1, y1, x2, y2, ns }] — obstacle edges only
let globalPath = [];          // [{ x, y }]
let vizGoalMarkers = [];      // [{ x, y, type }]

// Canvas / view state
let canvas, ctx;
const view = { cx: 0, cy: 0, scale: 20, targetScale: 20 }; // pixels per meter
let followRobot = true;
let showGraph = true;
let showPath = true;

// Goalpoint mode
let goalMode = false;

// Teleop
let teleopFwd = 0, teleopYaw = 0;
let teleopActive = false;
let teleopInterval = null;

// HTTP base (for bridge API)
let httpBase = '';

// Network profile for congested links:
// keep robot state responsive, throttle heavy visualization topics,
// and drop stale queued messages.
const NET = {
  odomThrottleMs: 50,
  goalStatusThrottleMs: 200,
  graphThrottleMs: 1200,
  pathThrottleMs: 400,
  nodeThrottleMs: 1500,
  queueLength: 1,
};

function getCurrentHost() {
  return window.location.hostname || 'localhost';
}

function getCurrentWsProtocol() {
  return window.location.protocol === 'https:' ? 'wss' : 'ws';
}

/* ====================================================================
   INIT
   ==================================================================== */

window.addEventListener('DOMContentLoaded', () => {
  canvas = document.getElementById('map-canvas');
  ctx = canvas.getContext('2d');
  resizeCanvas();
  window.addEventListener('resize', resizeCanvas);

  // Buttons
  document.getElementById('btn-connect').addEventListener('click', toggleConnect);
  document.getElementById('btn-reset-vgraph').addEventListener('click', resetVGraph);
  document.getElementById('btn-resume-nav').addEventListener('click', resumeNav);
  document.getElementById('btn-download-graph').addEventListener('click', downloadGraph);
  document.getElementById('file-upload-input').addEventListener('change', uploadGraph);
  document.getElementById('btn-center-robot').addEventListener('click', centerOnRobot);
  document.getElementById('btn-set-goal').addEventListener('click', toggleGoalMode);
  document.getElementById('chk-follow-robot').addEventListener('change', e => { followRobot = e.target.checked; });
  document.getElementById('chk-show-graph').addEventListener('change', e => { showGraph = e.target.checked; });
  document.getElementById('chk-show-path').addEventListener('change', e => { showPath = e.target.checked; });
  document.getElementById('chk-attemptable').addEventListener('change', publishAttemptable);
  document.getElementById('chk-update-vgraph').addEventListener('change', publishUpdateVGraph);
  document.getElementById('btn-safety-shutdown').addEventListener('click', safetyShutdown);
  document.getElementById('btn-reboot').addEventListener('click', rebootRobot);

  // Map interactions
  canvas.addEventListener('wheel', onWheel, { passive: false });
  canvas.addEventListener('mousedown', onMouseDown);
  canvas.addEventListener('mousemove', onMouseMove);
  canvas.addEventListener('mouseup', onMouseUp);
  canvas.addEventListener('contextmenu', e => { e.preventDefault(); cancelGoalMode(); });
  canvas.addEventListener('click', onMapClick);

  // Teleop stick
  initTeleopStick();

  // Render loop
  requestAnimationFrame(renderLoop);

  // Try auto-connect with defaults
  document.getElementById('robot-ip').value = getCurrentHost();
  const params = new URLSearchParams(window.location.search);
  if (params.has('ip')) document.getElementById('robot-ip').value = params.get('ip');
  if (params.has('ws')) document.getElementById('ws-port').value = params.get('ws');
  if (params.has('http')) document.getElementById('http-port').value = params.get('http');
});

function resizeCanvas() {
  canvas.width = canvas.parentElement.clientWidth;
  canvas.height = canvas.parentElement.clientHeight;
}

/* ====================================================================
   CONNECTION
   ==================================================================== */

function toggleConnect() {
  if (connected) { disconnectRos(); }
  else { connectRos(); }
}

function connectRos() {
  if (!window.ROSLIB) {
    console.error('ROSLIB is unavailable. Check roslib.min.js / roslib_shim.js loading.');
    setConnStatus(false);
    return;
  }

  const ip = document.getElementById('robot-ip').value.trim() || getCurrentHost();
  const wsPort = document.getElementById('ws-port').value.trim() || '9090';
  const httpPort = document.getElementById('http-port').value.trim() || '8080';
  httpBase = `http://${ip}:${httpPort}`;
  const url = `${getCurrentWsProtocol()}://${ip}:${wsPort}`;

  ros = new ROSLIB.Ros({ url });

  ros.on('connection', () => {
    connected = true;
    setConnStatus(true);
    setupTopics();
  });
  ros.on('error', (err) => {
    console.error('ROS error:', err);
  });
  ros.on('close', () => {
    connected = false;
    setConnStatus(false);
    cleanupTopics();
  });
}

function disconnectRos() {
  if (ros) ros.close();
  connected = false;
  setConnStatus(false);
  cleanupTopics();
}

function setConnStatus(ok) {
  const el = document.getElementById('conn-status');
  el.textContent = ok ? '● Connected' : '● Disconnected';
  el.className = 'status ' + (ok ? 'connected' : 'disconnected');
  document.getElementById('btn-connect').textContent = ok ? 'Disconnect' : 'Connect';
}

/* ====================================================================
   ROS TOPICS / SERVICES
   ==================================================================== */

function setupTopics() {
  // --- Subscribers ---
  subOdom = new ROSLIB.Topic({
    ros,
    name: '/state_estimation',
    messageType: 'nav_msgs/Odometry',
    throttle_rate: NET.odomThrottleMs,
    queue_length: NET.queueLength,
  });
  subOdom.subscribe(onOdom);

  subGoalStatus = new ROSLIB.Topic({
    ros,
    name: '/far_reach_goal_status',
    messageType: 'std_msgs/Bool',
    throttle_rate: NET.goalStatusThrottleMs,
    queue_length: NET.queueLength,
  });
  subGoalStatus.subscribe(msg => { goalReached = msg.data; });

  // Visualization markers from FAR planner (obstacle edges only)
  subVizGraph = new ROSLIB.Topic({
    ros,
    name: '/viz_graph_topic',
    messageType: 'visualization_msgs/MarkerArray',
    throttle_rate: NET.graphThrottleMs,
    queue_length: NET.queueLength,
  });
  subVizGraph.subscribe(onVizGraph);

  // Visualization markers (path)
  subVizPath = new ROSLIB.Topic({
    ros,
    name: '/viz_path_topic',
    messageType: 'visualization_msgs/Marker',
    throttle_rate: NET.pathThrottleMs,
    queue_length: NET.queueLength,
  });
  subVizPath.subscribe(onVizPath);

  // Visualization markers (goalpoint / nodes)
  subVizNode = new ROSLIB.Topic({
    ros,
    name: '/viz_node_topic',
    messageType: 'visualization_msgs/MarkerArray',
    throttle_rate: NET.nodeThrottleMs,
    queue_length: NET.queueLength,
  });
  subVizNode.subscribe(onVizNode);

  // --- Publishers ---
  pubGoalPoint = new ROSLIB.Topic({ ros, name: '/goal_point', messageType: 'geometry_msgs/PointStamped' });
  pubJoy = new ROSLIB.Topic({ ros, name: '/joy', messageType: 'sensor_msgs/Joy' });
  pubAttemptable = new ROSLIB.Topic({ ros, name: '/planning_attemptable', messageType: 'std_msgs/Bool' });
  pubUpdateVGraph = new ROSLIB.Topic({ ros, name: '/update_visibility_graph', messageType: 'std_msgs/Bool' });
  pubResetVGraph = new ROSLIB.Topic({ ros, name: '/reset_visibility_graph', messageType: 'std_msgs/Empty' });
  pubUploadGraph = new ROSLIB.Topic({ ros, name: '/web_gui/upload_graph', messageType: 'std_msgs/String' });

  // --- Service client ---
  srvDownloadGraph = new ROSLIB.Service({ ros, name: '/web_gui/download_graph', serviceType: 'std_srvs/Trigger' });
}

function cleanupTopics() {
  [subOdom, subGoalStatus, subVizGraph, subVizPath, subVizNode].forEach(s => { if (s) try { s.unsubscribe(); } catch (e) { } });
  subOdom = subGoalStatus = subVizGraph = subVizPath = subVizNode = null;
  pubGoalPoint = pubJoy = pubAttemptable = pubUpdateVGraph = pubResetVGraph = pubUploadGraph = null;
  srvDownloadGraph = null;
}

/* ====================================================================
   ODOM CALLBACK
   ==================================================================== */

function onOdom(msg) {
  const p = msg.pose.pose.position;
  const q = msg.pose.pose.orientation;
  robotState.x = p.x;
  robotState.y = p.y;
  robotState.z = p.z;
  // Convert quaternion to yaw
  const siny = 2.0 * (q.w * q.z + q.x * q.y);
  const cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  robotState.yaw = Math.atan2(siny, cosy);

  // Update UI
  document.getElementById('robot-x').textContent = robotState.x.toFixed(2);
  document.getElementById('robot-y').textContent = robotState.y.toFixed(2);
  document.getElementById('robot-yaw').textContent = (robotState.yaw * 180 / Math.PI).toFixed(1) + '°';
  document.getElementById('goal-status').textContent = goalReached === null ? '—' : (goalReached ? '✓ Yes' : '✗ No');
}

/* ====================================================================
   MARKER CALLBACKS (VGraph + Path + GoalNodes)
   ==================================================================== */

function onVizGraph(msg) {
  // Only keep obstacle (polygon_edge) and boundary edges — skip freespace_vgraph (light blue)
  const edges = [];
  let foundValidEdges = false;
  for (const marker of msg.markers) {
    const ns = marker.ns;
    if (marker.type !== 5) continue;  // LINE_LIST only
    if (ns !== 'polygon_edge' && ns !== 'boundary_edge') continue;
    
    foundValidEdges = true;
    if (!marker.points || marker.points.length === 0) continue;

    for (let i = 0; i + 1 < marker.points.length; i += 2) {
      const a = marker.points[i], b = marker.points[i + 1];
      edges.push({ x1: a.x, y1: a.y, x2: b.x, y2: b.y, ns });
    }
  }
  
  if (foundValidEdges) {
    graphEdges = edges;
  }
}

function onVizPath(msg) {
  globalPath = (msg.points || []).map(p => ({ x: p.x, y: p.y }));
}

function onVizNode(msg) {
  const markers = [];
  for (const marker of msg.markers) {
    const pts = marker.points && marker.points.length ? marker.points :
      [marker.pose ? marker.pose.position : null];
    for (const pt of pts) {
      if (!pt) continue;
      markers.push({ x: pt.x, y: pt.y, ns: marker.ns });
    }
  }
  vizGoalMarkers = markers;
}

/* ====================================================================
   MAP CANVAS RENDERING
   ==================================================================== */

let _dragStart = null, _dragView = null;

function worldToScreen(wx, wy) {
  const sx = (wx - view.cx) * view.scale + canvas.width / 2;
  const sy = -(wy - view.cy) * view.scale + canvas.height / 2;
  return [sx, sy];
}

function screenToWorld(sx, sy) {
  const wx = (sx - canvas.width / 2) / view.scale + view.cx;
  const wy = -(sy - canvas.height / 2) / view.scale + view.cy;
  return [wx, wy];
}

function onWheel(e) {
  e.preventDefault();
  const factor = e.deltaY < 0 ? 1.15 : 1 / 1.15;
  view.scale = Math.max(1, Math.min(500, view.scale * factor));
  document.getElementById('map-zoom').textContent = 'Zoom: ' + view.scale.toFixed(1) + 'x';
}

function onMouseDown(e) {
  if (e.button === 0 && !goalMode) {
    _dragStart = { x: e.clientX, y: e.clientY };
    _dragView = { cx: view.cx, cy: view.cy };
    followRobot = false;
    document.getElementById('chk-follow-robot').checked = false;
  }
}

function onMouseMove(e) {
  // Cursor position
  const rect = canvas.getBoundingClientRect();
  const [wx, wy] = screenToWorld(e.clientX - rect.left, e.clientY - rect.top);
  document.getElementById('map-cursor').textContent = `Cursor: (${wx.toFixed(2)}, ${wy.toFixed(2)})`;

  // Drag to pan
  if (_dragStart) {
    const dx = (e.clientX - _dragStart.x) / view.scale;
    const dy = (e.clientY - _dragStart.y) / view.scale;
    view.cx = _dragView.cx - dx;
    view.cy = _dragView.cy + dy;
  }
}

function onMouseUp(e) {
  _dragStart = null;
  _dragView = null;
}

function onMapClick(e) {
  if (!goalMode || !connected) return;
  const rect = canvas.getBoundingClientRect();
  const [wx, wy] = screenToWorld(e.clientX - rect.left, e.clientY - rect.top);
  sendGoalPoint(wx, wy);
  cancelGoalMode();
}

function renderLoop() {
  render();
  requestAnimationFrame(renderLoop);
}

function render() {
  const W = canvas.width, H = canvas.height;
  ctx.clearRect(0, 0, W, H);

  // Follow robot
  if (followRobot && connected) {
    view.cx += (robotState.x - view.cx) * 0.1;
    view.cy += (robotState.y - view.cy) * 0.1;
  }

  // Draw grid
  drawGrid();

  // Draw obstacle edges
  if (showGraph && graphEdges.length) {
    for (const e of graphEdges) {
      const [x1, y1] = worldToScreen(e.x1, e.y1);
      const [x2, y2] = worldToScreen(e.x2, e.y2);
      ctx.beginPath();
      ctx.moveTo(x1, y1);
      ctx.lineTo(x2, y2);
      if (e.ns === 'boundary_edge') {
        ctx.strokeStyle = 'rgba(255, 165, 0, 0.6)';
        ctx.lineWidth = 1.5;
      } else {
        ctx.strokeStyle = 'rgba(255, 60, 60, 0.7)';
        ctx.lineWidth = 1.5;
      }
      ctx.stroke();
    }
  }

  // Draw global path
  if (showPath && globalPath.length > 1) {
    ctx.beginPath();
    const [px0, py0] = worldToScreen(globalPath[0].x, globalPath[0].y);
    ctx.moveTo(px0, py0);
    for (let i = 1; i < globalPath.length; i++) {
      const [px, py] = worldToScreen(globalPath[i].x, globalPath[i].y);
      ctx.lineTo(px, py);
    }
    ctx.strokeStyle = '#2ecc71';
    ctx.lineWidth = 3;
    ctx.stroke();
  }

  // Draw goal markers
  for (const m of vizGoalMarkers) {
    const [sx, sy] = worldToScreen(m.x, m.y);
    const ns = m.ns;
    if (ns === 'waypoint' || ns === 'original_goal' || ns === 'free_goal') {
      ctx.beginPath();
      ctx.arc(sx, sy, 8, 0, Math.PI * 2);
      ctx.fillStyle = ns === 'waypoint' ? '#e67e22' : '#e74c3c';
      ctx.fill();
      ctx.strokeStyle = '#fff';
      ctx.lineWidth = 2;
      ctx.stroke();
      // Cross
      ctx.beginPath();
      ctx.moveTo(sx - 5, sy); ctx.lineTo(sx + 5, sy);
      ctx.moveTo(sx, sy - 5); ctx.lineTo(sx, sy + 5);
      ctx.strokeStyle = '#fff';
      ctx.lineWidth = 1.5;
      ctx.stroke();
    }
  }

  // Draw robot
  if (connected) {
    drawRobot();
  }
}

function drawGrid() {
  const W = canvas.width, H = canvas.height;
  // Determine grid spacing based on zoom
  let gridSize = 1; // meters
  if (view.scale < 5) gridSize = 10;
  else if (view.scale < 15) gridSize = 5;
  else if (view.scale < 50) gridSize = 2;
  else if (view.scale > 150) gridSize = 0.5;

  const [wl, wt] = screenToWorld(0, 0);
  const [wr, wb] = screenToWorld(W, H);
  const xmin = Math.floor(Math.min(wl, wr) / gridSize) * gridSize;
  const xmax = Math.ceil(Math.max(wl, wr) / gridSize) * gridSize;
  const ymin = Math.floor(Math.min(wt, wb) / gridSize) * gridSize;
  const ymax = Math.ceil(Math.max(wt, wb) / gridSize) * gridSize;

  ctx.strokeStyle = 'rgba(255,255,255,0.06)';
  ctx.lineWidth = 0.5;
  for (let x = xmin; x <= xmax; x += gridSize) {
    const [sx] = worldToScreen(x, 0);
    ctx.beginPath(); ctx.moveTo(sx, 0); ctx.lineTo(sx, H); ctx.stroke();
  }
  for (let y = ymin; y <= ymax; y += gridSize) {
    const [, sy] = worldToScreen(0, y);
    ctx.beginPath(); ctx.moveTo(0, sy); ctx.lineTo(W, sy); ctx.stroke();
  }

  // Origin axes
  const [ox, oy] = worldToScreen(0, 0);
  ctx.strokeStyle = 'rgba(255, 80, 80, 0.3)';
  ctx.lineWidth = 1;
  ctx.beginPath(); ctx.moveTo(ox, 0); ctx.lineTo(ox, H); ctx.stroke(); // X axis (vertical line at x=0)
  ctx.strokeStyle = 'rgba(80, 255, 80, 0.3)';
  ctx.beginPath(); ctx.moveTo(0, oy); ctx.lineTo(W, oy); ctx.stroke(); // Y axis
}

function drawRobot() {
  const [sx, sy] = worldToScreen(robotState.x, robotState.y);
  const size = Math.max(10, 18 * view.scale / 30);
  const yaw = -robotState.yaw; // screen Y is flipped

  ctx.save();
  ctx.translate(sx, sy);
  ctx.rotate(yaw);

  // Body
  ctx.beginPath();
  ctx.moveTo(size, 0);
  ctx.lineTo(-size * 0.6, -size * 0.5);
  ctx.lineTo(-size * 0.3, 0);
  ctx.lineTo(-size * 0.6, size * 0.5);
  ctx.closePath();
  ctx.fillStyle = 'rgba(83, 216, 251, 0.85)';
  ctx.fill();
  ctx.strokeStyle = '#fff';
  ctx.lineWidth = 1.5;
  ctx.stroke();

  // Direction dot
  ctx.beginPath();
  ctx.arc(size * 0.4, 0, 2.5, 0, Math.PI * 2);
  ctx.fillStyle = '#fff';
  ctx.fill();

  ctx.restore();
}

function centerOnRobot() {
  view.cx = robotState.x;
  view.cy = robotState.y;
}

/* ====================================================================
   GOALPOINT
   ==================================================================== */

function toggleGoalMode() {
  goalMode = !goalMode;
  const btn = document.getElementById('btn-set-goal');
  const hint = document.getElementById('goalpoint-hint');
  btn.classList.toggle('active', goalMode);
  hint.classList.toggle('hidden', !goalMode);
  canvas.style.cursor = goalMode ? 'crosshair' : '';
}

function cancelGoalMode() {
  goalMode = false;
  document.getElementById('btn-set-goal').classList.remove('active');
  document.getElementById('goalpoint-hint').classList.add('hidden');
  canvas.style.cursor = '';
}

function sendGoalPoint(x, y) {
  if (!pubGoalPoint || !pubJoy) return;

  // First send joy message (same as RViz goalpoint tool)
  const joyMsg = new ROSLIB.Message({
    header: { stamp: { sec: 0, nanosec: 0 }, frame_id: 'web_gui' },
    axes: [0, 0, -1.0, 0, 1.0, 1.0, 0, 0],
    buttons: [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0]
  });
  pubJoy.publish(joyMsg);

  // Then send goalpoint
  const goalMsg = new ROSLIB.Message({
    header: { stamp: { sec: 0, nanosec: 0 }, frame_id: 'map' },
    point: { x: x, y: y, z: robotState.z }
  });
  pubGoalPoint.publish(goalMsg);

  // Publish twice (like the RViz tool does with usleep)
  setTimeout(() => pubGoalPoint.publish(goalMsg), 15);

  setGraphStatus(`Goal sent: (${x.toFixed(1)}, ${y.toFixed(1)})`);
}

/* ====================================================================
   PLANNER CONTROLS
   ==================================================================== */

function resetVGraph() {
  if (!pubResetVGraph) return;
  pubResetVGraph.publish(new ROSLIB.Message({}));
  setGraphStatus('Visibility graph reset.');
}

function resumeNav() {
  if (!pubJoy) return;
  const msg = new ROSLIB.Message({
    header: { stamp: { sec: 0, nanosec: 0 }, frame_id: 'teleop_panel' },
    axes: [0, 0, -1.0, 0, 1.0, 1.0, 0, 0],
    buttons: [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0]
  });
  pubJoy.publish(msg);
  setGraphStatus('Resume navigation sent.');
}

function publishAttemptable() {
  if (!pubAttemptable) return;
  const val = document.getElementById('chk-attemptable').checked;
  pubAttemptable.publish(new ROSLIB.Message({ data: val }));
}

function publishUpdateVGraph() {
  if (!pubUpdateVGraph) return;
  const val = document.getElementById('chk-update-vgraph').checked;
  pubUpdateVGraph.publish(new ROSLIB.Message({ data: val }));
}

/* ====================================================================
   GRAPH VGH DOWNLOAD / UPLOAD  (file lives on PC, not robot)
   ==================================================================== */

function downloadGraph() {
  if (!srvDownloadGraph) { setGraphStatus('Not connected.'); return; }
  setGraphStatus('Requesting graph…');

  srvDownloadGraph.callService(new ROSLIB.ServiceRequest({}), (result) => {
    if (result.success) {
      const blob = new Blob([result.message], { type: 'text/plain' });
      const url = URL.createObjectURL(blob);
      const a = document.createElement('a');
      a.href = url;
      const ts = new Date().toISOString().replace(/[:.]/g, '-');
      a.download = `far_graph_${ts}.vgh`;
      document.body.appendChild(a);
      a.click();
      document.body.removeChild(a);
      URL.revokeObjectURL(url);
      setGraphStatus('Graph downloaded to PC.');
    } else {
      setGraphStatus('Download failed: ' + result.message);
    }
  }, (err) => {
    setGraphStatus('Service error: ' + err);
  });
}

function uploadGraph(event) {
  const file = event.target.files[0];
  if (!file) return;
  if (!pubUploadGraph) { setGraphStatus('Not connected.'); return; }

  const reader = new FileReader();
  reader.onload = (e) => {
    const content = e.target.result;
    pubUploadGraph.publish(new ROSLIB.Message({ data: content }));
    setGraphStatus(`Graph uploaded: ${file.name}`);
  };
  reader.readAsText(file);
  event.target.value = '';
}

function setGraphStatus(text) {
  document.getElementById('graph-status').textContent = text;
  // Auto-clear after 6s
  setTimeout(() => {
    const el = document.getElementById('graph-status');
    if (el.textContent === text) el.textContent = '';
  }, 6000);
}

/* ====================================================================
   SAFETY SHUTDOWN
   ==================================================================== */

let shutdownTriggered = false;

function safetyShutdown() {
  if (shutdownTriggered) return;
  if (!pubJoy || !connected) {
    document.getElementById('shutdown-status').textContent = 'Not connected!';
    return;
  }

  shutdownTriggered = true;
  console.error('EMERGENCY SHUTDOWN triggered!');

  // Stop any active teleop
  teleopActive = false;
  teleopFwd = 0;
  teleopYaw = 0;
  if (teleopInterval) {
    clearInterval(teleopInterval);
    teleopInterval = null;
  }

  // Send shutdown joy message (buttons[8] = 1) multiple times for redundancy
  const shutdownMsg = new ROSLIB.Message({
    header: { stamp: { sec: 0, nanosec: 0 }, frame_id: 'web_gui_emergency' },
    axes: [0, 0, -1.0, 0, 0, 1.0, 0, 0],
    buttons: [0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0]
  });

  // Publish 5 times at 50ms intervals for network reliability
  let count = 0;
  const sendInterval = setInterval(() => {
    pubJoy.publish(shutdownMsg);
    count++;
    if (count >= 5) {
      clearInterval(sendInterval);
    }
  }, 50);

  // Update UI
  const btn = document.getElementById('btn-safety-shutdown');
  btn.textContent = 'SHUTDOWN SENT';
  btn.classList.add('triggered');
  document.getElementById('shutdown-status').textContent = 'Robot is shutting down (StandDown)...';

  // Expose the reboot button
  document.getElementById('btn-reboot').classList.remove('hidden');
}

function rebootRobot() {
  if (!shutdownTriggered) return;
  if (!pubJoy || !pubGoalPoint || !connected) {
    document.getElementById('shutdown-status').textContent = 'Not connected!';
    return;
  }

  console.log('REBOOT triggered!');

  // Send reboot joy message (buttons[9] = 1)
  const rebootMsg = new ROSLIB.Message({
    header: { stamp: { sec: 0, nanosec: 0 }, frame_id: 'web_gui_emergency' },
    axes: [0, 0, -1.0, 0, 0, 1.0, 0, 0],
    buttons: [0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0]
  });
  pubJoy.publish(rebootMsg);

  // // Send an empty point to clear planner goals
  // const emptyGoalMsg = new ROSLIB.Message({
  //   header: { stamp: { sec: 0, nanosec: 0 }, frame_id: 'map' },
  //   point: { x: robotState.x, y: robotState.y, z: robotState.z }
  // });
  // pubGoalPoint.publish(emptyGoalMsg);

  // Update UI state
  shutdownTriggered = false;
  const btn = document.getElementById('btn-safety-shutdown');
  btn.textContent = 'EMERGENCY SHUTDOWN';
  btn.classList.remove('triggered');
  document.getElementById('shutdown-status').textContent = '';
  document.getElementById('btn-reboot').classList.add('hidden');
}



/* ====================================================================
   TELEOP (virtual joystick)
   ==================================================================== */

function initTeleopStick() {
  const stickCanvas = document.getElementById('teleop-stick');
  const stickCtx = stickCanvas.getContext('2d');
  const W = stickCanvas.width, H = stickCanvas.height;
  const cx = W / 2, cy = H / 2, maxR = W / 2 - 12;
  let stickX = 0, stickY = 0; // normalized -1..1
  let dragging = false;

  function drawStick() {
    stickCtx.clearRect(0, 0, W, H);
    // Outer ring
    stickCtx.beginPath();
    stickCtx.arc(cx, cy, maxR, 0, Math.PI * 2);
    stickCtx.strokeStyle = 'rgba(83, 216, 251, 0.4)';
    stickCtx.lineWidth = 2;
    stickCtx.stroke();
    // Cross lines
    stickCtx.strokeStyle = 'rgba(255,255,255,0.1)';
    stickCtx.lineWidth = 1;
    stickCtx.beginPath(); stickCtx.moveTo(cx, cy - maxR); stickCtx.lineTo(cx, cy + maxR); stickCtx.stroke();
    stickCtx.beginPath(); stickCtx.moveTo(cx - maxR, cy); stickCtx.lineTo(cx + maxR, cy); stickCtx.stroke();
    // Knob
    const knobX = cx + stickX * maxR;
    const knobY = cy - stickY * maxR;
    stickCtx.beginPath();
    stickCtx.arc(knobX, knobY, 16, 0, Math.PI * 2);
    stickCtx.fillStyle = dragging ? 'rgba(83, 216, 251, 0.9)' : 'rgba(83, 216, 251, 0.5)';
    stickCtx.fill();
    stickCtx.strokeStyle = '#fff';
    stickCtx.lineWidth = 2;
    stickCtx.stroke();
  }
  drawStick();

  function updateStick(e) {
    const rect = stickCanvas.getBoundingClientRect();
    let dx, dy;
    if (e.touches) {
      dx = e.touches[0].clientX - rect.left - cx;
      dy = -(e.touches[0].clientY - rect.top - cy);
    } else {
      dx = e.clientX - rect.left - cx;
      dy = -(e.clientY - rect.top - cy);
    }
    let len = Math.sqrt(dx * dx + dy * dy);
    if (len > maxR) { dx = dx / len * maxR; dy = dy / len * maxR; }
    stickX = dx / maxR; // left/right → yaw
    stickY = dy / maxR; // up/down → forward
    teleopFwd = stickY;
    teleopYaw = stickX;
    document.getElementById('teleop-fwd').textContent = teleopFwd.toFixed(2);
    document.getElementById('teleop-yaw').textContent = teleopYaw.toFixed(2);
    drawStick();
  }

  function startDrag(e) {
    e.preventDefault();
    dragging = true;
    teleopActive = true;
    updateStick(e);
    if (!teleopInterval) {
      teleopInterval = setInterval(sendTeleopJoy, 100);
    }
  }

  function moveDrag(e) {
    if (!dragging) return;
    e.preventDefault();
    updateStick(e);
  }

  function stopDrag() {
    if (!dragging) return;
    dragging = false;
    teleopActive = false;
    stickX = 0; stickY = 0;
    teleopFwd = 0; teleopYaw = 0;
    document.getElementById('teleop-fwd').textContent = '0.00';
    document.getElementById('teleop-yaw').textContent = '0.00';
    drawStick();
    // Send one zero command, then stop interval
    sendTeleopJoy();
    if (teleopInterval) {
      clearInterval(teleopInterval);
      teleopInterval = null;
    }
  }

  stickCanvas.addEventListener('mousedown', startDrag);
  stickCanvas.addEventListener('touchstart', startDrag, { passive: false });
  window.addEventListener('mousemove', moveDrag);
  window.addEventListener('touchmove', moveDrag, { passive: false });
  window.addEventListener('mouseup', stopDrag);
  window.addEventListener('touchend', stopDrag);
}

function sendTeleopJoy() {
  if (!pubJoy || !connected) return;
  // axes[0] = manual yaw (positive=left, negative=right)
  // axes[4] = manual fwd/back (positive=forward, negative=backward)
  // axes[2] = 1.0 → autonomy OFF
  // axes[5] = -1.0 when stick active → manual mode ON (allows backward)
  //         = 1.0  when stick released → manual mode OFF (back to normal)
  // buttons[7] = 1 (teleop panel flag)
  const manualTrigger = teleopActive ? -1.0 : 1.0;
  const msg = new ROSLIB.Message({
    header: { stamp: { sec: 0, nanosec: 0 }, frame_id: 'teleop_panel' },
    axes: [-teleopYaw, 0, 1.0, 0, teleopFwd, manualTrigger, 0, 0],
    buttons: [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0]
  });
  pubJoy.publish(msg);
}
