// Auto-generated from index.html
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Scuba Control</title>
  <style>
    :root {
      --bg: #0f1720;
      --panel: #182634;
      --panel-border: #263a4d;
      --text: #e6edf3;
      --text-dim: #8b9bab;
      --accent: #3fb950;
      --accent-dim: #1f6feb;
      --danger: #f85149;
      --warn: #d29922;
    }

    * { box-sizing: border-box; }

    body {
      font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Arial, sans-serif;
      background: var(--bg);
      color: var(--text);
      margin: 0;
      padding: 16px;
    }

    h1 {
      font-size: 1.4rem;
      margin: 0 0 16px;
      text-align: center;
    }

    .grid {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(280px, 1fr));
      gap: 12px;
      max-width: 1000px;
      margin: 0 auto;
    }

    .card {
      background: var(--panel);
      border: 1px solid var(--panel-border);
      border-radius: 10px;
      padding: 14px 16px;
    }

    .card h2 {
      font-size: 0.85rem;
      text-transform: uppercase;
      letter-spacing: 0.05em;
      color: var(--text-dim);
      margin: 0 0 10px;
    }

    .row {
      display: flex;
      justify-content: space-between;
      align-items: center;
      padding: 4px 0;
      font-size: 0.95rem;
    }

    .row .label { color: var(--text-dim); }
    .row .value { font-weight: 600; }

    .badge {
      display: inline-block;
      padding: 2px 8px;
      border-radius: 999px;
      font-size: 0.75rem;
      font-weight: 600;
    }
    .badge.ok { background: rgba(63, 185, 80, 0.15); color: var(--accent); }
    .badge.warn { background: rgba(210, 153, 34, 0.15); color: var(--warn); }
    .badge.off { background: rgba(139, 155, 171, 0.15); color: var(--text-dim); }
    .badge.danger { background: rgba(248, 81, 73, 0.15); color: var(--danger); }

    button, select, input[type=number] {
      font-family: inherit;
      font-size: 0.9rem;
      border-radius: 6px;
      border: 1px solid var(--panel-border);
      background: #1c2b3a;
      color: var(--text);
      padding: 8px 10px;
    }

    button {
      cursor: pointer;
      background: var(--accent-dim);
      border-color: var(--accent-dim);
      color: white;
      font-weight: 600;
    }
    button:hover { filter: brightness(1.1); }
    button.secondary { background: #1c2b3a; border-color: var(--panel-border); color: var(--text); }
    button.danger { background: var(--danger); border-color: var(--danger); }
    button.secondary.active { background: var(--accent); border-color: var(--accent); color: #0f1720; }

    .btn-row { display: flex; gap: 8px; flex-wrap: wrap; margin-top: 8px; }
    .btn-row > * { flex: 1; min-width: 90px; }

    label { font-size: 0.85rem; color: var(--text-dim); display: block; margin-bottom: 4px; }
    .field { margin-bottom: 10px; }
    .field input, .field select { width: 100%; }

    /* 3D robot body (Dolphin Sprite-shaped: wide, flat) built from 6 CSS
       faces - no WebGL/3D library needed, just perspective + preserve-3d. */
    .scene3d {
      width: 180px;
      height: 180px;
      margin: 0 auto;
      perspective: 500px;
      display: flex;
      align-items: center;
      justify-content: center;
    }
    .robot3d {
      position: relative;
      width: 130px;
      height: 36px;
      transform-style: preserve-3d;
      transition: transform 0.15s ease-out;
    }
    .robot3d .face {
      position: absolute;
      border: 1px solid rgba(0, 0, 0, 0.35);
      border-radius: 4px;
    }
    .robot3d .front  { width: 130px; height: 36px; left: 0; top: 0; background: #e5533d; transform: translateZ(40px); }
    .robot3d .front .arrow {
      position: absolute;
      left: 50%;
      top: 50%;
      width: 0;
      height: 0;
      border-left: 10px solid transparent;
      border-right: 10px solid transparent;
      border-bottom: 16px solid #fff;
      transform: translate(-50%, -50%);
    }
    .robot3d .back   { width: 130px; height: 36px; left: 0; top: 0; background: #123246; transform: rotateY(180deg) translateZ(40px); }
    .robot3d .left   { width: 80px; height: 36px; left: 25px; top: 0; background: #16456a; transform: rotateY(-90deg) translateZ(65px); }
    .robot3d .right  { width: 80px; height: 36px; left: 25px; top: 0; background: #16456a; transform: rotateY(90deg) translateZ(65px); }
    .robot3d .top    { width: 130px; height: 80px; left: 0; top: -22px; background: #2b6ea8; transform: rotateX(90deg) translateZ(18px); }
    .robot3d .bottom { width: 130px; height: 80px; left: 0; top: -22px; background: #0a1e2e; transform: rotateX(90deg) translateZ(-18px); }

    #cleaningMap {
      border: 1px solid var(--panel-border);
      border-radius: 6px;
      display: block;
      margin: 0 auto;
      max-width: 100%;
    }

    #logArea {
      background: #0a0f15;
      color: #7ee787;
      font-family: "SF Mono", Consolas, monospace;
      font-size: 0.75rem;
      height: 180px;
      overflow: auto;
      padding: 8px;
      border-radius: 6px;
      white-space: pre-wrap;
      margin: 0;
    }

    .section-title {
      max-width: 1000px;
      margin: 24px auto 10px;
      padding-left: 2px;
      font-size: 0.95rem;
      font-weight: 700;
      color: var(--text-dim);
      text-transform: uppercase;
      letter-spacing: 0.06em;
      border-bottom: 1px solid var(--panel-border);
      padding-bottom: 6px;
    }
    .section-title:first-of-type { margin-top: 0; }

    .errors-list { max-height: 160px; overflow: auto; }
    .error-row { display: flex; justify-content: space-between; font-size: 0.85rem; padding: 4px 0; border-bottom: 1px solid var(--panel-border); }
    .error-row:last-child { border-bottom: none; }
    .empty-hint { color: var(--text-dim); font-size: 0.85rem; font-style: italic; }
  </style>
</head>
<body>
  <h1>🐬 Scuba Control</h1>

  <div class="section-title">Overview</div>
  <div class="grid">

    <div class="card">
      <h2>Status</h2>
      <div class="row"><span class="label">State</span><span class="value" id="status"><span class="badge off">-</span></span></div>
      <div class="row"><span class="label">Angle</span><span class="value" id="angle">-</span></div>
      <div class="row"><span class="label">Yaw</span><span class="value" id="yaw">-</span></div>
      <div class="row"><span class="label">IMU</span><span class="value" id="imuName">-</span></div>
      <div class="row"><span class="label">Magnetometer</span><span class="value" id="imuMag">-</span></div>
    </div>

    <div class="card">
      <h2>Control</h2>
      <div class="field">
        <label for="duration">Run for (minutes, 0 = unlimited)</label>
        <input type="number" id="duration" value="0" min="0">
      </div>
      <div class="btn-row">
        <button onclick="controlRobot('start')">Start</button>
        <button class="secondary" data-state="STOPPED" onclick="controlRobot('stop')">Stop</button>
      </div>
      <div class="btn-row" style="margin-top:8px">
        <button class="secondary" data-state="MOVING_FORWARD" onclick="controlRobot('forward')">Forward</button>
        <button class="secondary" data-state="MOVING_BACKWARD" onclick="controlRobot('backward')">Backward</button>
        <button class="secondary" data-state="TURNING" onclick="controlRobot('turn')">Turn</button>
      </div>
      <p class="empty-hint">Forward/Backward nudge for a few seconds, then stop on their own.</p>
      <div class="btn-row" style="margin-top:8px">
        <button class="danger" data-state="MAINTENANCE" onclick="controlRobot('maintenance')">Enter maintenance</button>
      </div>
      <p class="empty-hint">Stops both motors and holds until Start is pressed - lets the Maintenance card's water motor ramp test run without flipping the robot over.</p>
      <div class="row" style="margin-top:10px"><span class="label">Session</span><span class="value" id="session">-</span></div>
    </div>

  </div>

  <div class="section-title">Attitude</div>
  <div class="grid">

    <div class="card">
      <h2>Attitude</h2>
      <div class="scene3d">
        <div class="robot3d" id="robot3d">
          <div class="face top"></div>
          <div class="face bottom"></div>
          <div class="face front"><div class="arrow"></div></div>
          <div class="face back"></div>
          <div class="face left"></div>
          <div class="face right"></div>
        </div>
      </div>
      <div class="row"><span class="label">Pitch</span><span class="value" id="pitchVal">-</span></div>
      <div class="row"><span class="label">Roll</span><span class="value" id="rollVal">-</span></div>
    </div>

    <div class="card">
      <h2>Heading</h2>
      <svg viewBox="0 0 200 200" width="180" height="180" style="display:block;margin:0 auto;">
        <circle cx="100" cy="100" r="94" fill="#0a0f15" stroke="var(--panel-border)" stroke-width="3"/>
        <text x="100" y="18" text-anchor="middle" fill="#8b9bab" font-size="14">N</text>
        <text x="184" y="105" text-anchor="middle" fill="#8b9bab" font-size="14">E</text>
        <text x="100" y="194" text-anchor="middle" fill="#8b9bab" font-size="14">S</text>
        <text x="16" y="105" text-anchor="middle" fill="#8b9bab" font-size="14">W</text>
        <g id="headingArrow">
          <polygon points="100,25 90,115 100,98 110,115" fill="#f85149"/>
        </g>
        <circle cx="100" cy="100" r="4" fill="#e6edf3"/>
      </svg>
      <div class="row"><span class="label">Yaw</span><span class="value" id="headingYaw">-</span></div>
    </div>

  </div>

  <div class="section-title">Settings</div>
  <div class="grid">

    <div class="card">
      <h2>Robot</h2>
      <div class="field">
        <label for="turnStrategy">Turn strategy</label>
        <select id="turnStrategy">
          <option value="legacy">legacy (raw gyro)</option>
          <option value="duration">duration (fixed pulse)</option>
          <option value="kalman">kalman (filtered gyro)</option>
        </select>
      </div>
      <div class="field">
        <label for="statsSaveInterval">Stats save interval (minutes, 0 = only on session end)</label>
        <input type="number" id="statsSaveInterval" value="10" min="0">
      </div>
      <button onclick="saveConfig()">Save settings</button>
    </div>

    <div class="card">
      <h2>MQTT / Home Assistant</h2>
      <div class="field">
        <label><input type="checkbox" id="mqttEnabled" style="width:auto"> Enabled</label>
      </div>
      <div class="field">
        <label for="mqttHost">Broker host</label>
        <input type="text" id="mqttHost" placeholder="192.168.1.10">
      </div>
      <div class="field">
        <label for="mqttPort">Broker port</label>
        <input type="number" id="mqttPort" value="1883">
      </div>
      <div class="field">
        <label for="mqttUser">User (optional)</label>
        <input type="text" id="mqttUser">
      </div>
      <div class="field">
        <label for="mqttPassword">Password (optional, leave blank to keep current)</label>
        <input type="password" id="mqttPassword">
      </div>
      <div class="field">
        <label for="mqttTopicPrefix">Topic prefix</label>
        <input type="text" id="mqttTopicPrefix" placeholder="scuba">
      </div>
      <div class="btn-row">
        <button onclick="saveMqtt()">Save</button>
        <button class="danger" onclick="resetMqtt()">Reset</button>
      </div>
    </div>

  </div>

  <div class="section-title">Tuning</div>
  <div class="grid">

    <div class="card">
      <h2>Speeds</h2>
      <div class="field">
        <label for="t_movimientoMoveSpeed">Forward speed</label>
        <input type="number" id="t_movimientoMoveSpeed" step="1">
      </div>
      <div class="field">
        <label for="t_movimientoMoveBackwardsSpeed">Backward speed</label>
        <input type="number" id="t_movimientoMoveBackwardsSpeed" step="1">
      </div>
      <div class="field">
        <label for="t_movimientoIdleSpeed">Turning/idle speed</label>
        <input type="number" id="t_movimientoIdleSpeed" step="1">
      </div>
      <div class="field">
        <label for="t_aguaTurnSpeed">Water motor - turning</label>
        <input type="number" id="t_aguaTurnSpeed" step="1">
      </div>
      <div class="field">
        <label for="t_aguaMoveSpeed">Water motor - moving</label>
        <input type="number" id="t_aguaMoveSpeed" step="1">
      </div>
      <div class="field">
        <label for="t_aguaIdleSpeed">Water motor - idle</label>
        <input type="number" id="t_aguaIdleSpeed" step="1">
      </div>
      <div class="btn-row">
        <button onclick="saveTuning()">Save</button>
        <button class="danger" onclick="resetTuning()">Reset to defaults</button>
      </div>
    </div>

    <div class="card">
      <h2>Angles &amp; timing</h2>
      <div class="field">
        <label for="t_wallAngleThreshold">Wall detection angle (deg)</label>
        <input type="number" id="t_wallAngleThreshold" step="1">
      </div>
      <div class="field">
        <label for="t_wallAngleRecoverThreshold">Wall recovery angle (deg)</label>
        <input type="number" id="t_wallAngleRecoverThreshold" step="1">
      </div>
      <div class="field">
        <label for="t_floorInclinationPrecision">Upright precision (deg)</label>
        <input type="number" id="t_floorInclinationPrecision" step="1">
      </div>
      <div class="field">
        <label for="t_turnAngleDeg">Turn angle (deg)</label>
        <input type="number" id="t_turnAngleDeg" step="1">
      </div>
      <div class="field">
        <label for="t_upsideDownThreshold">Upside-down detection (accel X, g)</label>
        <input type="number" id="t_upsideDownThreshold" step="0.05" min="0" max="1">
      </div>
      <div class="field">
        <label for="t_movingTimeoutMs">Movement timeout (ms)</label>
        <input type="number" id="t_movingTimeoutMs" step="1000">
      </div>
      <div class="field">
        <label for="t_maxTimeTurningMs">Max turning time (ms)</label>
        <input type="number" id="t_maxTimeTurningMs" step="1000">
      </div>
      <div class="field">
        <label for="t_delayAutostartMs">Autostart delay (ms)</label>
        <input type="number" id="t_delayAutostartMs" step="1000">
      </div>
      <div class="field">
        <label for="t_turnDurationMs">Fixed turn duration (ms)</label>
        <input type="number" id="t_turnDurationMs" step="100">
      </div>
      <div class="field">
        <label for="t_attitudeSmoothingAlpha">Attitude smoothing (0-1, lower = smoother)</label>
        <input type="number" id="t_attitudeSmoothingAlpha" step="0.05" min="0" max="1">
      </div>
      <div class="field">
        <label for="t_manualActionDurationMs">Manual forward/backward nudge duration (ms)</label>
        <input type="number" id="t_manualActionDurationMs" step="100">
      </div>
      <div class="btn-row">
        <button onclick="saveTuning()">Save</button>
        <button class="danger" onclick="resetTuning()">Reset to defaults</button>
      </div>
    </div>

  </div>

  <div class="section-title">Diagnostics</div>
  <div class="grid">

    <div class="card">
      <h2>Maintenance</h2>
      <div class="row"><span class="label">Firmware</span><span class="value" id="firmwareCommit">-</span></div>
      <div class="row"><span class="label">Boots</span><span class="value" id="bootCount">-</span></div>
      <div class="row"><span class="label">Total runtime</span><span class="value" id="totalRuntime">-</span></div>
      <p class="empty-hint">Motor ramp tests only run while flipped into Maintenance state.</p>
      <div class="btn-row">
        <button onclick="maintenanceAction('rampAgua')">Ramp water motor</button>
      </div>
      <div class="btn-row">
        <button onclick="maintenanceAction('rampMovimientoForward')">Ramp forward</button>
        <button onclick="maintenanceAction('rampMovimientoBackward')">Ramp backward</button>
      </div>
      <div class="btn-row">
        <button class="danger" onclick="maintenanceAction('resetStats')">Reset stats</button>
      </div>
      <div class="btn-row">
        <button class="danger" onclick="maintenanceReboot()">Reboot</button>
        <button class="danger" onclick="maintenanceFactoryReset()">Factory reset</button>
      </div>
    </div>

    <div class="card">
      <h2>Accelerometer calibration</h2>
      <div class="row"><span class="label">Status</span><span class="value" id="calStatus">-</span></div>
      <div class="row"><span class="label">Zero offset</span><span class="value" id="calOffset">-</span></div>
      <p class="empty-hint">Rest the robot flat before calibrating.</p>
      <div class="btn-row">
        <button onclick="calibrate()">Calibrate zero</button>
        <button class="danger" onclick="clearCalibration()">Clear</button>
      </div>
    </div>

    <div class="card">
      <h2>Fault log</h2>
      <div class="errors-list" id="errorsList"><span class="empty-hint">Loading…</span></div>
      <div class="btn-row">
        <button class="danger" onclick="clearErrors()">Clear all</button>
      </div>
    </div>

  </div>

  <div class="section-title">Map &amp; Logs</div>
  <div class="grid">
    <div class="card">
      <h2>Cleaning progress</h2>
      <canvas id="cleaningMap" width="300" height="300"></canvas>
    </div>

    <div class="card">
      <h2>Logs</h2>
      <pre id="logArea"></pre>
    </div>
  </div>

  <script>
    var GRID_SIZE = 30;

    function get(url) {
      return fetch(url).then(r => r.json());
    }

    function controlRobot(action) {
      var url = "/control?action=" + action;
      if (action === "start") {
        var minutes = document.getElementById("duration").value || 0;
        url += "&duration=" + minutes;
      }
      fetch(url);
    }

    function saveConfig() {
      var strategy = document.getElementById("turnStrategy").value;
      var interval = document.getElementById("statsSaveInterval").value || 0;
      get("/config?turnStrategy=" + strategy + "&statsSaveInterval=" + interval);
    }

    function calibrate() {
      get("/calibrate").then(refreshStatus);
    }

    function clearCalibration() {
      get("/calibrate?action=clear").then(refreshStatus);
    }

    function clearErrors() {
      fetch("/errors?action=clear").then(refreshErrors);
    }

    function maintenanceAction(action) {
      fetch("/maintenance?action=" + action).then(function(r) {
        if (!r.ok) return r.text().then(function(t) { alert(t); });
        refreshStatus();
      });
    }

    function maintenanceReboot() {
      if (!confirm("Reboot the robot now?")) return;
      fetch("/maintenance?action=reboot");
    }

    function maintenanceFactoryReset() {
      if (!confirm("Factory reset: wipes MQTT config, tuning, calibration and fault log, resets stats, then reboots. Continue?")) return;
      fetch("/maintenance?action=factoryReset");
    }

    function saveMqtt() {
      var params = new URLSearchParams({
        enabled: document.getElementById("mqttEnabled").checked ? "1" : "0",
        host: document.getElementById("mqttHost").value,
        port: document.getElementById("mqttPort").value || 1883,
        user: document.getElementById("mqttUser").value,
        topicPrefix: document.getElementById("mqttTopicPrefix").value || "scuba"
      });
      var password = document.getElementById("mqttPassword").value;
      if (password) params.set("password", password);
      get("/mqtt?" + params.toString()).then(refreshMqtt);
    }

    function resetMqtt() {
      get("/mqtt?action=reset").then(refreshMqtt);
    }

    function refreshMqtt() {
      get("/mqtt").then(function(data) {
        document.getElementById("mqttEnabled").checked = data.enabled;
        document.getElementById("mqttHost").value = data.host;
        document.getElementById("mqttPort").value = data.port;
        document.getElementById("mqttUser").value = data.user;
        document.getElementById("mqttTopicPrefix").value = data.topicPrefix;
        document.getElementById("mqttPassword").value = "";
        document.getElementById("mqttPassword").placeholder = data.hasPassword ? "(set - leave blank to keep)" : "";
      });
    }

    var STATE_BADGE_CLASS = {
      MOVING_FORWARD: "ok",
      MOVING_BACKWARD: "ok",
      TURNING: "ok",
      STARTING: "warn",
      STOPPED: "off",
      MAINTENANCE: "danger"
    };

    function refreshStatus() {
      get("/status").then(function(data) {
        var badgeClass = STATE_BADGE_CLASS[data.state] || "off";
        document.getElementById("status").innerHTML = '<span class="badge ' + badgeClass + '">' + data.state + '</span>';
        document.querySelectorAll("[data-state]").forEach(function(btn) {
          btn.classList.toggle("active", btn.getAttribute("data-state") === data.state);
        });
        document.getElementById("angle").innerHTML = data.angle.toFixed(1) + "°";
        document.getElementById("yaw").innerHTML = data.yaw.toFixed(1) + "°";

        document.getElementById("pitchVal").innerHTML = data.pitchDeg.toFixed(1) + "°";
        document.getElementById("rollVal").innerHTML = data.rollDeg.toFixed(1) + "°";
        document.getElementById("robot3d").style.transform =
          "rotateX(" + (-data.pitchDeg) + "deg) rotateY(" + data.rollDeg + "deg)";

        document.getElementById("headingYaw").innerHTML = data.yaw.toFixed(1) + "°";
        document.getElementById("headingArrow").setAttribute(
          "transform", "rotate(" + data.yaw + " 100 100)"
        );
        document.getElementById("imuName").innerHTML = data.imuName;
        document.getElementById("imuMag").innerHTML = data.imuHasMagnetometer
          ? '<span class="badge ok">present</span>'
          : '<span class="badge off">absent</span>';

        document.getElementById("session").innerHTML =
          data.sessionDurationMinutes > 0
            ? (data.sessionElapsedSeconds / 60).toFixed(1) + " / " + data.sessionDurationMinutes + " min"
            : (data.sessionElapsedSeconds / 60).toFixed(1) + " min (unlimited)";

        document.getElementById("firmwareCommit").innerHTML = data.firmwareCommit;
        document.getElementById("bootCount").innerHTML = data.maintenance.bootCount;
        document.getElementById("totalRuntime").innerHTML = data.maintenance.totalRuntimeHours.toFixed(1) + " h";

        document.getElementById("calStatus").innerHTML = data.accelCalibrated
          ? '<span class="badge ok">calibrated</span>'
          : '<span class="badge warn">uncalibrated</span>';
        document.getElementById("calOffset").innerHTML = data.accelZeroOffset.toFixed(3);

        drawMap(data.map, data.x, data.y);
      });
    }

    function refreshConfig() {
      get("/config").then(function(data) {
        document.getElementById("turnStrategy").value = data.turnStrategy;
        document.getElementById("statsSaveInterval").value = data.statsSaveIntervalMinutes;
      });
    }

    var TUNING_FIELDS = [
      "movimientoMoveSpeed", "movimientoMoveBackwardsSpeed", "movimientoIdleSpeed",
      "aguaTurnSpeed", "aguaMoveSpeed", "aguaIdleSpeed",
      "wallAngleThreshold", "wallAngleRecoverThreshold", "floorInclinationPrecision", "turnAngleDeg",
      "upsideDownThreshold",
      "movingTimeoutMs", "maxTimeTurningMs", "delayAutostartMs", "turnDurationMs",
      "attitudeSmoothingAlpha", "manualActionDurationMs"
    ];

    function saveTuning() {
      var params = {};
      TUNING_FIELDS.forEach(function(f) {
        params[f] = document.getElementById("t_" + f).value;
      });
      get("/tuning?" + new URLSearchParams(params).toString()).then(refreshTuning);
    }

    function resetTuning() {
      get("/tuning?action=reset").then(refreshTuning);
    }

    function refreshTuning() {
      get("/tuning").then(function(data) {
        TUNING_FIELDS.forEach(function(f) {
          document.getElementById("t_" + f).value = data[f];
        });
      });
    }

    function refreshErrors() {
      get("/errors").then(function(data) {
        var el = document.getElementById("errorsList");
        if (!data.entries.length) {
          el.innerHTML = '<span class="empty-hint">No faults logged.</span>';
          return;
        }
        el.innerHTML = data.entries.map(function(e) {
          var badge = e.active ? '<span class="badge warn">active</span>' : '<span class="badge off">cleared</span>';
          return '<div class="error-row"><span>' + e.name + '</span>' + badge + '</div>';
        }).join("");
      });
    }

    function drawMap(mapData, robotX, robotY) {
      var canvas = document.getElementById("cleaningMap");
      var ctx = canvas.getContext("2d");
      var cellSize = canvas.width / GRID_SIZE;

      ctx.fillStyle = "#0a0f15";
      ctx.fillRect(0, 0, canvas.width, canvas.height);

      ctx.fillStyle = "#3fb950";
      for (var y = 0; y < GRID_SIZE; y++) {
        for (var x = 0; x < GRID_SIZE; x++) {
          if (mapData[y][x]) {
            ctx.fillRect(x * cellSize, y * cellSize, cellSize, cellSize);
          }
        }
      }

      ctx.fillStyle = "#f85149";
      ctx.fillRect(robotX * cellSize, robotY * cellSize, cellSize, cellSize);
    }

    function updateLogs() {
      fetch('/logs')
        .then(r => r.text())
        .then(txt => { document.getElementById('logArea').textContent = txt; });
    }

    setInterval(refreshStatus, 1000);
    setInterval(updateLogs, 2000);
    setInterval(refreshErrors, 5000);

    refreshStatus();
    refreshConfig();
    refreshErrors();
    refreshMqtt();
    refreshTuning();
    updateLogs();
  </script>
</body>
</html>

)rawliteral";
