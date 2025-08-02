// Auto-generated from index.html
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body { font-family: Arial; text-align: center; margin:0px auto; padding-top: 30px; }
    table { margin-left: auto; margin-right: auto; }
    .button { padding: 10px 20px; font-size: 24px; border-radius: 5px; }
    #cleaningMap { border: 1px solid black; margin: 20px auto; }
  </style>
</head>
<body>
  <h1>Robot Vacuum Control</h1>
  <p>Status: <span id="status">Stopped</span></p>
  <p>Angle: <span id="angle">0°</span></p>
  <p>Yaw: <span id="yaw">0°</span></p>

  <button class="button" onclick="controlRobot('start')">Start</button>
  <button class="button" onclick="controlRobot('stop')">Stop</button>
  <button class="button" onclick="controlRobot('turn')">Turn</button>

  <h2>Cleaning Progress</h2>
  <canvas id="cleaningMap" width="300" height="300"></canvas>
  <div>
    <h3>Logs</h3>
    <pre id="logArea" style="background:#222;color:#0f0;height:200px;overflow:auto"></pre>
  </div>
  <script>
    function controlRobot(action) {
      var xhr = new XMLHttpRequest();
      xhr.open("GET", "/control?action=" + action, true);
      xhr.send();
    }

    setInterval(function() {
      var xhr = new XMLHttpRequest();
      xhr.onreadystatechange = function() {
        if (this.readyState == 4 && this.status == 200) {
          var data = JSON.parse(this.responseText);
          document.getElementById("status").innerHTML = data.state;
          document.getElementById("angle").innerHTML = data.angle + "°";
          document.getElementById("yaw").innerHTML = data.yaw + "°";
          drawMap(data.map);
        }
      };
      xhr.open("GET", "/status", true);
      xhr.send();
    }, 1000);

    function drawMap(mapData) {
      var canvas = document.getElementById("cleaningMap");
      var ctx = canvas.getContext("2d");
      var cellSize = canvas.width / 20;

      ctx.clearRect(0, 0, canvas.width, canvas.height);

      for (var y = 0; y < 20; y++) {
        for (var x = 0; x < 20; x++) {
          if (mapData[y][x]) {
            ctx.fillStyle = "green";
            ctx.fillRect(x * cellSize, y * cellSize, cellSize, cellSize);
          }
        }
      }

      // Draw robot position
      ctx.fillStyle = "red";
      ctx.fillRect(mapData.x * cellSize, mapData.y * cellSize, cellSize, cellSize);
    }

    function updateLogs() {
      fetch('/logs')
        .then(r => r.text())
        .then(txt => {
          document.getElementById('logArea').textContent = txt;
        });
    }
    setInterval(updateLogs, 2000); // Actualiza cada 2 segundos
    updateLogs();
  </script>
</body>
</html>
)rawliteral";
