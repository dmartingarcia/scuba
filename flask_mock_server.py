#!/usr/bin/env python3

from flask import Flask, send_from_directory, jsonify, request

app = Flask(__name__, static_folder="src")

# Mock data - shape must match the real firmware's /status and /config
# (src/net/web_server.cpp) so the UI behaves the same against both.
GRID_SIZE = 30
mock_map = [[(x + y) % 7 == 0 for x in range(GRID_SIZE)] for y in range(GRID_SIZE)]
mock_position = {"x": 15, "y": 15}
mock_session = {"durationMinutes": 0, "startedAtSeconds": 0}
mock_stats_save_interval_minutes = 10
mock_maintenance = {"bootCount": 3, "totalRuntimeHours": 12.5}
mock_logs = "Log de ejemplo\nRobot iniciado\nMovimiento adelante\n..."


@app.route("/")
def index():
    return send_from_directory("src", "index.html")


@app.route("/status")
def status():
    return jsonify(
        {
            "state": "MOVING_FORWARD",
            "angle": 45,
            "yaw": 123,
            "x": mock_position["x"],
            "y": mock_position["y"],
            "sessionDurationMinutes": mock_session["durationMinutes"],
            "sessionElapsedSeconds": 90,
            "imuName": "MPU9250",
            "imuHasMagnetometer": False,
            "maintenance": mock_maintenance,
            "map": mock_map,
        }
    )


@app.route("/control")
def control():
    action = request.args.get("action", "")
    print(f"Accion recibida: {action}")
    if action == "start" and "duration" in request.args:
        mock_session["durationMinutes"] = int(request.args.get("duration", 0))
    return "OK"


@app.route("/config")
def config():
    if "sessionDuration" in request.args:
        mock_session["durationMinutes"] = int(request.args["sessionDuration"])
    global mock_stats_save_interval_minutes
    if "statsSaveInterval" in request.args:
        mock_stats_save_interval_minutes = int(request.args["statsSaveInterval"])
    return jsonify(
        {
            "sessionDurationMinutes": mock_session["durationMinutes"],
            "statsSaveIntervalMinutes": mock_stats_save_interval_minutes,
            "turnStrategy": "legacy",
        }
    )


@app.route("/errors")
def errors():
    if request.args.get("action") == "clear":
        return "OK"
    return jsonify({"entries": []})


@app.route("/logs")
def logs():
    return mock_logs, 200, {"Content-Type": "text/plain"}


if __name__ == "__main__":
    app.run(debug=True)
