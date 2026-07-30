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
mock_accel_calibration = {"calibrated": False, "zOffset": 0.0}
mock_errors = []
mock_mqtt = {"enabled": False, "host": "", "port": 1883, "user": "", "password": "", "topicPrefix": "scuba"}
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
            "accelCalibrated": mock_accel_calibration["calibrated"],
            "accelZeroOffset": mock_accel_calibration["zOffset"],
            # Simulates a slightly misaligned IMU mount until calibrated,
            # so the 3D model visibly flattens after hitting Calibrate.
            "pitchDeg": 0.0 if mock_accel_calibration["calibrated"] else 8.0,
            "rollDeg": 0.0 if mock_accel_calibration["calibrated"] else -5.0,
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
        mock_errors.clear()
        return "OK"
    return jsonify({"entries": mock_errors})


@app.route("/calibrate")
def calibrate():
    if request.args.get("action") == "clear":
        mock_accel_calibration["calibrated"] = False
        mock_accel_calibration["zOffset"] = 0.0
    else:
        mock_accel_calibration["calibrated"] = True
        mock_accel_calibration["zOffset"] = -0.02
    return jsonify(mock_accel_calibration)


@app.route("/mqtt")
def mqtt():
    if request.args.get("action") == "reset":
        mock_mqtt.update({"enabled": False, "host": "", "port": 1883, "user": "", "password": "", "topicPrefix": "scuba"})
    else:
        if "enabled" in request.args:
            mock_mqtt["enabled"] = request.args["enabled"] == "1"
        if "host" in request.args:
            mock_mqtt["host"] = request.args["host"]
        if "port" in request.args:
            mock_mqtt["port"] = int(request.args["port"])
        if "user" in request.args:
            mock_mqtt["user"] = request.args["user"]
        if "password" in request.args:
            mock_mqtt["password"] = request.args["password"]
        if "topicPrefix" in request.args:
            mock_mqtt["topicPrefix"] = request.args["topicPrefix"]

    return jsonify(
        {
            "enabled": mock_mqtt["enabled"],
            "host": mock_mqtt["host"],
            "port": mock_mqtt["port"],
            "user": mock_mqtt["user"],
            "topicPrefix": mock_mqtt["topicPrefix"],
            "hasPassword": len(mock_mqtt["password"]) > 0,
        }
    )


@app.route("/logs")
def logs():
    return mock_logs, 200, {"Content-Type": "text/plain"}


if __name__ == "__main__":
    app.run(debug=True)
