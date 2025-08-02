#!/usr/bin/env python3

from flask import Flask, send_from_directory, jsonify, request

app = Flask(__name__, static_folder="src")

# Mock data
mock_map = [[1 if (x + y) % 7 == 0 else 0 for x in range(20)] for y in range(20)]
mock_map_data = {"x": 10, "y": 10}
mock_logs = "Log de ejemplo\nRobot iniciado\nMovimiento adelante\n..."


@app.route("/")
def index():
    return send_from_directory("src", "index.html")


@app.route("/status")
def status():
    # Simula el estado del robot
    data = {
        "state": "MOVING_FORWARD",
        "yaw": 123,
        "angle": 45,  # Simulated angle
        "map": mock_map,
    }
    # Store robot position separately instead of trying to attach to the list
    robot_x = mock_map_data["x"]
    robot_y = mock_map_data["y"]
    return jsonify(
        {
            "state": data["state"],
            "yaw": data["yaw"],
            "angle": data["angle"],
            "map": data["map"],
            "robot_position": {"x": robot_x, "y": robot_y},
        }
    )


@app.route("/control")
def control():
    action = request.args.get("action", "")
    print(f"Acción recibida: {action}")
    return "OK"


@app.route("/logs")
def logs():
    return mock_logs, 200, {"Content-Type": "text/plain"}


if __name__ == "__main__":
    app.run(debug=True)
