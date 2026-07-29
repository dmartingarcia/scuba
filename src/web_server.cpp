#include "web_server.h"
#include "globals.h"
#include "sensors.h"
#include "index.h" // HTML content for the web interface

void setupWebServer() {
  // Serve main page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/html", INDEX_HTML);
  });

  // Serve logs
  server.on("/logs", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", logBuffer.get());
  });

  // Control endpoint
  server.on("/control", HTTP_GET, [](AsyncWebServerRequest *request) {
    String action;
    if (request->hasParam("action")) {
      action = request->getParam("action")->value();

      if (action == "start") {
        currentState = MOVING_FORWARD;
      } else if (action == "stop") {
        currentState = STOPPED;
      } else if (action == "turn") {
        currentState = TURNING;
      }
    }
    request->send(200, "text/plain", "OK");
  });

  // Status endpoint
  server.on("/status", HTTP_GET, [](AsyncWebServerRequest *request) {
    String response = "{";
    response += "\"state\":\"" + resolveState(currentState) + "\"";
    response += ",\"angle\":" + String(angle());
    response += ",\"yaw\": " + String(yaw);
    response += ",\"map\":[";

    for (int y = 0; y < GRID_SIZE; y++) {
      response += "[";
      for (int x = 0; x < GRID_SIZE; x++) {
        response += cleanedArea[y][x] ? "true" : "false";
        if (x < GRID_SIZE - 1) response += ",";
      }
      response += "]";
      if (y < GRID_SIZE - 1) response += ",";
    }

    response += "],\"x\":" + String(currentX) + ",\"y\":" + String(currentY);
    response += "}";

    request->send(200, "application/json", response);
  });

  server.begin();
}
