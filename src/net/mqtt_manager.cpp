#include <WiFi.h>
#include <PubSubClient.h>
#include "mqtt_manager.h"
#include "../globals.h"
#include "../config.h"
#include "../app/error_reporter.h"
#include "../logic/mqtt_topics.h"
#include "../logic/ha_state.h"

static WiFiClient wifiClient;
static PubSubClient client(wifiClient);
static unsigned long nextReconnectAttempt = 0;
static unsigned long nextPublish = 0;

static bool isActiveState() {
  return currentState == MOVING_FORWARD || currentState == MOVING_BACKWARD || currentState == TURNING;
}

static void publishDiscovery() {
  std::string prefix(mqttConfig.topicPrefix.c_str());
  std::string topic = mqttDiscoveryTopic(prefix);
  std::string payload = mqttDiscoveryPayload(prefix, MQTT_DEVICE_NAME);
  client.publish(topic.c_str(), payload.c_str(), true); // retained
}

static void publishState() {
  std::string prefix(mqttConfig.topicPrefix.c_str());
  HaState state = deriveHaState(isActiveState(), sessionCompletedByTimer);
  client.publish(mqttStateTopic(prefix).c_str(), haStateName(state), true);
}

static bool connectMqtt() {
  client.setServer(mqttConfig.host.c_str(), mqttConfig.port);

  std::string prefix(mqttConfig.topicPrefix.c_str());
  std::string availTopic = mqttAvailabilityTopic(prefix);

  bool ok;
  if (mqttConfig.user.length() > 0) {
    ok = client.connect(prefix.c_str(), mqttConfig.user.c_str(), mqttConfig.password.c_str(),
                        availTopic.c_str(), 0, true, "offline");
  } else {
    ok = client.connect(prefix.c_str(), availTopic.c_str(), 0, true, "offline");
  }

  if (ok) {
    client.publish(availTopic.c_str(), "online", true);
    publishDiscovery();
    publishState();
    clearErrorCode(ErrorCode::MqttConnectFailed);
    logBuffer.println("MQTT connected");
  } else {
    logError(ErrorCode::MqttConnectFailed);
  }
  return ok;
}

void setupMqtt() {
  nextReconnectAttempt = 0;
  nextPublish = 0;
}

void maintainMqtt() {
  if (!mqttConfig.enabled || mqttConfig.host.length() == 0) return;
  if (WiFi.status() != WL_CONNECTED) return;

  if (!client.connected()) {
    if (millis() < nextReconnectAttempt) return;
    nextReconnectAttempt = millis() + MQTT_RECONNECT_INTERVAL_MS;
    if (!connectMqtt()) return;
  }

  client.loop();

  if (millis() >= nextPublish) {
    nextPublish = millis() + MQTT_PUBLISH_INTERVAL_MS;
    publishState();
  }
}
