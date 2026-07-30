#include <LittleFS.h>
#include <ArduinoJson.h>
#include "mqtt_config_store.h"
#include "../globals.h"
#include "../config.h"

void saveMqttConfig() {
  File f = LittleFS.open(MQTT_CONFIG_PATH, "w");
  if (!f) return;

  JsonDocument doc;
  doc["enabled"] = mqttConfig.enabled;
  doc["host"] = mqttConfig.host;
  doc["port"] = mqttConfig.port;
  doc["user"] = mqttConfig.user;
  doc["password"] = mqttConfig.password;
  doc["topicPrefix"] = mqttConfig.topicPrefix;
  serializeJson(doc, f);
  f.close();
}

void mqttConfigInit() {
  LittleFS.begin(true); // no-op if already mounted elsewhere

  File f = LittleFS.open(MQTT_CONFIG_PATH, "r");
  if (!f) {
    mqttConfig = MqttConfig();
    return;
  }

  JsonDocument doc;
  deserializeJson(doc, f);
  f.close();

  mqttConfig.enabled = doc["enabled"] | false;
  mqttConfig.host = doc["host"] | "";
  mqttConfig.port = doc["port"] | 1883;
  mqttConfig.user = doc["user"] | "";
  mqttConfig.password = doc["password"] | "";
  mqttConfig.topicPrefix = doc["topicPrefix"] | String(MQTT_DEFAULT_TOPIC_PREFIX);
}

void resetMqttConfig() {
  mqttConfig = MqttConfig();
  saveMqttConfig();
  logBuffer.println("MQTT config reset to defaults");
}
