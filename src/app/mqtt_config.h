#ifndef MQTT_CONFIG_H
#define MQTT_CONFIG_H

#include <Arduino.h>
#include "../config.h"

// Broker connection settings + topic prefix, all UI-configurable and
// persisted to LittleFS (see mqtt_config_store.h). Password is never
// echoed back by the web API.
struct MqttConfig {
    bool enabled = false;
    String host = "";
    uint16_t port = 1883;
    String user = "";
    String password = "";
    String topicPrefix = MQTT_DEFAULT_TOPIC_PREFIX;
};

#endif // MQTT_CONFIG_H
