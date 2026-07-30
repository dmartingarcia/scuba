#ifndef MQTT_CONFIG_STORE_H
#define MQTT_CONFIG_STORE_H

void mqttConfigInit();   // Load persisted MQTT config from LittleFS
void saveMqttConfig();   // Persist current globals.mqttConfig
void resetMqttConfig();  // Back to defaults (disabled, empty), persist

#endif // MQTT_CONFIG_STORE_H
