#ifndef MQTT_MANAGER_H
#define MQTT_MANAGER_H

void setupMqtt();    // Call once from setup(), after mqttConfigInit()
void maintainMqtt(); // Call every loop(): reconnects + publishes, all non-blocking

#endif // MQTT_MANAGER_H
