#ifndef MQTT_TOPICS_H
#define MQTT_TOPICS_H

#include <string>

inline std::string mqttStateTopic(const std::string& prefix) {
    return prefix + "/state";
}

inline std::string mqttAvailabilityTopic(const std::string& prefix) {
    return prefix + "/availability";
}

// Home Assistant MQTT Discovery topic for a simple text sensor.
inline std::string mqttDiscoveryTopic(const std::string& prefix) {
    return "homeassistant/sensor/" + prefix + "/config";
}

// Minimal HA discovery payload: state_topic + availability_topic + a
// device block so it groups under one device in the HA UI instead of
// showing up as a bare orphaned entity.
inline std::string mqttDiscoveryPayload(const std::string& prefix, const std::string& deviceName) {
    return std::string("{")
        + "\"name\":\"" + deviceName + "\","
        + "\"unique_id\":\"" + prefix + "_state\","
        + "\"state_topic\":\"" + mqttStateTopic(prefix) + "\","
        + "\"availability_topic\":\"" + mqttAvailabilityTopic(prefix) + "\","
        + "\"payload_available\":\"online\","
        + "\"payload_not_available\":\"offline\","
        + "\"device\":{"
            + "\"identifiers\":[\"" + prefix + "\"],"
            + "\"name\":\"" + deviceName + "\","
            + "\"manufacturer\":\"scuba\""
        + "}"
    + "}";
}

#endif // MQTT_TOPICS_H
