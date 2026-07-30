#include <unity.h>
#include "../../src/logic/mqtt_topics.h"

void setUp(void) {}
void tearDown(void) {}

void test_state_topic(void) {
    TEST_ASSERT_EQUAL_STRING("scuba/state", mqttStateTopic("scuba").c_str());
}

void test_availability_topic(void) {
    TEST_ASSERT_EQUAL_STRING("scuba/availability", mqttAvailabilityTopic("scuba").c_str());
}

void test_discovery_topic(void) {
    TEST_ASSERT_EQUAL_STRING("homeassistant/sensor/scuba/config", mqttDiscoveryTopic("scuba").c_str());
}

void test_discovery_payload_contains_required_fields(void) {
    std::string payload = mqttDiscoveryPayload("scuba", "Pool Robot");
    TEST_ASSERT_TRUE(payload.find("\"state_topic\":\"scuba/state\"") != std::string::npos);
    TEST_ASSERT_TRUE(payload.find("\"availability_topic\":\"scuba/availability\"") != std::string::npos);
    TEST_ASSERT_TRUE(payload.find("\"unique_id\":\"scuba_state\"") != std::string::npos);
    TEST_ASSERT_TRUE(payload.find("\"name\":\"Pool Robot\"") != std::string::npos);
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_state_topic);
    RUN_TEST(test_availability_topic);
    RUN_TEST(test_discovery_topic);
    RUN_TEST(test_discovery_payload_contains_required_fields);
    return UNITY_END();
}
