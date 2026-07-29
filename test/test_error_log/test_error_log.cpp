#include <unity.h>
#include "../../src/logic/error_log.h"

void setUp(void) {}
void tearDown(void) {}

static ErrorLog emptyLog() {
    ErrorLog log = {};
    log.count = 0;
    log.nextIndex = 0;
    log.activeMask = 0;
    return log;
}

void test_pushError_records_first_occurrence(void) {
    ErrorLog log = emptyLog();
    log = pushError(log, (uint8_t)ErrorCode::ImuInitFailed, 1000);
    TEST_ASSERT_EQUAL_UINT8(1, log.count);
    TEST_ASSERT_EQUAL_UINT8((uint8_t)ErrorCode::ImuInitFailed, log.entries[0].code);
    TEST_ASSERT_EQUAL_UINT32(1000, log.entries[0].timestamp);
    TEST_ASSERT_TRUE(isErrorActive(log, (uint8_t)ErrorCode::ImuInitFailed));
}

void test_pushError_dedupes_repeated_active_error(void) {
    ErrorLog log = emptyLog();
    log = pushError(log, (uint8_t)ErrorCode::ImuReadFailed, 1000);
    log = pushError(log, (uint8_t)ErrorCode::ImuReadFailed, 2000);
    log = pushError(log, (uint8_t)ErrorCode::ImuReadFailed, 3000);
    TEST_ASSERT_EQUAL_UINT8(1, log.count); // only the first got recorded
    TEST_ASSERT_EQUAL_UINT32(1000, log.entries[0].timestamp);
}

void test_clearError_allows_logging_again(void) {
    ErrorLog log = emptyLog();
    log = pushError(log, (uint8_t)ErrorCode::WifiConnectFailed, 1000);
    log = clearError(log, (uint8_t)ErrorCode::WifiConnectFailed);
    TEST_ASSERT_FALSE(isErrorActive(log, (uint8_t)ErrorCode::WifiConnectFailed));

    log = pushError(log, (uint8_t)ErrorCode::WifiConnectFailed, 5000);
    TEST_ASSERT_EQUAL_UINT8(2, log.count);
    TEST_ASSERT_EQUAL_UINT32(5000, log.entries[1].timestamp);
}

void test_different_error_codes_both_recorded(void) {
    ErrorLog log = emptyLog();
    log = pushError(log, (uint8_t)ErrorCode::ImuInitFailed, 1000);
    log = pushError(log, (uint8_t)ErrorCode::BmpInitFailed, 2000);
    TEST_ASSERT_EQUAL_UINT8(2, log.count);
    TEST_ASSERT_TRUE(isErrorActive(log, (uint8_t)ErrorCode::ImuInitFailed));
    TEST_ASSERT_TRUE(isErrorActive(log, (uint8_t)ErrorCode::BmpInitFailed));
}

void test_pushError_wraps_and_caps_count_at_capacity(void) {
    ErrorLog log = emptyLog();
    // Push more distinct codes than capacity by clearing+repushing a couple
    // of codes so each push is a fresh (non-deduped) occurrence.
    for (int i = 0; i < ERROR_LOG_CAPACITY + 3; i++) {
        uint8_t code = (uint8_t)(1 + (i % 5)); // cycle through the 5 known codes
        log = clearError(log, code);
        log = pushError(log, code, 1000 + i);
    }
    TEST_ASSERT_EQUAL_UINT8(ERROR_LOG_CAPACITY, log.count);
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_pushError_records_first_occurrence);
    RUN_TEST(test_pushError_dedupes_repeated_active_error);
    RUN_TEST(test_clearError_allows_logging_again);
    RUN_TEST(test_different_error_codes_both_recorded);
    RUN_TEST(test_pushError_wraps_and_caps_count_at_capacity);
    return UNITY_END();
}
