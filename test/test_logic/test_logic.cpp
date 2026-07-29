#include <unity.h>
#include "../../src/logic/speed_utils.h"
#include "../../src/logic/turn_math.h"
#include "../../src/logic/position_math.h"

void setUp(void) {}
void tearDown(void) {}

// --- speed_utils ---

void test_clampSpeed_within_range_unchanged(void) {
    TEST_ASSERT_EQUAL_INT(0, clampSpeed(0));
    TEST_ASSERT_EQUAL_INT(100, clampSpeed(100));
    TEST_ASSERT_EQUAL_INT(-100, clampSpeed(-100));
}

void test_clampSpeed_clamps_upper_bound(void) {
    TEST_ASSERT_EQUAL_INT(255, clampSpeed(300));
    TEST_ASSERT_EQUAL_INT(255, clampSpeed(255));
}

void test_clampSpeed_clamps_lower_bound(void) {
    TEST_ASSERT_EQUAL_INT(-255, clampSpeed(-300));
    TEST_ASSERT_EQUAL_INT(-255, clampSpeed(-255));
}

// --- turn_math ---

void test_normalizeAngle360_wraps_negative(void) {
    TEST_ASSERT_EQUAL_FLOAT(350.0f, normalizeAngle360(-10.0f));
}

void test_normalizeAngle360_wraps_over(void) {
    TEST_ASSERT_EQUAL_FLOAT(10.0f, normalizeAngle360(370.0f));
}

void test_normalizeAngle360_leaves_in_range_unchanged(void) {
    TEST_ASSERT_EQUAL_FLOAT(180.0f, normalizeAngle360(180.0f));
}

void test_computeTurnRange_wraps_below_zero(void) {
    YawRange range = computeTurnRange(5.0f, 15.0f);
    TEST_ASSERT_EQUAL_FLOAT(350.0f, range.min); // 5 - 15 = -10 -> 350
    TEST_ASSERT_EQUAL_FLOAT(20.0f, range.max);
}

void test_computeTurnRange_wraps_above_360(void) {
    YawRange range = computeTurnRange(355.0f, 15.0f);
    TEST_ASSERT_EQUAL_FLOAT(340.0f, range.min);
    TEST_ASSERT_EQUAL_FLOAT(10.0f, range.max); // 355 + 15 = 370 -> 10
}

void test_isYawOutsideRange_inside(void) {
    YawRange range = {10.0f, 40.0f};
    TEST_ASSERT_FALSE(isYawOutsideRange(25.0f, range));
}

void test_isYawOutsideRange_below(void) {
    YawRange range = {10.0f, 40.0f};
    TEST_ASSERT_TRUE(isYawOutsideRange(5.0f, range));
}

void test_isYawOutsideRange_above(void) {
    YawRange range = {10.0f, 40.0f};
    TEST_ASSERT_TRUE(isYawOutsideRange(45.0f, range));
}

void test_isTimeElapsed_false_before_duration(void) {
    TEST_ASSERT_FALSE(isTimeElapsed(2000, 1000, 5000));
}

void test_isTimeElapsed_true_at_duration(void) {
    TEST_ASSERT_TRUE(isTimeElapsed(6000, 1000, 5000));
}

// --- position_math ---

void test_updateGridPosition_none_direction_unchanged(void) {
    GridPos pos = {5, 5};
    GridPos result = updateGridPosition(pos, 0.0f, MoveDirection::None, 30);
    TEST_ASSERT_EQUAL_INT(5, result.x);
    TEST_ASSERT_EQUAL_INT(5, result.y);
}

void test_updateGridPosition_forward_north(void) {
    GridPos pos = {5, 5};
    GridPos result = updateGridPosition(pos, 0.0f, MoveDirection::Forward, 30);
    TEST_ASSERT_EQUAL_INT(5, result.x);
    TEST_ASSERT_EQUAL_INT(4, result.y);
}

void test_updateGridPosition_forward_east(void) {
    GridPos pos = {5, 5};
    GridPos result = updateGridPosition(pos, 90.0f, MoveDirection::Forward, 30);
    TEST_ASSERT_EQUAL_INT(6, result.x);
    TEST_ASSERT_EQUAL_INT(5, result.y);
}

void test_updateGridPosition_backward_east_moves_west(void) {
    GridPos pos = {5, 5};
    GridPos result = updateGridPosition(pos, 90.0f, MoveDirection::Backward, 30);
    TEST_ASSERT_EQUAL_INT(4, result.x);
    TEST_ASSERT_EQUAL_INT(5, result.y);
}

// Regression test for the out-of-bounds bug: backward movement at the
// grid's upper edge must clamp to gridSize - 1, never reach gridSize.
void test_updateGridPosition_clamps_upper_edge_going_backward(void) {
    GridPos pos = {29, 29}; // last valid index for gridSize = 30
    GridPos east = updateGridPosition(pos, 90.0f, MoveDirection::Forward, 30);
    TEST_ASSERT_EQUAL_INT(29, east.x); // would be 30 (OOB) without clamping

    GridPos south = updateGridPosition(pos, 180.0f, MoveDirection::Forward, 30);
    TEST_ASSERT_EQUAL_INT(29, south.y); // would be 30 (OOB) without clamping
}

void test_updateGridPosition_clamps_lower_edge(void) {
    GridPos pos = {0, 0};
    // Facing North (yaw 0) and moving Forward decreases y towards the edge.
    GridPos result = updateGridPosition(pos, 0.0f, MoveDirection::Forward, 30);
    TEST_ASSERT_EQUAL_INT(0, result.y); // would be -1 without clamping
}

int main(int argc, char **argv) {
    UNITY_BEGIN();

    RUN_TEST(test_clampSpeed_within_range_unchanged);
    RUN_TEST(test_clampSpeed_clamps_upper_bound);
    RUN_TEST(test_clampSpeed_clamps_lower_bound);

    RUN_TEST(test_normalizeAngle360_wraps_negative);
    RUN_TEST(test_normalizeAngle360_wraps_over);
    RUN_TEST(test_normalizeAngle360_leaves_in_range_unchanged);
    RUN_TEST(test_computeTurnRange_wraps_below_zero);
    RUN_TEST(test_computeTurnRange_wraps_above_360);
    RUN_TEST(test_isYawOutsideRange_inside);
    RUN_TEST(test_isYawOutsideRange_below);
    RUN_TEST(test_isYawOutsideRange_above);
    RUN_TEST(test_isTimeElapsed_false_before_duration);
    RUN_TEST(test_isTimeElapsed_true_at_duration);

    RUN_TEST(test_updateGridPosition_none_direction_unchanged);
    RUN_TEST(test_updateGridPosition_forward_north);
    RUN_TEST(test_updateGridPosition_forward_east);
    RUN_TEST(test_updateGridPosition_backward_east_moves_west);
    RUN_TEST(test_updateGridPosition_clamps_upper_edge_going_backward);
    RUN_TEST(test_updateGridPosition_clamps_lower_edge);

    return UNITY_END();
}
