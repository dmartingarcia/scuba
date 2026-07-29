#include "robot_state.h"

String resolveState(RobotState state) {
  switch (state) {
    case MOVING_FORWARD: return "MOVING_FORWARD";
    case MOVING_BACKWARD: return "MOVING_BACKWARD";
    case TURNING: return "TURNING";
    case STOPPED: return "STOPPED";
    case STARTING: return "STARTING";
    default: return "UNKNOWN";
  }
}
