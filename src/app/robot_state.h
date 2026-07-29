#ifndef ROBOT_STATE_H
#define ROBOT_STATE_H

#include <Arduino.h>

enum RobotState {
  MOVING_FORWARD,
  MOVING_BACKWARD,
  TURNING,
  STOPPED,
  STARTING
};

String resolveState(RobotState state);

#endif // ROBOT_STATE_H
