#ifndef ROBOT_STATE_H
#define ROBOT_STATE_H

#include <Arduino.h>

enum RobotState {
  MOVING_FORWARD,
  MOVING_BACKWARD,
  TURNING,
  STOPPED,
  STARTING,
  // Detected upside down (see isUpsideDown() in sensors.cpp). Terminal like
  // STOPPED - only /control?action=start or a physical reboot leaves it.
  MAINTENANCE
};

String resolveState(RobotState state);

#endif // ROBOT_STATE_H
