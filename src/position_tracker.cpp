#include "position_tracker.h"
#include "globals.h"
#include "logic/position_math.h"

void updatePosition() {
  if (nextPositionUpdate > (long)millis()) return;
  nextPositionUpdate += DELAY_UPDATING_POSITION;

  MoveDirection direction;
  if (currentState == MOVING_FORWARD) {
    direction = MoveDirection::Forward;
  } else if (currentState == MOVING_BACKWARD) {
    direction = MoveDirection::Backward;
  } else {
    return; // Not moving (turning/stopped/starting): position doesn't change
  }

  // Mark current position as cleaned
  cleanedArea[currentY][currentX] = true;

  GridPos updated = updateGridPosition({currentX, currentY}, yaw, direction, GRID_SIZE);
  currentX = updated.x;
  currentY = updated.y;
}
