#ifndef POSITION_MATH_H
#define POSITION_MATH_H

enum class MoveDirection { None, Forward, Backward };

struct GridPos {
    int x;
    int y;
};

inline int clampIndex(int value, int gridSize) {
    if (value < 0) return 0;
    if (value > gridSize - 1) return gridSize - 1;
    return value;
}

// Moves one grid cell in the compass direction implied by yaw, clamped to the grid bounds.
inline GridPos updateGridPosition(GridPos pos, float yaw, MoveDirection dir, int gridSize) {
    if (dir == MoveDirection::None) return pos;

    int delta = (dir == MoveDirection::Forward) ? 1 : -1;

    if (yaw >= 315 || yaw < 45) { // North
        pos.y = clampIndex(pos.y - delta, gridSize);
    } else if (yaw >= 45 && yaw < 135) { // East
        pos.x = clampIndex(pos.x + delta, gridSize);
    } else if (yaw >= 135 && yaw < 225) { // South
        pos.y = clampIndex(pos.y + delta, gridSize);
    } else { // West
        pos.x = clampIndex(pos.x - delta, gridSize);
    }

    return pos;
}

#endif // POSITION_MATH_H
