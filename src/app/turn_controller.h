#ifndef TURN_CONTROLLER_H
#define TURN_CONTROLLER_H

#include "../logic/turn_strategy.h"

// Pluggable turn strategy - same shape as ImuSensor (hal/imu.h). Each
// concrete strategy owns its own turning logic; robot_logic just asks for
// "the controller for the configured strategy" and calls turn() on it.
class TurnController {
public:
    virtual ~TurnController() {}
    virtual bool turn(int targetDegrees) = 0; // true = reached target, false = timed out
    virtual const char* name() const = 0;
};

TurnController& turnControllerFor(TurnStrategy strategy);

#endif // TURN_CONTROLLER_H
