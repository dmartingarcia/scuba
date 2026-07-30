#ifndef HA_STATE_H
#define HA_STATE_H

// What we tell Home Assistant, mapped from the robot's own state machine:
// "paused" (STOPPED, no other reason), "finished" (STOPPED because the
// configured session duration ran out), "cleaning" (anything else active).
enum class HaState { Cleaning, Paused, Finished };

inline HaState deriveHaState(bool isActive, bool sessionCompletedByTimer) {
    if (isActive) return HaState::Cleaning;
    return sessionCompletedByTimer ? HaState::Finished : HaState::Paused;
}

inline const char* haStateName(HaState s) {
    switch (s) {
        case HaState::Cleaning: return "cleaning";
        case HaState::Finished: return "finished";
        case HaState::Paused:
        default: return "paused";
    }
}

#endif // HA_STATE_H
