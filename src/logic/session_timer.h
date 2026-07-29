#ifndef SESSION_TIMER_H
#define SESSION_TIMER_H

// durationMs == 0 means "unlimited" - no auto-stop.
inline bool isSessionTimeUp(unsigned long now, unsigned long startMillis, unsigned long durationMs) {
    if (durationMs == 0) return false;
    return (now - startMillis) >= durationMs;
}

#endif // SESSION_TIMER_H
