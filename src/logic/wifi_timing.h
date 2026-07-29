#ifndef WIFI_TIMING_H
#define WIFI_TIMING_H

// Pure timing decisions for non-blocking WiFi reconnect, kept separate from
// the actual WiFi.begin()/status() calls so they're testable without hardware.

inline bool isReconnectDue(unsigned long now, unsigned long nextAttemptAt) {
    return now >= nextAttemptAt;
}

inline unsigned long nextReconnectAttempt(unsigned long now, unsigned long intervalMs) {
    return now + intervalMs;
}

#endif // WIFI_TIMING_H
