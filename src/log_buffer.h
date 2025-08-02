#ifndef LOGBUFFER_H
#define LOGBUFFER_H

#include <Arduino.h>

class LogBuffer {
public:
    LogBuffer(size_t maxLen = 2000, size_t trimTo = 1500)
        : maxLength(maxLen), trimLength(trimTo) {}

    void print(const String& msg) {
        add(msg);
        Serial.print(msg);
    }

    void println(const String& msg) {
        add(msg + "\n");
        Serial.println(msg);
    }

    void add(const String& msg) {
        buffer += msg;
        if (buffer.length() > maxLength) {
            int idx = buffer.indexOf('\n', buffer.length() - trimLength);
            if (idx > 0) buffer = buffer.substring(idx + 1);
        }
    }

    String get() const {
        return buffer;
    }

    void clear() {
        buffer = "";
    }

private:
    String buffer;
    size_t maxLength;
    size_t trimLength;
};

#endif // LOGBUFFER_H