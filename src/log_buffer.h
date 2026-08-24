#ifndef LOGBUFFER_H
#define LOGBUFFER_H

#include <Arduino.h>
#include <esp_heap_caps.h>
#include "config.h"

// Ring-ish append-only log. Backed by a fixed buffer allocated once in
// PSRAM (see LOG_BUFFER_SIZE in config.h) rather than a growing String in
// internal SRAM, so it can be large without competing with WiFi/
// AsyncWebServer for the tight internal heap. Falls back to a much smaller
// internal-heap buffer if PSRAM allocation fails (e.g. no PSRAM on this
// board).
class LogBuffer {
public:
    LogBuffer(size_t maxLen = LOG_BUFFER_SIZE, size_t trimTo = LOG_BUFFER_TRIM_SIZE)
        : maxLength(maxLen), trimLength(trimTo), length(0) {
        buffer = (char*)heap_caps_malloc(maxLength + 1, MALLOC_CAP_SPIRAM);
        if (!buffer) {
            maxLength = LOG_BUFFER_FALLBACK_SIZE;
            trimLength = LOG_BUFFER_FALLBACK_TRIM_SIZE;
            buffer = (char*)malloc(maxLength + 1);
        }
        if (buffer) buffer[0] = '\0';
    }

    void print(const String& msg) {
        add(msg);
        Serial.print(msg);
    }

    void println(const String& msg) {
        add(msg + "\n");
        Serial.println(msg);
    }

    void add(const String& msg) {
        if (!buffer || maxLength == 0) return;

        size_t msgLen = msg.length();
        if (msgLen > maxLength) {
            // Pathological: a single message bigger than the whole buffer - keep only its tail.
            memcpy(buffer, msg.c_str() + (msgLen - maxLength), maxLength);
            length = maxLength;
            buffer[length] = '\0';
            return;
        }

        while (length + msgLen > maxLength) {
            trim();
        }
        memcpy(buffer + length, msg.c_str(), msgLen);
        length += msgLen;
        buffer[length] = '\0';
    }

    String get() const {
        return buffer ? String(buffer) : String();
    }

    void clear() {
        length = 0;
        if (buffer) buffer[0] = '\0';
    }

private:
    // Drops the oldest bytes down to ~trimLength, cutting at the next
    // newline so a partial line isn't left dangling at the start.
    void trim() {
        if (length <= trimLength) return;
        size_t dropFrom = length - trimLength;
        char* newlineAt = (char*)memchr(buffer + dropFrom, '\n', length - dropFrom);
        size_t keepFrom = newlineAt ? (size_t)(newlineAt - buffer) + 1 : dropFrom;
        size_t keepLen = length - keepFrom;
        memmove(buffer, buffer + keepFrom, keepLen);
        length = keepLen;
        buffer[length] = '\0';
    }

    char* buffer;
    size_t maxLength;
    size_t trimLength;
    size_t length;
};

#endif // LOGBUFFER_H
