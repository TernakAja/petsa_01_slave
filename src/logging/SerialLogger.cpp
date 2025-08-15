#include "SerialLogger.h"
#include <stdarg.h>

SerialLogger::SerialLogger(LogLevel level, bool timestamp)
    : currentLogLevel(level), useTimestamp(timestamp), startTime(millis()) {
}

const char* SerialLogger::getLevelString(LogLevel level) const {
    switch (level) {
        case LogLevel::DEBUG:    return "DEBUG";
        case LogLevel::INFO:     return "INFO ";
        case LogLevel::WARNING:  return "WARN ";
        case LogLevel::ERROR:    return "ERROR";
        case LogLevel::CRITICAL: return "CRIT ";
        default: return "UNKN ";
    }
}

void SerialLogger::printTimestamp() const {
    if (!useTimestamp) return;
    
    unsigned long elapsed = millis() - startTime;
    unsigned long seconds = elapsed / 1000;
    unsigned long milliseconds = elapsed % 1000;
    unsigned long minutes = seconds / 60;
    seconds = seconds % 60;
    unsigned long hours = minutes / 60;
    minutes = minutes % 60;
    
    Serial.printf("[%02lu:%02lu:%02lu.%03lu] ", hours, minutes, seconds, milliseconds);
}

void SerialLogger::printLevel(LogLevel level) const {
    Serial.print("[");
    Serial.print(getLevelString(level));
    Serial.print("] ");
}

void SerialLogger::log(LogLevel level, const char* message) {
    if (level < currentLogLevel) return;
    
    printTimestamp();
    printLevel(level);
    Serial.println(message);
}

void SerialLogger::logf(LogLevel level, const char* format, ...) {
    if (level < currentLogLevel) return;
    
    char buffer[256];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    log(level, buffer);
}