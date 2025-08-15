#ifndef ILOGGER_H
#define ILOGGER_H

#include <Arduino.h>

enum class LogLevel {
    DEBUG,
    INFO,
    WARNING,
    ERROR,
    CRITICAL
};

class ILogger {
public:
    virtual ~ILogger() = default;
    virtual void log(LogLevel level, const char* message) = 0;
    virtual void logf(LogLevel level, const char* format, ...) = 0;
    virtual void setLogLevel(LogLevel level) = 0;
    virtual LogLevel getLogLevel() const = 0;
    
    void debug(const char* message) { log(LogLevel::DEBUG, message); }
    void info(const char* message) { log(LogLevel::INFO, message); }
    void warning(const char* message) { log(LogLevel::WARNING, message); }
    void error(const char* message) { log(LogLevel::ERROR, message); }
    void critical(const char* message) { log(LogLevel::CRITICAL, message); }
    
    void debugf(const char* format, ...) {
        va_list args; va_start(args, format); 
        char buffer[256]; vsnprintf(buffer, sizeof(buffer), format, args); 
        va_end(args); log(LogLevel::DEBUG, buffer);
    }
    
    void infof(const char* format, ...) {
        va_list args; va_start(args, format); 
        char buffer[256]; vsnprintf(buffer, sizeof(buffer), format, args); 
        va_end(args); log(LogLevel::INFO, buffer);
    }
    
    void warningf(const char* format, ...) {
        va_list args; va_start(args, format); 
        char buffer[256]; vsnprintf(buffer, sizeof(buffer), format, args); 
        va_end(args); log(LogLevel::WARNING, buffer);
    }
    
    void errorf(const char* format, ...) {
        va_list args; va_start(args, format); 
        char buffer[256]; vsnprintf(buffer, sizeof(buffer), format, args); 
        va_end(args); log(LogLevel::ERROR, buffer);
    }
};

#endif