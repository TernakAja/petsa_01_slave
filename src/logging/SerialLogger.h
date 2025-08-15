#ifndef SERIAL_LOGGER_H
#define SERIAL_LOGGER_H

#include "../../include/ILogger.h"

class SerialLogger : public ILogger {
private:
    LogLevel currentLogLevel;
    bool useTimestamp;
    unsigned long startTime;
    
    const char* getLevelString(LogLevel level) const;
    void printTimestamp() const;
    void printLevel(LogLevel level) const;

public:
    SerialLogger(LogLevel level = LogLevel::INFO, bool timestamp = true);
    
    void log(LogLevel level, const char* message) override;
    void logf(LogLevel level, const char* format, ...) override;
    void setLogLevel(LogLevel level) override { currentLogLevel = level; }
    LogLevel getLogLevel() const override { return currentLogLevel; }
    
    void setUseTimestamp(bool use) { useTimestamp = use; }
    bool getUseTimestamp() const { return useTimestamp; }
};

#endif