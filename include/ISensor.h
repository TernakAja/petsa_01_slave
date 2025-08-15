#ifndef ISENSOR_H
#define ISENSOR_H

#include <Arduino.h>

enum class SensorStatus {
    OK,
    ERROR,
    NOT_INITIALIZED,
    NO_DATA,
    INVALID_READING
};

struct SensorReading {
    float value;
    SensorStatus status;
    unsigned long timestamp;
    
    SensorReading(float val = NAN, SensorStatus stat = SensorStatus::NO_DATA) 
        : value(val), status(stat), timestamp(millis()) {}
    
    bool isValid() const {
        return status == SensorStatus::OK && !isnan(value);
    }
};

class ISensor {
public:
    virtual ~ISensor() = default;
    virtual bool initialize() = 0;
    virtual SensorReading read() = 0;
    virtual bool isReady() const = 0;
    virtual const char* getName() const = 0;
    virtual void reset() = 0;
};

#endif