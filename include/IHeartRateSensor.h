#ifndef IHEART_RATE_SENSOR_H
#define IHEART_RATE_SENSOR_H

#include "ISensor.h"

struct HeartRateReading : public SensorReading {
    float spo2;
    uint32_t irSignal;
    uint32_t redSignal;
    
    HeartRateReading(float bpm = NAN, float oxygen = NAN, uint32_t ir = 0, uint32_t red = 0,
                    SensorStatus stat = SensorStatus::NO_DATA)
        : SensorReading(bpm, stat), spo2(oxygen), irSignal(ir), redSignal(red) {}
};

class IHeartRateSensor : public ISensor {
public:
    virtual HeartRateReading readHeartRate() = 0;
    virtual SensorReading readSpO2() = 0;
    virtual bool isFingerDetected() = 0;
    virtual void enterSleepMode() = 0;
    virtual void exitSleepMode() = 0;
};

#endif