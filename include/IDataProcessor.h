#ifndef IDATA_PROCESSOR_H
#define IDATA_PROCESSOR_H

#include <Arduino.h>

struct VitalSigns {
    float temperature;
    float heartRate;
    float spo2;
    unsigned long timestamp;
    int dataPoints;
    
    VitalSigns(float temp = NAN, float hr = NAN, float oxygen = NAN, 
              unsigned long ts = 0, int points = 0)
        : temperature(temp), heartRate(hr), spo2(oxygen), 
          timestamp(ts), dataPoints(points) {}
          
    bool isValid() const {
        return !isnan(temperature) && !isnan(heartRate) && !isnan(spo2);
    }
};

class IDataProcessor {
public:
    virtual ~IDataProcessor() = default;
    virtual void addDataPoint(float temperature, float heartRate, float spo2) = 0;
    virtual VitalSigns getAveragedData() = 0;
    virtual bool isReadyToSend() const = 0;
    virtual void reset() = 0;
    virtual int getCurrentDataPoints() const = 0;
    virtual int getMaxDataPoints() const = 0;
};

#endif