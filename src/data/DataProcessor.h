#ifndef DATA_PROCESSOR_H
#define DATA_PROCESSOR_H

#include "../../include/IDataProcessor.h"
#include "../../include/ILogger.h"
#include "../../include/Config.h"

class DataValidator {
public:
    static bool isValidTemperature(float temp) {
        return !isnan(temp) && temp >= 10.0f && temp <= 60.0f;
    }
    
    static bool isValidHeartRate(float hr) {
        return !isnan(hr) && hr >= 30.0f && hr <= 200.0f;
    }
    
    static bool isValidSpO2(float spo2) {
        return !isnan(spo2) && spo2 >= 70.0f && spo2 <= 100.0f;
    }
    
    static VitalSigns validateVitalSigns(const VitalSigns& data) {
        VitalSigns validated = data;
        
        if (!isValidTemperature(data.temperature)) {
            validated.temperature = NAN;
        }
        
        if (!isValidHeartRate(data.heartRate)) {
            validated.heartRate = NAN;
        }
        
        if (!isValidSpO2(data.spo2)) {
            validated.spo2 = NAN;
        }
        
        return validated;
    }
};

class DataProcessor : public IDataProcessor {
private:
    ILogger* logger;
    const Config* config;
    
    float* temperatureData;
    float* heartRateData;
    float* spo2Data;
    
    int currentDataPoints;
    int maxDataPoints;
    unsigned long lastCollectionTime;
    
    void allocateBuffers();
    void deallocateBuffers();
    float calculateAverage(const float* data, int count, bool (*validator)(float)) const;
    int countValidData(const float* data, int count, bool (*validator)(float)) const;

public:
    DataProcessor(ILogger* log, const Config* cfg);
    ~DataProcessor();
    
    void addDataPoint(float temperature, float heartRate, float spo2) override;
    VitalSigns getAveragedData() override;
    bool isReadyToSend() const override;
    void reset() override;
    int getCurrentDataPoints() const override { return currentDataPoints; }
    int getMaxDataPoints() const override { return maxDataPoints; }
    
    void setMaxDataPoints(int points);
    unsigned long getLastCollectionTime() const { return lastCollectionTime; }
    float getDataPoint(int index, int type) const; // 0=temp, 1=hr, 2=spo2
};

#endif