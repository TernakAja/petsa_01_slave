#ifndef MAX30105_HEART_RATE_SENSOR_H
#define MAX30105_HEART_RATE_SENSOR_H

#include "../../include/IHeartRateSensor.h"
#include "../../include/ILogger.h"
#include "../../include/Config.h"
#include "MAX30105.h"
#include "heartRate.h"
#include "spo2_algorithm.h"

class MAX30105HeartRateSensor : public IHeartRateSensor {
private:
    MAX30105 particleSensor;
    ILogger* logger;
    const Config* config;
    
    long lastBeat;
    float beatsPerMinute;
    float spo2Value;
    unsigned long lastReadTime;
    bool isAwake;
    bool initialized;
    
    float bpmBuffer[4];
    uint8_t bpmIndex;
    
    static constexpr uint16_t BUFFER_SIZE = 100;
    uint32_t irBuffer[BUFFER_SIZE];
    uint32_t redBuffer[BUFFER_SIZE];
    
    static constexpr unsigned long READ_INTERVAL = 200;
    static constexpr uint32_t FINGER_THRESHOLD = 35000;
    static constexpr uint32_t SPO2_FINGER_THRESHOLD = 40000;
    static constexpr float MIN_BPM = 30.0f;
    static constexpr float MAX_BPM = 180.0f;
    static constexpr unsigned long MIN_BEAT_INTERVAL = 250;
    static constexpr unsigned long MAX_BEAT_INTERVAL = 2000;
    static constexpr unsigned long MAX_READ_TIME = 2500;
    
    bool setupSensor();
    bool detectFinger() const;
    float processHeartBeatData();
    bool collectSpO2Data();

public:
    MAX30105HeartRateSensor(ILogger* log, const Config* cfg);
    
    bool initialize() override;
    SensorReading read() override;
    bool isReady() const override { return initialized; }
    const char* getName() const override { return "MAX30105"; }
    void reset() override;
    
    HeartRateReading readHeartRate() override;
    SensorReading readSpO2() override;
    bool isFingerDetected() override;
    void enterSleepMode() override;
    void exitSleepMode() override;
};

#endif