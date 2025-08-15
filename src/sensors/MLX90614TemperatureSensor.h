#ifndef MLX90614_TEMPERATURE_SENSOR_H
#define MLX90614_TEMPERATURE_SENSOR_H

#include "../../include/ITemperatureSensor.h"
#include "../../include/ILogger.h"
#include "../../include/Config.h"
#include <Adafruit_MLX90614.h>

template<int WindowSize = 15>
class RobustTempFilter {
private:
    float buffer[WindowSize];
    int count;
    int index;
    bool filled;

public:
    RobustTempFilter() : count(0), index(0), filled(false) {}
    
    void clear() { count = 0; index = 0; filled = false; }
    
    void push(float value) {
        buffer[index] = value;
        index = (index + 1) % WindowSize;
        if (count < WindowSize) count++;
        if (count == WindowSize) filled = true;
    }
    
    int size() const { return count; }
    
    float median() const;
    float mad(float median) const;
    float clippedMean(float k = 2.0f) const;
};

class MLX90614TemperatureSensor : public ITemperatureSensor {
private:
    Adafruit_MLX90614 mlx;
    ILogger* logger;
    const Config* config;
    
    float lastStableTemp;
    float lastSeenTemp;
    float lastObjectTemp;
    float lastAmbientTemp;
    unsigned long lastReadTime;
    
    RobustTempFilter<15> filter;
    bool settling;
    unsigned long settleStartTime;
    bool initialized;
    
    static constexpr float JUMP_THRESHOLD = 0.8f;
    static constexpr float MAD_THRESHOLD = 0.15f;
    static constexpr float CLIP_K = 2.0f;
    static constexpr float CONTACT_DELTA = 0.3f;
    static constexpr float CONTACT_BIAS = 3.0f;
    static constexpr unsigned long READ_INTERVAL = 200;
    static constexpr unsigned long SETTLE_MIN_MS = 700;
    static constexpr unsigned long SETTLE_MAX_MS = 4000;
    
    bool validateReading(float obj1, float obj2, float ambient) const;
    float calculateCoreTemperature(float objectTemp, float ambientTemp) const;
    bool processTemperatureReading(float coreTemp);

public:
    MLX90614TemperatureSensor(ILogger* log, const Config* cfg);
    
    bool initialize() override;
    SensorReading read() override;
    bool isReady() const override { return initialized; }
    const char* getName() const override { return "MLX90614"; }
    void reset() override;
    
    TemperatureReading readTemperature() override;
    float getAmbientTemperature() override { return lastAmbientTemp; }
    float getObjectTemperature() override { return lastObjectTemp; }
    bool isInContact() override;
};

#endif