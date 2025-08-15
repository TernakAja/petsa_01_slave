#include "MLX90614TemperatureSensor.h"
#include <Wire.h>

template<int WindowSize>
float RobustTempFilter<WindowSize>::median() const {
    if (count == 0) return NAN;
    
    float temp[WindowSize];
    for (int i = 0; i < count; i++) {
        temp[i] = buffer[i];
    }
    
    for (int i = 1; i < count; i++) {
        float key = temp[i];
        int j = i - 1;
        while (j >= 0 && temp[j] > key) {
            temp[j + 1] = temp[j];
            j--;
        }
        temp[j + 1] = key;
    }
    
    return (count & 1) ? temp[count / 2] : 0.5f * (temp[count / 2 - 1] + temp[count / 2]);
}

template<int WindowSize>
float RobustTempFilter<WindowSize>::mad(float med) const {
    if (count == 0) return NAN;
    
    float temp[WindowSize];
    for (int i = 0; i < count; i++) {
        temp[i] = fabsf(buffer[i] - med);
    }
    
    for (int i = 1; i < count; i++) {
        float key = temp[i];
        int j = i - 1;
        while (j >= 0 && temp[j] > key) {
            temp[j + 1] = temp[j];
            j--;
        }
        temp[j + 1] = key;
    }
    
    return (count & 1) ? temp[count / 2] : 0.5f * (temp[count / 2 - 1] + temp[count / 2]);
}

template<int WindowSize>
float RobustTempFilter<WindowSize>::clippedMean(float k) const {
    if (count == 0) return NAN;
    
    float med = median();
    float MAD = mad(med);
    float sigma = 1.4826f * MAD;
    
    if (!(sigma > 0)) return med;
    
    float lo = med - k * sigma;
    float hi = med + k * sigma;
    float sum = 0;
    int cnt = 0;
    
    for (int i = 0; i < count; i++) {
        float v = buffer[i];
        if (v >= lo && v <= hi) {
            sum += v;
            cnt++;
        }
    }
    
    return (cnt > 0) ? (sum / cnt) : med;
}

MLX90614TemperatureSensor::MLX90614TemperatureSensor(ILogger* log, const Config* cfg)
    : logger(log), config(cfg), lastStableTemp(NAN), lastSeenTemp(NAN),
      lastObjectTemp(NAN), lastAmbientTemp(NAN), lastReadTime(0),
      settling(false), settleStartTime(0), initialized(false) {
}

bool MLX90614TemperatureSensor::initialize() {
    if (logger) logger->info("Initializing MLX90614 temperature sensor");
    
    Wire.setClock(config->i2c.clockSpeed);
    Wire.setClockStretchLimit(config->i2c.stretchLimit);
    
    for (int retry = 0; retry < 10 && !initialized; retry++) {
        if (logger) logger->debugf("MLX90614 init attempt %d", retry + 1);
        
        if (mlx.begin()) {
            float emissivity = mlx.readEmissivity();
            if (!isnan(emissivity)) {
                if (logger) logger->infof("MLX90614 emissivity: %.2f", emissivity);
            }
            initialized = true;
            break;
        }
        delay(250);
        yield();
    }
    
    if (initialized) {
        if (logger) logger->info("MLX90614 initialized successfully");
    } else {
        if (logger) logger->error("MLX90614 initialization failed");
    }
    
    return initialized;
}

bool MLX90614TemperatureSensor::validateReading(float obj1, float obj2, float ambient) const {
    return !(isnan(obj1) || isnan(obj2) || isnan(ambient)) &&
           (ambient > -40.0f && ambient < 125.0f) &&
           (obj1 > -70.0f && obj1 < 380.0f) &&
           (obj2 > -70.0f && obj2 < 380.0f) &&
           (fabsf(obj1 - obj2) <= 3.0f);
}

float MLX90614TemperatureSensor::calculateCoreTemperature(float objectTemp, float ambientTemp) const {
    bool inContact = (objectTemp - ambientTemp) >= CONTACT_DELTA;
    return inContact ? (0.8f * objectTemp + 0.1f * ambientTemp + CONTACT_BIAS) : objectTemp;
}

bool MLX90614TemperatureSensor::processTemperatureReading(float coreTemp) {
    if (coreTemp < 10.0f || coreTemp > 60.0f) {
        return false;
    }
    
    if (isnan(lastStableTemp)) {
        lastSeenTemp = coreTemp;
        filter.clear();
        filter.push(coreTemp);
        lastStableTemp = coreTemp;
        settling = false;
        settleStartTime = millis();
        return true;
    }
    
    filter.push(coreTemp);
    
    bool inContact = isInContact();
    if (!inContact) {
        lastStableTemp = filter.clippedMean(CLIP_K);
        settling = false;
        lastSeenTemp = coreTemp;
        return true;
    }
    
    if (isnan(lastSeenTemp) || fabsf(coreTemp - lastSeenTemp) > JUMP_THRESHOLD) {
        settling = true;
        settleStartTime = millis();
        filter.clear();
        filter.push(coreTemp);
    }
    lastSeenTemp = coreTemp;
    
    bool ready = false;
    if (filter.size() >= 5) {
        float med = filter.median();
        float MAD = filter.mad(med);
        if ((MAD < MAD_THRESHOLD && (millis() - settleStartTime) >= SETTLE_MIN_MS) ||
            (settling && (millis() - settleStartTime) >= SETTLE_MAX_MS)) {
            ready = true;
        }
    }
    
    if (settling) {
        if (ready) {
            lastStableTemp = filter.clippedMean(CLIP_K);
            settling = false;
        }
    } else {
        lastStableTemp = filter.clippedMean(CLIP_K);
    }
    
    return true;
}

SensorReading MLX90614TemperatureSensor::read() {
    TemperatureReading tempReading = readTemperature();
    return SensorReading(tempReading.value, tempReading.status);
}

TemperatureReading MLX90614TemperatureSensor::readTemperature() {
    if (!initialized) {
        return TemperatureReading(NAN, NAN, NAN, false, SensorStatus::NOT_INITIALIZED);
    }
    
    if (millis() - lastReadTime < READ_INTERVAL) {
        return TemperatureReading(lastStableTemp, lastAmbientTemp, lastObjectTemp, 
                                 isInContact(), SensorStatus::OK);
    }
    
    lastReadTime = millis();
    
    const int MAX_ATTEMPTS = 3;
    float objectTemp = NAN, ambientTemp = NAN;
    bool readOk = false;
    
    for (int attempt = 0; attempt < MAX_ATTEMPTS && !readOk; ++attempt) {
        float obj1 = mlx.readObjectTempC();
        delayMicroseconds(500);
        float obj2 = mlx.readObjectTempC();
        float ambient = mlx.readAmbientTempC();
        
        if (validateReading(obj1, obj2, ambient)) {
            objectTemp = obj1;
            ambientTemp = ambient;
            readOk = true;
        } else {
            delay(2);
            yield();
        }
    }
    
    if (!readOk) {
        if (logger) logger->warning("MLX90614 read validation failed");
        return TemperatureReading(lastStableTemp, lastAmbientTemp, lastObjectTemp,
                                 isInContact(), SensorStatus::ERROR);
    }
    
    lastObjectTemp = objectTemp;
    lastAmbientTemp = ambientTemp;
    
    if (ambientTemp < -20.0f || ambientTemp > 60.0f) {
        if (logger) logger->warningf("Ambient temperature out of range: %.2f", ambientTemp);
        return TemperatureReading(lastStableTemp, ambientTemp, objectTemp,
                                 isInContact(), SensorStatus::INVALID_READING);
    }
    
    float coreTemp = calculateCoreTemperature(objectTemp, ambientTemp);
    if (!processTemperatureReading(coreTemp)) {
        return TemperatureReading(lastStableTemp, ambientTemp, objectTemp,
                                 isInContact(), SensorStatus::INVALID_READING);
    }
    
    return TemperatureReading(lastStableTemp, ambientTemp, objectTemp,
                             isInContact(), SensorStatus::OK);
}

bool MLX90614TemperatureSensor::isInContact() {
    return !isnan(lastObjectTemp) && !isnan(lastAmbientTemp) &&
           (lastObjectTemp - lastAmbientTemp) >= CONTACT_DELTA;
}

void MLX90614TemperatureSensor::reset() {
    filter.clear();
    lastStableTemp = NAN;
    lastSeenTemp = NAN;
    settling = false;
    settleStartTime = 0;
    lastReadTime = 0;
    
    if (logger) logger->info("MLX90614 sensor reset");
}