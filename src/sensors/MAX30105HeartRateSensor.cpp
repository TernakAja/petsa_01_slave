#include "MAX30105HeartRateSensor.h"
#include <Wire.h>

MAX30105HeartRateSensor::MAX30105HeartRateSensor(ILogger* log, const Config* cfg)
    : logger(log), config(cfg), lastBeat(0), beatsPerMinute(0.0f), spo2Value(98.0f),
      lastReadTime(0), isAwake(true), initialized(false), bpmIndex(0) {
    memset(bpmBuffer, 0, sizeof(bpmBuffer));
}

bool MAX30105HeartRateSensor::initialize() {
    if (logger) logger->info("Initializing MAX30105 heart rate sensor");
    
    Wire.setClock(config->i2c.clockSpeed);
    Wire.setClockStretchLimit(config->i2c.stretchLimit);
    
    for (int retry = 0; retry < 10 && !initialized; retry++) {
        if (logger) logger->debugf("MAX30105 init attempt %d", retry + 1);
        
        if (particleSensor.begin(Wire, config->i2c.clockSpeed)) {
            if (setupSensor()) {
                initialized = true;
                break;
            }
        }
        delay(250);
        yield();
    }
    
    if (initialized) {
        if (logger) logger->info("MAX30105 initialized successfully");
    } else {
        if (logger) logger->error("MAX30105 initialization failed");
    }
    
    return initialized;
}

bool MAX30105HeartRateSensor::setupSensor() {
    try {
        particleSensor.setPulseAmplitudeRed(0x1F);
        particleSensor.setPulseAmplitudeIR(0x1F);
        particleSensor.setPulseAmplitudeGreen(0x00);
        particleSensor.setup(60, 4, 2, 100, 411, 4096);
        return true;
    } catch (...) {
        if (logger) logger->error("MAX30105 sensor setup failed");
        return false;
    }
}

bool MAX30105HeartRateSensor::detectFinger() const {
    return particleSensor.getIR() >= FINGER_THRESHOLD;
}

bool MAX30105HeartRateSensor::isFingerDetected() {
    if (!initialized) return false;
    if (!isAwake) exitSleepMode();
    return detectFinger();
}

void MAX30105HeartRateSensor::enterSleepMode() {
    if (initialized && isAwake) {
        particleSensor.shutDown();
        isAwake = false;
        if (logger) logger->debug("MAX30105 entered sleep mode");
    }
}

void MAX30105HeartRateSensor::exitSleepMode() {
    if (initialized && !isAwake) {
        particleSensor.wakeUp();
        isAwake = true;
        if (logger) logger->debug("MAX30105 exited sleep mode");
    }
}

float MAX30105HeartRateSensor::processHeartBeatData() {
    if (!isAwake) exitSleepMode();
    
    long irValue = particleSensor.getIR();
    if (irValue < FINGER_THRESHOLD) {
        beatsPerMinute = 0;
        return beatsPerMinute;
    }
    
    static float prevIR = 0;
    unsigned long startTime = millis();
    
    while ((millis() - startTime) < MAX_READ_TIME) {
        long currentIR = particleSensor.getIR();
        
        float filtered = currentIR - 0.95f * prevIR;
        prevIR = currentIR;
        
        if (checkForBeat(filtered)) {
            unsigned long currentTime = millis();
            unsigned long delta = currentTime - lastBeat;
            lastBeat = currentTime;
            
            if (delta > MIN_BEAT_INTERVAL && delta < MAX_BEAT_INTERVAL) {
                float newBPM = 60.0f / (delta / 1000.0f);
                if (newBPM >= MIN_BPM && newBPM <= MAX_BPM) {
                    bpmBuffer[(bpmIndex++) & 3] = newBPM;
                    float sum = bpmBuffer[0] + bpmBuffer[1] + bpmBuffer[2] + bpmBuffer[3];
                    beatsPerMinute = sum / 4.0f;
                    
                    if (logger) logger->debugf("Heart beat detected: %.1f BPM", beatsPerMinute);
                    return beatsPerMinute;
                }
            }
        }
        delay(15);
        yield();
    }
    
    return beatsPerMinute;
}

bool MAX30105HeartRateSensor::collectSpO2Data() {
    if (logger) logger->debug("Starting SpO2 data collection");
    
    exitSleepMode();
    
    long irValue = particleSensor.getIR();
    if (irValue < SPO2_FINGER_THRESHOLD) {
        if (logger) logger->warningf("No finger detected for SpO2 (IR=%ld)", irValue);
        enterSleepMode();
        return false;
    }
    
    for (byte i = 0; i < BUFFER_SIZE; i++) {
        while (!particleSensor.available()) {
            particleSensor.check();
            yield();
        }
        redBuffer[i] = particleSensor.getRed();
        irBuffer[i] = particleSensor.getIR();
        particleSensor.nextSample();
        
        if (i % 25 == 0 && logger) {
            int percent = (i * 100) / (BUFFER_SIZE - 1);
            logger->debugf("SpO2 sampling %d%%", percent);
        }
        
        delay(10);
        yield();
    }
    
    int32_t spo2;
    int8_t validSPO2;
    int32_t heartRate;
    int8_t validHeartRate;
    
    maxim_heart_rate_and_oxygen_saturation(irBuffer, BUFFER_SIZE, redBuffer,
                                           &spo2, &validSPO2, &heartRate, &validHeartRate);
    
    bool hrValid = (validHeartRate && heartRate > 30 && heartRate < 200);
    bool spValid = (validSPO2 && spo2 >= 70 && spo2 <= 100);
    
    if (spValid) {
        spo2Value = (float)spo2;
        if (logger) logger->infof("SpO2 reading: %.1f%%", spo2Value);
    } else {
        if (logger) logger->warning("Invalid SpO2 calculation");
    }
    
    if (hrValid) {
        beatsPerMinute = (float)heartRate;
        if (logger) logger->infof("Heart rate from SpO2: %.1f BPM", beatsPerMinute);
    }
    
    enterSleepMode();
    return spValid;
}

SensorReading MAX30105HeartRateSensor::read() {
    HeartRateReading hrReading = readHeartRate();
    return SensorReading(hrReading.value, hrReading.status);
}

HeartRateReading MAX30105HeartRateSensor::readHeartRate() {
    if (!initialized) {
        return HeartRateReading(NAN, NAN, 0, 0, SensorStatus::NOT_INITIALIZED);
    }
    
    if (millis() - lastReadTime < READ_INTERVAL) {
        return HeartRateReading(beatsPerMinute, spo2Value, 
                               particleSensor.getIR(), particleSensor.getRed(),
                               SensorStatus::OK);
    }
    
    lastReadTime = millis();
    
    if (!detectFinger()) {
        if (logger) logger->debug("No finger detected for heart rate");
        return HeartRateReading(0.0f, spo2Value, 
                               particleSensor.getIR(), particleSensor.getRed(),
                               SensorStatus::NO_DATA);
    }
    
    float newBPM = processHeartBeatData();
    
    return HeartRateReading(newBPM, spo2Value,
                           particleSensor.getIR(), particleSensor.getRed(),
                           SensorStatus::OK);
}

SensorReading MAX30105HeartRateSensor::readSpO2() {
    if (!initialized) {
        return SensorReading(NAN, SensorStatus::NOT_INITIALIZED);
    }
    
    if (!detectFinger()) {
        if (logger) logger->warning("No finger detected for SpO2 measurement");
        return SensorReading(spo2Value, SensorStatus::NO_DATA);
    }
    
    bool success = collectSpO2Data();
    
    return SensorReading(spo2Value, 
                        success ? SensorStatus::OK : SensorStatus::INVALID_READING);
}

void MAX30105HeartRateSensor::reset() {
    memset(bpmBuffer, 0, sizeof(bpmBuffer));
    bpmIndex = 0;
    beatsPerMinute = 0.0f;
    lastBeat = 0;
    lastReadTime = 0;
    
    if (initialized) {
        exitSleepMode();
        setupSensor();
    }
    
    if (logger) logger->info("MAX30105 sensor reset");
}