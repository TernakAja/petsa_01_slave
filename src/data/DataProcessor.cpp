#include "DataProcessor.h"

DataProcessor::DataProcessor(ILogger* log, const Config* cfg)
    : logger(log), config(cfg), temperatureData(nullptr), heartRateData(nullptr), 
      spo2Data(nullptr), currentDataPoints(0), maxDataPoints(cfg->sensor.maxDataPoints),
      lastCollectionTime(0) {
    allocateBuffers();
    if (logger) logger->infof("DataProcessor initialized with %d data points", maxDataPoints);
}

DataProcessor::~DataProcessor() {
    deallocateBuffers();
}

void DataProcessor::allocateBuffers() {
    deallocateBuffers();
    
    temperatureData = new float[maxDataPoints];
    heartRateData = new float[maxDataPoints];
    spo2Data = new float[maxDataPoints];
    
    if (!temperatureData || !heartRateData || !spo2Data) {
        if (logger) logger->critical("Failed to allocate memory for data buffers");
        deallocateBuffers();
        return;
    }
    
    for (int i = 0; i < maxDataPoints; i++) {
        temperatureData[i] = NAN;
        heartRateData[i] = NAN;
        spo2Data[i] = NAN;
    }
    
    if (logger) logger->debugf("Allocated buffers for %d data points", maxDataPoints);
}

void DataProcessor::deallocateBuffers() {
    delete[] temperatureData;
    delete[] heartRateData;
    delete[] spo2Data;
    
    temperatureData = nullptr;
    heartRateData = nullptr;
    spo2Data = nullptr;
}

void DataProcessor::addDataPoint(float temperature, float heartRate, float spo2) {
    if (!temperatureData || !heartRateData || !spo2Data) {
        if (logger) logger->error("Data buffers not allocated");
        return;
    }
    
    if (currentDataPoints >= maxDataPoints) {
        if (logger) logger->warning("Data buffer full, cannot add more data points");
        return;
    }
    
    temperatureData[currentDataPoints] = temperature;
    heartRateData[currentDataPoints] = heartRate;
    spo2Data[currentDataPoints] = spo2;
    
    currentDataPoints++;
    lastCollectionTime = millis();
    
    if (logger) {
        logger->debugf("Added data point %d/%d - Temp: %.2f, HR: %.2f, SpO2: %.1f", 
                      currentDataPoints, maxDataPoints, temperature, heartRate, spo2);
    }
}

float DataProcessor::calculateAverage(const float* data, int count, bool (*validator)(float)) const {
    if (!data || count == 0) return NAN;
    
    float sum = 0.0f;
    int validCount = 0;
    
    for (int i = 0; i < count; i++) {
        if (validator(data[i])) {
            sum += data[i];
            validCount++;
        }
    }
    
    return (validCount > 0) ? (sum / validCount) : NAN;
}

int DataProcessor::countValidData(const float* data, int count, bool (*validator)(float)) const {
    if (!data || count == 0) return 0;
    
    int validCount = 0;
    for (int i = 0; i < count; i++) {
        if (validator(data[i])) {
            validCount++;
        }
    }
    
    return validCount;
}

VitalSigns DataProcessor::getAveragedData() {
    if (currentDataPoints == 0) {
        if (logger) logger->warning("No data points available for averaging");
        return VitalSigns();
    }
    
    float avgTemp = calculateAverage(temperatureData, currentDataPoints, DataValidator::isValidTemperature);
    float avgHR = calculateAverage(heartRateData, currentDataPoints, DataValidator::isValidHeartRate);
    float avgSpO2 = calculateAverage(spo2Data, currentDataPoints, DataValidator::isValidSpO2);
    
    int tempCount = countValidData(temperatureData, currentDataPoints, DataValidator::isValidTemperature);
    int hrCount = countValidData(heartRateData, currentDataPoints, DataValidator::isValidHeartRate);
    int spo2Count = countValidData(spo2Data, currentDataPoints, DataValidator::isValidSpO2);
    
    if (isnan(avgTemp) && currentDataPoints > 0) {
        avgTemp = 27.0f; // Default fallback temperature
        if (logger) logger->warning("Using fallback temperature due to no valid readings");
    }
    
    if (isnan(avgHR)) {
        avgHR = 0.0f; // Default fallback heart rate
        if (logger) logger->warning("No valid heart rate readings");
    }
    
    if (isnan(avgSpO2)) {
        avgSpO2 = 98.0f; // Default fallback SpO2
        if (logger) logger->warning("Using fallback SpO2 due to no valid readings");
    }
    
    VitalSigns result(avgTemp, avgHR, avgSpO2, millis(), currentDataPoints);
    
    if (logger) {
        logger->infof("Averaged data - Temp: %.2f°C (%d), HR: %.2f (%d), SpO2: %.1f%% (%d)",
                     avgTemp, tempCount, avgHR, hrCount, avgSpO2, spo2Count);
    }
    
    return DataValidator::validateVitalSigns(result);
}

bool DataProcessor::isReadyToSend() const {
    return currentDataPoints >= maxDataPoints;
}

void DataProcessor::reset() {
    currentDataPoints = 0;
    lastCollectionTime = 0;
    
    if (temperatureData && heartRateData && spo2Data) {
        for (int i = 0; i < maxDataPoints; i++) {
            temperatureData[i] = NAN;
            heartRateData[i] = NAN;
            spo2Data[i] = NAN;
        }
    }
    
    if (logger) logger->debug("Data processor reset");
}

void DataProcessor::setMaxDataPoints(int points) {
    if (points <= 0 || points > 1000) { // Safety limit
        if (logger) logger->errorf("Invalid max data points: %d", points);
        return;
    }
    
    if (points != maxDataPoints) {
        maxDataPoints = points;
        allocateBuffers();
        reset();
        
        if (logger) logger->infof("Max data points changed to %d", maxDataPoints);
    }
}

float DataProcessor::getDataPoint(int index, int type) const {
    if (index < 0 || index >= currentDataPoints) {
        return NAN;
    }
    
    switch (type) {
        case 0: return temperatureData ? temperatureData[index] : NAN;
        case 1: return heartRateData ? heartRateData[index] : NAN;
        case 2: return spo2Data ? spo2Data[index] : NAN;
        default: return NAN;
    }
}