#ifndef PETSA_APPLICATION_H
#define PETSA_APPLICATION_H

#include "../include/Config.h"
#include "../include/ILogger.h"
#include "../include/ITemperatureSensor.h"
#include "../include/IHeartRateSensor.h"
#include "../include/IDataProcessor.h"
#include "../include/IIoTClient.h"
#include "logging/SerialLogger.h"
#include "ui/EventCardUI.h"
#include "sensors/MLX90614TemperatureSensor.h"
#include "sensors/MAX30105HeartRateSensor.h"
#include "data/DataProcessor.h"
#include "communication/AzureIoTClient.h"

enum class ApplicationState {
    INITIALIZING,
    CONNECTING,
    COLLECTING_DATA,
    PROCESSING_DATA,
    TRANSMITTING,
    ERROR,
    SLEEPING
};

class PETSAApplication {
private:
    Config config;
    SerialLogger logger;
    EventCardUI ui;
    
    MLX90614TemperatureSensor* temperatureSensor;
    MAX30105HeartRateSensor* heartRateSensor;
    DataProcessor* dataProcessor;
    AzureIoTClient* iotClient;
    
    ApplicationState currentState;
    unsigned long lastCollectionTime;
    unsigned long lastSendTime;
    unsigned long lastSpO2Time;
    unsigned long lastHeartRateTime;
    unsigned long sensorRetryTime;
    
    uint8_t spo2BackoffIndex;
    static const uint32_t SPO2_BACKOFF_MS[];
    static const int LED_PIN = 2;
    
    enum LedMode { LED_OFF, LED_FAST, LED_SLOW, LED_ERROR };
    LedMode ledMode;
    unsigned long lastLedToggle;
    bool ledState;
    
    bool initializeHardware();
    bool initializeSensors();
    bool initializeIoT();
    void setupI2C();
    void scanI2C();
    bool recoverI2C();
    
    void updateLED();
    void setLED(LedMode mode);
    
    void collectSensorData();
    void processSensorReadings();
    void transmitData();
    void handleSensorRetry();
    
    void printBootInfo();
    void printSystemStatus();
    
public:
    PETSAApplication();
    ~PETSAApplication();
    
    bool initialize();
    void run();
    void loop();
    
    ApplicationState getState() const { return currentState; }
    const Config& getConfig() const { return config; }
    ILogger& getLogger() { return logger; }
    EventCardUI& getUI() { return ui; }
};

#endif