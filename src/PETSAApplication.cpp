#include "PETSAApplication.h"
#include <Wire.h>
#include <ESP8266WiFi.h>

const uint32_t PETSAApplication::SPO2_BACKOFF_MS[] = {0, 5000, 10000, 20000, 30000, 60000, 120000};

PETSAApplication::PETSAApplication()
    : logger(LogLevel::INFO, true), ui(&logger), temperatureSensor(nullptr),
      heartRateSensor(nullptr), dataProcessor(nullptr), iotClient(nullptr),
      currentState(ApplicationState::INITIALIZING), lastCollectionTime(0),
      lastSendTime(0), lastSpO2Time(0), lastHeartRateTime(0), sensorRetryTime(0),
      spo2BackoffIndex(0), ledMode(LED_OFF), lastLedToggle(0), ledState(false) {
    
    config.loadFromDefaults();
}

PETSAApplication::~PETSAApplication() {
    delete temperatureSensor;
    delete heartRateSensor;
    delete dataProcessor;
    delete iotClient;
}

bool PETSAApplication::initialize() {
    currentState = ApplicationState::INITIALIZING;
    
    Serial.begin(115200);
    while (!Serial && millis() < 5000) {
        yield();
        delay(10);
    }
    
    logger.info("=== PETSA Health Monitoring System ===");
    
    if (!config.validate()) {
        logger.critical("Configuration validation failed");
        currentState = ApplicationState::ERROR;
        return false;
    }
    
    config.print();
    
    if (!initializeHardware()) {
        logger.critical("Hardware initialization failed");
        currentState = ApplicationState::ERROR;
        return false;
    }
    
    if (!initializeSensors()) {
        logger.error("Sensor initialization failed");
    }
    
    if (config.powerMode == PowerMode::PERFORMANCE) {
        if (!initializeIoT()) {
            logger.error("IoT initialization failed");
        }
    } else {
        logger.info("Eco mode: IoT will initialize on demand");
    }
    
    lastCollectionTime = millis();
    lastSendTime = millis();
    lastSpO2Time = millis();
    lastHeartRateTime = millis();
    sensorRetryTime = millis();
    
    currentState = ApplicationState::COLLECTING_DATA;
    setLED(LED_SLOW);
    
    logger.info("PETSA application initialized successfully");
    return true;
}

bool PETSAApplication::initializeHardware() {
    logger.info("Initializing hardware");
    
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);
    setLED(LED_FAST);
    
    setupI2C();
    scanI2C();
    
    printBootInfo();
    
    return true;
}

void PETSAApplication::setupI2C() {
    if (digitalRead(config.i2c.sdaPin) == LOW || digitalRead(config.i2c.sclPin) == LOW) {
        ui.cardHeader("I2C RECOVERY", "EVENT");
        ui.cardKV("state", "bus appears stuck");
        ui.cardFooter();
        
        bool recovered = recoverI2C();
        ui.pipeStart(false);
        ui.pipeStepEnd(recovered ? "I2C recovery OK" : "I2C recovery FAILED", 
                      recovered ? "✔" : "✖");
    }
    
    Wire.begin(config.i2c.sdaPin, config.i2c.sclPin);
    Wire.setClock(config.i2c.clockSpeed);
    Wire.setClockStretchLimit(config.i2c.stretchLimit);
    
    logger.infof("I2C initialized - SDA:%d SCL:%d Speed:%dkHz", 
                config.i2c.sdaPin, config.i2c.sclPin, config.i2c.clockSpeed / 1000);
}

bool PETSAApplication::recoverI2C() {
    pinMode(config.i2c.sdaPin, INPUT_PULLUP);
    pinMode(config.i2c.sclPin, INPUT_PULLUP);
    delay(2);
    
    if (digitalRead(config.i2c.sdaPin) == HIGH && digitalRead(config.i2c.sclPin) == HIGH) {
        return true;
    }
    
    if (digitalRead(config.i2c.sclPin) == LOW) {
        return false;
    }
    
    pinMode(config.i2c.sclPin, OUTPUT);
    for (uint8_t i = 0; i < 16 && digitalRead(config.i2c.sdaPin) == LOW; i++) {
        digitalWrite(config.i2c.sclPin, LOW);
        delayMicroseconds(10);
        digitalWrite(config.i2c.sclPin, HIGH);
        delayMicroseconds(10);
        yield();
    }
    
    pinMode(config.i2c.sdaPin, OUTPUT);
    digitalWrite(config.i2c.sdaPin, LOW);
    delayMicroseconds(10);
    digitalWrite(config.i2c.sclPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(config.i2c.sdaPin, HIGH);
    delayMicroseconds(10);
    
    Wire.begin(config.i2c.sdaPin, config.i2c.sclPin);
    Wire.setClock(config.i2c.clockSpeed);
    Wire.setClockStretchLimit(config.i2c.stretchLimit);
    
    pinMode(config.i2c.sdaPin, INPUT_PULLUP);
    pinMode(config.i2c.sclPin, INPUT_PULLUP);
    
    return (digitalRead(config.i2c.sdaPin) == HIGH && digitalRead(config.i2c.sclPin) == HIGH);
}

void PETSAApplication::scanI2C() {
    pinMode(config.i2c.sdaPin, INPUT_PULLUP);
    pinMode(config.i2c.sclPin, INPUT_PULLUP);
    int sda = digitalRead(config.i2c.sdaPin);
    int scl = digitalRead(config.i2c.sclPin);
    
    ui.cardHeader("I2C SCAN", "EVENT");
    ui.cardKVf("lines", "SDA=%d SCL=%d", sda, scl);
    
    int found = 0;
    for (byte address = 1; address < 127 && found < 24; address++) {
        Wire.beginTransmission(address);
        if (Wire.endTransmission() == 0) {
            char addressStr[16];
            snprintf(addressStr, sizeof(addressStr), "0x%02X", address);
            ui.cardKV("device", addressStr);
            found++;
        }
        delay(2);
        yield();
    }
    
    ui.cardKVf("found", "%d", found);
    ui.cardFooter();
    
    if (found == 0 && (sda == 0 || scl == 0)) {
        ui.pipeStart(false);
        ui.pipeStepEnd("I2C line stuck low. Check power/pull-ups/wiring.", "✖");
    }
}

bool PETSAApplication::initializeSensors() {
    logger.info("Initializing sensors");
    
    temperatureSensor = new MLX90614TemperatureSensor(&logger, &config);
    heartRateSensor = new MAX30105HeartRateSensor(&logger, &config);
    dataProcessor = new DataProcessor(&logger, &config);
    
    bool tempOk = temperatureSensor->initialize();
    delay(400);
    yield();
    
    bool heartOk = heartRateSensor->initialize();
    
    if (!tempOk || !heartOk) {
        logger.warning("Some sensors failed to initialize");
        return false;
    }
    
    logger.info("All sensors initialized successfully");
    return true;
}

bool PETSAApplication::initializeIoT() {
    logger.info("Initializing IoT communication");
    
    iotClient = new AzureIoTClient(&logger, &config);
    
    if (!iotClient->initialize()) {
        logger.error("IoT client initialization failed");
        return false;
    }
    
    if (config.powerMode == PowerMode::PERFORMANCE) {
        iotClient->setKeepAlive(45);
        iotClient->setOutputPower(10.5f);
        iotClient->enableModemSleep(true);
        
        if (!iotClient->connect()) {
            logger.error("IoT connection failed");
            return false;
        }
    }
    
    logger.info("IoT communication initialized successfully");
    return true;
}

void PETSAApplication::printBootInfo() {
    ui.cardHeader("BOOT", "EVENT");
    ui.cardKVf("heap", "%u B", ESP.getFreeHeap());
    
    char chipId[16];
    snprintf(chipId, sizeof(chipId), "%08X", ESP.getChipId());
    ui.cardKV("chip id", chipId);
    
    snprintf(chipId, sizeof(chipId), "%08X", ESP.getFlashChipId());
    ui.cardKV("flash id", chipId);
    
    const char* powerModeStr;
    switch (config.powerMode) {
        case PowerMode::PERFORMANCE: powerModeStr = "PERFORMANCE"; break;
        case PowerMode::ECO: powerModeStr = "ECO"; break;
        case PowerMode::ULTRA_ECO: powerModeStr = "ULTRA_ECO"; break;
        default: powerModeStr = "UNKNOWN"; break;
    }
    ui.cardKV("power mode", powerModeStr);
    
    ui.cardFooter();
}

void PETSAApplication::setLED(LedMode mode) {
    ledMode = mode;
}

void PETSAApplication::updateLED() {
    unsigned long now = millis();
    unsigned long period;
    
    switch (ledMode) {
        case LED_FAST: period = 200; break;
        case LED_SLOW: period = 800; break;
        case LED_ERROR: period = 100; break;
        default: digitalWrite(LED_PIN, HIGH); return;
    }
    
    if (now - lastLedToggle >= period) {
        lastLedToggle = now;
        ledState = !ledState;
        digitalWrite(LED_PIN, ledState ? LOW : HIGH);
    }
}

void PETSAApplication::run() {
    while (true) {
        loop();
        yield();
        delay(10);
    }
}

void PETSAApplication::loop() {
    updateLED();
    unsigned long currentTime = millis();
    
    if (currentState == ApplicationState::ERROR) {
        setLED(LED_ERROR);
        delay(1000);
        return;
    }
    
    handleSensorRetry();
    
    if (currentTime - lastSpO2Time >= (config.sensor.spo2Interval + SPO2_BACKOFF_MS[spo2BackoffIndex])) {
        SensorReading spo2Reading = heartRateSensor->readSpO2();
        bool validSpO2 = spo2Reading.isValid() && spo2Reading.value >= 70.0f && spo2Reading.value <= 100.0f;
        
        spo2BackoffIndex = validSpO2 ? 0 : min(spo2BackoffIndex + 1, 6);
        lastSpO2Time = currentTime;
    }
    
    if (currentTime - lastCollectionTime >= config.sensor.collectionInterval) {
        collectSensorData();
        lastCollectionTime = currentTime;
    }
    
    if (currentTime - lastSendTime >= config.sensor.sendInterval) {
        transmitData();
        lastSendTime = currentTime;
    }
    
    if (iotClient && config.powerMode == PowerMode::PERFORMANCE) {
        iotClient->loop();
    }
}

void PETSAApplication::collectSensorData() {
    currentState = ApplicationState::COLLECTING_DATA;
    
    TemperatureReading tempReading = temperatureSensor->readTemperature();
    HeartRateReading hrReading = heartRateSensor->readHeartRate();
    
    unsigned long currentTime = millis();
    if (currentTime - lastHeartRateTime >= 2000) {
        hrReading = heartRateSensor->readHeartRate();
        lastHeartRateTime = currentTime;
    }
    
    float temperature = tempReading.isValid() ? tempReading.value : NAN;
    float heartRate = hrReading.isValid() ? hrReading.value : NAN;
    float spo2 = hrReading.spo2;
    
    dataProcessor->addDataPoint(temperature, heartRate, spo2);
    
    ui.printCollectionProgress(dataProcessor->getCurrentDataPoints(), 
                              dataProcessor->getMaxDataPoints(),
                              millis(), ESP.getFreeHeap());
    
    printSystemStatus();
    
    ui.printSensorData(temperature, heartRate, spo2, 
                      tempReading.ambientTemp, tempReading.objectTemp);
}

void PETSAApplication::transmitData() {
    if (!dataProcessor->isReadyToSend()) {
        return;
    }
    
    currentState = ApplicationState::TRANSMITTING;
    
    VitalSigns data = dataProcessor->getAveragedData();
    
    bool success = false;
    if (iotClient) {
        if (config.powerMode == PowerMode::PERFORMANCE) {
            if (!iotClient->isConnected()) {
                iotClient->connect();
            }
            success = iotClient->sendData(data);
        } else {
            if (iotClient->connect()) {
                success = iotClient->sendData(data);
                if (config.powerMode == PowerMode::ECO) {
                    iotClient->disconnect();
                }
            }
        }
    }
    
    String topic = "devices/" + config.azure.deviceId + "/messages/events/";
    ui.printTransmissionResult(topic.c_str(), success,
                              data.temperature, 0,
                              data.heartRate, 0,
                              data.spo2, 0);
    
    dataProcessor->reset();
    currentState = ApplicationState::COLLECTING_DATA;
}

void PETSAApplication::handleSensorRetry() {
    unsigned long currentTime = millis();
    if (currentTime - sensorRetryTime >= 30000) {
        if (!temperatureSensor->isReady() || !heartRateSensor->isReady()) {
            logger.info("Retrying failed sensors");
            
            if (!temperatureSensor->isReady()) {
                temperatureSensor->initialize();
            }
            
            if (!heartRateSensor->isReady()) {
                heartRateSensor->initialize();
            }
        }
        sensorRetryTime = currentTime;
    }
}

void PETSAApplication::printSystemStatus() {
    ConnectionInfo connInfo;
    if (iotClient) {
        connInfo = iotClient->getConnectionInfo();
    }
    
    const char* wifiStatus = "Disconnected";
    const char* mqttStatus = "Disconnected";
    int rssi = 0;
    
    if (config.powerMode == PowerMode::PERFORMANCE) {
        switch (connInfo.wifiStatus) {
            case ConnectionStatus::CONNECTED: wifiStatus = "Connected"; break;
            case ConnectionStatus::CONNECTING: wifiStatus = "Connecting"; break;
            case ConnectionStatus::ERROR: wifiStatus = "Error"; break;
            default: wifiStatus = "Disconnected"; break;
        }
        
        switch (connInfo.mqttStatus) {
            case ConnectionStatus::CONNECTED: mqttStatus = "Connected"; break;
            case ConnectionStatus::CONNECTING: mqttStatus = "Connecting"; break;
            case ConnectionStatus::ERROR: mqttStatus = "Error"; break;
            default: mqttStatus = "Disconnected"; break;
        }
        
        rssi = connInfo.signalStrength;
    } else {
        wifiStatus = "Sleeping";
        mqttStatus = "Sleeping";
    }
    
    ui.printSystemStatus(wifiStatus, mqttStatus, config.azure.deviceId.c_str(), rssi);
}