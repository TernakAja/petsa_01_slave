#include "../../include/Config.h"
#include "../../iot_configs.h"

void Config::loadFromDefaults() {
    wifi.ssid = IOT_CONFIG_WIFI_SSID;
    wifi.password = IOT_CONFIG_WIFI_PASSWORD;
    wifi.maxRetries = 6;
    wifi.retryDelay = 1000;
    
    azure.hubFqdn = IOT_CONFIG_IOTHUB_FQDN;
    azure.deviceId = IOT_CONFIG_DEVICE_ID;
    azure.deviceKey = IOT_CONFIG_DEVICE_KEY;
    azure.port = 8883;
    azure.sasTokenLifetime = 3600;
    
    i2c.sdaPin = D2;
    i2c.sclPin = D1;
    i2c.clockSpeed = 100000;
    i2c.stretchLimit = 250000;
    
    sensor.collectionInterval = 3000;
    sensor.sendInterval = 60000;
    sensor.spo2Interval = 10000;
    sensor.maxDataPoints = 30;
    
    powerMode = PowerMode::PERFORMANCE;
    ledPin = 2;
}

bool Config::validate() const {
    if (wifi.ssid.length() == 0) {
        Serial.println("ERROR: WiFi SSID not configured");
        return false;
    }
    
    if (wifi.password.length() == 0) {
        Serial.println("ERROR: WiFi password not configured");
        return false;
    }
    
    if (azure.hubFqdn.length() == 0) {
        Serial.println("ERROR: Azure IoT Hub FQDN not configured");
        return false;
    }
    
    if (azure.deviceId.length() == 0) {
        Serial.println("ERROR: Azure device ID not configured");
        return false;
    }
    
    if (azure.deviceKey.length() == 0) {
        Serial.println("ERROR: Azure device key not configured");
        return false;
    }
    
    if (sensor.collectionInterval < 1000 || sensor.collectionInterval > 60000) {
        Serial.println("ERROR: Collection interval must be between 1-60 seconds");
        return false;
    }
    
    if (sensor.sendInterval < 10000 || sensor.sendInterval > 3600000) {
        Serial.println("ERROR: Send interval must be between 10 seconds - 1 hour");
        return false;
    }
    
    if (sensor.maxDataPoints < 1 || sensor.maxDataPoints > 1000) {
        Serial.println("ERROR: Max data points must be between 1-1000");
        return false;
    }
    
    if (i2c.clockSpeed < 50000 || i2c.clockSpeed > 400000) {
        Serial.println("ERROR: I2C clock speed must be between 50-400 kHz");
        return false;
    }
    
    return true;
}

void Config::print() const {
    Serial.println("================ CONFIGURATION ================");
    
    Serial.println("[WiFi Configuration]");
    Serial.print("SSID: "); Serial.println(wifi.ssid);
    Serial.print("Password: "); 
    for (int i = 0; i < wifi.password.length(); i++) Serial.print("*");
    Serial.println();
    Serial.print("Max Retries: "); Serial.println(wifi.maxRetries);
    Serial.print("Retry Delay: "); Serial.print(wifi.retryDelay); Serial.println(" ms");
    
    Serial.println("\n[Azure IoT Configuration]");
    Serial.print("Hub FQDN: "); Serial.println(azure.hubFqdn);
    Serial.print("Device ID: "); Serial.println(azure.deviceId);
    Serial.print("Device Key: ");
    for (int i = 0; i < azure.deviceKey.length(); i++) Serial.print("*");
    Serial.println();
    Serial.print("Port: "); Serial.println(azure.port);
    Serial.print("SAS Token Lifetime: "); Serial.print(azure.sasTokenLifetime); Serial.println(" seconds");
    
    Serial.println("\n[I2C Configuration]");
    Serial.print("SDA Pin: D"); Serial.println(i2c.sdaPin);
    Serial.print("SCL Pin: D"); Serial.println(i2c.sclPin);
    Serial.print("Clock Speed: "); Serial.print(i2c.clockSpeed / 1000); Serial.println(" kHz");
    Serial.print("Stretch Limit: "); Serial.print(i2c.stretchLimit / 1000); Serial.println(" ms");
    
    Serial.println("\n[Sensor Configuration]");
    Serial.print("Collection Interval: "); Serial.print(sensor.collectionInterval); Serial.println(" ms");
    Serial.print("Send Interval: "); Serial.print(sensor.sendInterval); Serial.println(" ms");
    Serial.print("SpO2 Interval: "); Serial.print(sensor.spo2Interval); Serial.println(" ms");
    Serial.print("Max Data Points: "); Serial.println(sensor.maxDataPoints);
    
    Serial.println("\n[System Configuration]");
    Serial.print("Power Mode: ");
    switch (powerMode) {
        case PowerMode::PERFORMANCE: Serial.println("PERFORMANCE"); break;
        case PowerMode::ECO: Serial.println("ECO"); break;
        case PowerMode::ULTRA_ECO: Serial.println("ULTRA_ECO"); break;
        default: Serial.println("UNKNOWN"); break;
    }
    Serial.print("LED Pin: "); Serial.println(ledPin);
    
    Serial.println("===============================================");
}