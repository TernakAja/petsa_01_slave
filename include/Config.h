#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

enum class PowerMode {
    PERFORMANCE,    // Always connected, high power
    ECO,           // Radio sleeps between transmissions
    ULTRA_ECO      // Deep sleep between transmissions
};

struct I2CConfig {
    uint8_t sdaPin;
    uint8_t sclPin;
    uint32_t clockSpeed;
    uint32_t stretchLimit;
    
    I2CConfig(uint8_t sda = D2, uint8_t scl = D1, 
              uint32_t speed = 100000, uint32_t stretch = 250000)
        : sdaPin(sda), sclPin(scl), clockSpeed(speed), stretchLimit(stretch) {}
};

struct SensorConfig {
    unsigned long collectionInterval;
    unsigned long sendInterval;
    unsigned long spo2Interval;
    int maxDataPoints;
    
    SensorConfig(unsigned long collection = 3000, unsigned long send = 60000,
                unsigned long spo2 = 10000, int points = 30)
        : collectionInterval(collection), sendInterval(send), 
          spo2Interval(spo2), maxDataPoints(points) {}
};

struct WiFiConfig {
    String ssid;
    String password;
    int maxRetries;
    unsigned long retryDelay;
    
    WiFiConfig(const String& s = "", const String& p = "", 
              int retries = 6, unsigned long delay = 1000)
        : ssid(s), password(p), maxRetries(retries), retryDelay(delay) {}
};

struct AzureConfig {
    String hubFqdn;
    String deviceId;
    String deviceKey;
    int port;
    unsigned long sasTokenLifetime;
    
    AzureConfig(const String& hub = "", const String& device = "", 
               const String& key = "", int p = 8883, 
               unsigned long lifetime = 3600)
        : hubFqdn(hub), deviceId(device), deviceKey(key), 
          port(p), sasTokenLifetime(lifetime) {}
};

class Config {
public:
    I2CConfig i2c;
    SensorConfig sensor;
    WiFiConfig wifi;
    AzureConfig azure;
    PowerMode powerMode;
    uint8_t ledPin;
    
    Config() : powerMode(PowerMode::PERFORMANCE), ledPin(2) {}
    
    void loadFromDefaults();
    bool validate() const;
    void print() const;
};

#endif