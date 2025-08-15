#ifndef AZURE_IOT_CLIENT_H
#define AZURE_IOT_CLIENT_H

#include "../../include/IIoTClient.h"
#include "../../include/ILogger.h"
#include "../../include/Config.h"
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>
#include <az_core.h>
#include <az_iot.h>
#include <azure_ca.h>
#include <base64.h>
#include <bearssl/bearssl.h>
#include <bearssl/bearssl_hmac.h>
#include <libb64/cdecode.h>

class AzureIoTClient : public IIoTClient {
private:
    ILogger* logger;
    const Config* config;
    
    WiFiClientSecure wifiClient;
    PubSubClient mqttClient;
    X509List cert;
    az_iot_hub_client client;
    
    char sasToken[300];
    uint8_t signature[512];
    unsigned char encryptedSignature[32];
    char base64DecodedDeviceKey[32];
    time_t sasExpiryTime;
    
    ConnectionInfo connectionInfo;
    bool initialized;
    int connectionFailures;
    unsigned long lastConnectionCheck;
    
    static constexpr int MAX_CONNECTION_FAILURES = 5;
    static constexpr unsigned long CONNECTION_CHECK_INTERVAL = 30000;
    static constexpr int MQTT_PACKET_SIZE = 512;
    static constexpr int ONE_HOUR_IN_SECS = 3600;
    
    bool initializeTime();
    bool initializeAzureClient();
    bool generateSasToken();
    bool connectWiFi();
    bool connectMQTT();
    void updateConnectionInfo();
    void handleConnectionFailure();

public:
    AzureIoTClient(ILogger* log, const Config* cfg);
    
    bool initialize() override;
    bool connect() override;
    void disconnect() override;
    bool isConnected() const override;
    bool sendData(const VitalSigns& data) override;
    ConnectionInfo getConnectionInfo() const override;
    void loop() override;
    bool refreshAuthentication() override;
    
    void setKeepAlive(int seconds);
    void setOutputPower(float dbm);
    void enableModemSleep(bool enable);
};

#endif