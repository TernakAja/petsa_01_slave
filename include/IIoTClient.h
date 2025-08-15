#ifndef IIOT_CLIENT_H
#define IIOT_CLIENT_H

#include <Arduino.h>
#include "IDataProcessor.h"

enum class ConnectionStatus {
    DISCONNECTED,
    CONNECTING,
    CONNECTED,
    ERROR,
    WIFI_FAILED,
    MQTT_FAILED
};

struct ConnectionInfo {
    ConnectionStatus wifiStatus;
    ConnectionStatus mqttStatus;
    int signalStrength;
    String ipAddress;
    unsigned long lastConnected;
    
    ConnectionInfo() : wifiStatus(ConnectionStatus::DISCONNECTED), 
                      mqttStatus(ConnectionStatus::DISCONNECTED),
                      signalStrength(0), lastConnected(0) {}
};

class IIoTClient {
public:
    virtual ~IIoTClient() = default;
    virtual bool initialize() = 0;
    virtual bool connect() = 0;
    virtual void disconnect() = 0;
    virtual bool isConnected() const = 0;
    virtual bool sendData(const VitalSigns& data) = 0;
    virtual ConnectionInfo getConnectionInfo() const = 0;
    virtual void loop() = 0;
    virtual bool refreshAuthentication() = 0;
};

#endif