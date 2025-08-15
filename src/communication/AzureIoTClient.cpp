#include "AzureIoTClient.h"
#include <time.h>

AzureIoTClient::AzureIoTClient(ILogger* log, const Config* cfg)
    : logger(log), config(cfg), cert((const char*)ca_pem), mqttClient(wifiClient),
      sasExpiryTime(0), initialized(false), connectionFailures(0), lastConnectionCheck(0) {
}

bool AzureIoTClient::initialize() {
    if (logger) logger->info("Initializing Azure IoT Client");
    
    if (!initializeAzureClient()) {
        if (logger) logger->error("Failed to initialize Azure client");
        return false;
    }
    
    wifiClient.setTrustAnchors(&cert);
    mqttClient.setServer(config->azure.hubFqdn.c_str(), config->azure.port);
    mqttClient.setBufferSize(MQTT_PACKET_SIZE);
    mqttClient.setKeepAlive(60);
    
    initialized = true;
    if (logger) logger->info("Azure IoT Client initialized successfully");
    return true;
}

bool AzureIoTClient::initializeAzureClient() {
    az_iot_hub_client_options options = az_iot_hub_client_options_default();
    options.user_agent = AZ_SPAN_FROM_STR("c%2F" AZ_SDK_VERSION_STRING "(ard;esp8266)");
    
    az_result result = az_iot_hub_client_init(&client,
        az_span_create((uint8_t*)config->azure.hubFqdn.c_str(), config->azure.hubFqdn.length()),
        az_span_create((uint8_t*)config->azure.deviceId.c_str(), config->azure.deviceId.length()),
        &options);
    
    if (az_result_failed(result)) {
        if (logger) logger->errorf("Azure client init failed: %d", result);
        return false;
    }
    
    return true;
}

bool AzureIoTClient::initializeTime() {
    if (logger) logger->info("Synchronizing time with NTP");
    
    configTime(0, 0, "pool.ntp.org", "time.nist.gov");
    time_t now = time(NULL);
    int attempts = 0;
    const int maxAttempts = 30;
    
    while (now < 1510592825 && attempts < maxAttempts) {
        delay(300);
        now = time(NULL);
        attempts++;
        yield();
        
        if (attempts % 5 == 0 && logger) {
            logger->debugf("Time sync attempt %d/%d", attempts, maxAttempts);
        }
    }
    
    bool synced = (now >= 1510592825);
    if (synced) {
        char timeStr[32];
        strftime(timeStr, sizeof(timeStr), "%Y-%m-%d %H:%M:%S", gmtime(&now));
        if (logger) logger->infof("Time synchronized: %s", timeStr);
    } else {
        if (logger) logger->warning("Time synchronization failed");
    }
    
    return synced;
}

bool AzureIoTClient::connectWiFi() {
    if (WiFi.status() == WL_CONNECTED) {
        updateConnectionInfo();
        return true;
    }
    
    if (logger) logger->infof("Connecting to WiFi: %s", config->wifi.ssid.c_str());
    
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(50);
    WiFi.begin(config->wifi.ssid.c_str(), config->wifi.password.c_str());
    
    for (int attempt = 1; attempt <= config->wifi.maxRetries; attempt++) {
        if (logger) logger->debugf("WiFi connection attempt %d/%d", attempt, config->wifi.maxRetries);
        
        for (int i = 0; i < 5 && WiFi.status() != WL_CONNECTED; i++) {
            delay(config->wifi.retryDelay);
            yield();
        }
        
        if (WiFi.status() == WL_CONNECTED) {
            updateConnectionInfo();
            if (logger) {
                logger->infof("WiFi connected - IP: %s, RSSI: %d dBm", 
                             connectionInfo.ipAddress.c_str(), connectionInfo.signalStrength);
            }
            return true;
        }
    }
    
    connectionInfo.wifiStatus = ConnectionStatus::WIFI_FAILED;
    if (logger) logger->error("WiFi connection failed");
    return false;
}

bool AzureIoTClient::generateSasToken() {
    az_span signature_span = az_span_create((uint8_t*)signature, sizeof(signature));
    az_span out_signature_span;
    uint32_t expiration = time(NULL) + config->azure.sasTokenLifetime;
    
    if (expiration < 1510592825) {
        if (logger) logger->error("Invalid time for SAS token generation");
        return false;
    }
    
    az_result result = az_iot_hub_client_sas_get_signature(&client, expiration, 
                                                          signature_span, &out_signature_span);
    if (az_result_failed(result)) {
        if (logger) logger->errorf("SAS signature generation failed: %d", result);
        return false;
    }
    
    int keyLen = base64_decode_chars(config->azure.deviceKey.c_str(), 
                                    config->azure.deviceKey.length(), 
                                    base64DecodedDeviceKey);
    if (keyLen == 0) {
        if (logger) logger->error("Device key decode failed");
        return false;
    }
    
    br_hmac_key_context kc;
    br_hmac_key_init(&kc, &br_sha256_vtable, base64DecodedDeviceKey, keyLen);
    br_hmac_context hmac_ctx;
    br_hmac_init(&hmac_ctx, &kc, 32);
    br_hmac_update(&hmac_ctx, az_span_ptr(out_signature_span), az_span_size(out_signature_span));
    br_hmac_out(&hmac_ctx, encryptedSignature);
    
    String b64enc_sig = base64::encode(encryptedSignature, br_hmac_size(&hmac_ctx));
    az_span b64enc_sig_span = az_span_create((uint8_t*)b64enc_sig.c_str(), b64enc_sig.length());
    
    result = az_iot_hub_client_sas_get_password(&client, expiration, b64enc_sig_span, 
                                               AZ_SPAN_EMPTY, sasToken, sizeof(sasToken), NULL);
    if (az_result_failed(result)) {
        if (logger) logger->errorf("SAS password generation failed: %d", result);
        return false;
    }
    
    sasExpiryTime = time(NULL) + config->azure.sasTokenLifetime - 60;
    if (logger) logger->debug("SAS token generated successfully");
    return true;
}

bool AzureIoTClient::connectMQTT() {
    if (!initialized) {
        if (logger) logger->error("Azure client not initialized");
        return false;
    }
    
    if (WiFi.status() != WL_CONNECTED) {
        connectionInfo.mqttStatus = ConnectionStatus::WIFI_FAILED;
        return false;
    }
    
    char mqttClientId[128];
    size_t clientIdLength;
    az_result result = az_iot_hub_client_get_client_id(&client, mqttClientId, 
                                                      sizeof(mqttClientId) - 1, &clientIdLength);
    if (az_result_failed(result)) {
        if (logger) logger->errorf("MQTT client ID generation failed: %d", result);
        return false;
    }
    mqttClientId[clientIdLength] = '\0';
    
    char mqttUsername[256];
    result = az_iot_hub_client_get_user_name(&client, mqttUsername, sizeof(mqttUsername), NULL);
    if (az_result_failed(result)) {
        if (logger) logger->errorf("MQTT username generation failed: %d", result);
        return false;
    }
    
    if (logger) logger->infof("Connecting to MQTT with client ID: %s", mqttClientId);
    
    connectionInfo.mqttStatus = ConnectionStatus::CONNECTING;
    
    for (int attempt = 1; attempt <= 3; attempt++) {
        if (logger) logger->debugf("MQTT connection attempt %d/3", attempt);
        
        if (mqttClient.connect(mqttClientId, mqttUsername, sasToken, NULL, 0, false, NULL, true)) {
            String deviceBoundTopic = "devices/" + config->azure.deviceId + "/messages/devicebound/#";
            mqttClient.subscribe(deviceBoundTopic.c_str());
            mqttClient.subscribe("$iothub/methods/POST/#");
            
            connectionInfo.mqttStatus = ConnectionStatus::CONNECTED;
            connectionInfo.lastConnected = millis();
            
            if (logger) logger->info("MQTT connected successfully");
            return true;
        }
        
        delay(1200);
        yield();
    }
    
    connectionInfo.mqttStatus = ConnectionStatus::MQTT_FAILED;
    if (logger) logger->error("MQTT connection failed");
    return false;
}

bool AzureIoTClient::connect() {
    if (!initialized) {
        if (logger) logger->error("Client not initialized");
        return false;
    }
    
    if (!connectWiFi()) {
        handleConnectionFailure();
        return false;
    }
    
    if (!initializeTime()) {
        if (logger) logger->warning("Time sync failed, continuing anyway");
    }
    
    if (!generateSasToken()) {
        handleConnectionFailure();
        return false;
    }
    
    if (!connectMQTT()) {
        handleConnectionFailure();
        return false;
    }
    
    connectionFailures = 0;
    return true;
}

void AzureIoTClient::disconnect() {
    if (mqttClient.connected()) {
        mqttClient.disconnect();
    }
    
    WiFi.disconnect(true);
    connectionInfo.wifiStatus = ConnectionStatus::DISCONNECTED;
    connectionInfo.mqttStatus = ConnectionStatus::DISCONNECTED;
    
    if (logger) logger->info("Disconnected from Azure IoT Hub");
}

bool AzureIoTClient::isConnected() const {
    return WiFi.status() == WL_CONNECTED && mqttClient.connected();
}

bool AzureIoTClient::sendData(const VitalSigns& data) {
    if (!isConnected()) {
        if (logger) logger->error("Cannot send data - not connected");
        return false;
    }
    
    String topic = "devices/" + config->azure.deviceId + "/messages/events/";
    
    char payload[256];
    char timestamp[32];
    time_t now = time(NULL);
    strftime(timestamp, sizeof(timestamp), "%Y-%m-%dT%H:%M:%SZ", gmtime(&now));
    
    snprintf(payload, sizeof(payload),
        "{\"deviceId\":\"%s\",\"pulseRate\":%.2f,\"temperature\":%.2f,"
        "\"sp02\":%.1f,\"timestamp\":\"%s\"}",
        config->azure.deviceId.c_str(), data.heartRate, data.temperature, 
        data.spo2, timestamp);
    
    bool success = mqttClient.publish(topic.c_str(), payload);
    
    if (success) {
        if (logger) logger->infof("Data sent successfully: %s", payload);
    } else {
        if (logger) logger->errorf("Failed to send data: %s", payload);
        handleConnectionFailure();
    }
    
    return success;
}

ConnectionInfo AzureIoTClient::getConnectionInfo() const {
    return connectionInfo;
}

void AzureIoTClient::updateConnectionInfo() {
    if (WiFi.status() == WL_CONNECTED) {
        connectionInfo.wifiStatus = ConnectionStatus::CONNECTED;
        connectionInfo.ipAddress = WiFi.localIP().toString();
        connectionInfo.signalStrength = WiFi.RSSI();
    } else {
        connectionInfo.wifiStatus = ConnectionStatus::DISCONNECTED;
        connectionInfo.ipAddress = "";
        connectionInfo.signalStrength = 0;
    }
    
    if (mqttClient.connected()) {
        connectionInfo.mqttStatus = ConnectionStatus::CONNECTED;
    } else {
        connectionInfo.mqttStatus = ConnectionStatus::DISCONNECTED;
    }
}

void AzureIoTClient::handleConnectionFailure() {
    connectionFailures++;
    if (logger) logger->warningf("Connection failure #%d", connectionFailures);
    
    if (connectionFailures >= MAX_CONNECTION_FAILURES) {
        if (logger) logger->critical("Too many connection failures, restarting system");
        ESP.restart();
    }
}

void AzureIoTClient::loop() {
    unsigned long currentTime = millis();
    
    if (currentTime - lastConnectionCheck >= CONNECTION_CHECK_INTERVAL) {
        lastConnectionCheck = currentTime;
        updateConnectionInfo();
        
        if (!isConnected()) {
            if (logger) logger->info("Connection lost, attempting to reconnect");
            connect();
        }
        
        if (sasExpiryTime > 0 && time(NULL) >= sasExpiryTime) {
            if (logger) logger->info("SAS token expired, refreshing");
            refreshAuthentication();
        }
    }
    
    if (mqttClient.connected()) {
        mqttClient.loop();
    }
}

bool AzureIoTClient::refreshAuthentication() {
    if (logger) logger->info("Refreshing SAS token");
    
    if (!generateSasToken()) {
        if (logger) logger->error("SAS token refresh failed");
        return false;
    }
    
    if (mqttClient.connected()) {
        mqttClient.disconnect();
        delay(100);
    }
    
    return connectMQTT();
}

void AzureIoTClient::setKeepAlive(int seconds) {
    mqttClient.setKeepAlive(seconds);
}

void AzureIoTClient::setOutputPower(float dbm) {
    WiFi.setOutputPower(dbm);
}

void AzureIoTClient::enableModemSleep(bool enable) {
    WiFi.setSleep(enable);
}