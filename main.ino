// Sensor libraries
#include <Wire.h>
#include <Adafruit_MLX90614.h>
#ifndef I2C_BUFFER_LENGTH
#define I2C_BUFFER_LENGTH BUFFER_LENGTH
#endif
#include "MAX30105.h"
#include "heartRate.h"
#include "spo2_algorithm.h" // SpO2 algorithm library

// IoT libraries
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
#include <ArduinoJson.h>
#include <Ticker.h>
#include "iot_configs.h"

// SensorState class
class SensorState
{
private:
    float bpm = 0.0f;
    float temperature = 0.0f;
    float spo2 = 0.0f;
    bool hasNewData = false;

public:
    void setState(float newTemp, float newBpm, float newSpo2)
    {
        bool hasChanged = (bpm != newBpm) || (temperature != newTemp) || (spo2 != newSpo2);
        bpm = newBpm;
        temperature = newTemp;
        spo2 = newSpo2;
        if (hasChanged)
        {
            hasNewData = true;
        }
    }

    float getBPM() const { return bpm; }
    float getTemperature() const { return temperature; }
    float getSpO2() const { return spo2; }

    bool hasChanged()
    {
        if (hasNewData)
        {
            hasNewData = false;
            return true;
        }
        return false;
    }
};

// Sensor classes
class MLX90614Sensor
{
private:
    Adafruit_MLX90614 mlx;
    float lastValidTemp = 38.5;
    unsigned long lastReadTime = 0;
    const unsigned long READ_INTERVAL = 100;

public:
    bool begin()
    {
        Serial.println("Initializing MLX90614...");
        int retryCount = 0;
        const int maxRetries = 10;

        while (retryCount < maxRetries)
        {
            if (mlx.begin())
            {
                mlx.writeEmissivity(0.98);
                Serial.print("Emissivity = ");
                Serial.println(mlx.readEmissivity());
                Serial.println("MLX90614 Loaded!");
                return true;
            }
            Serial.print("Retry ");
            Serial.print(retryCount + 1);
            Serial.println("/10 for MLX90614...");
            delay(500);
            yield();
            retryCount++;
        }
        Serial.println("Failed to initialize MLX90614.");
        scanI2C();
        return false;
    }

    float readCoreBodyTemperature()
    {
        if (millis() - lastReadTime < READ_INTERVAL)
        {
            return lastValidTemp;
        }
        lastReadTime = millis();

        Wire.setClock(100000);
        float tEar = mlx.readObjectTempC();
        float tAmbient = mlx.readAmbientTempC();
        Wire.setClock(400000);

        if (isnan(tEar) || isnan(tAmbient) ||
            tEar < -10.0 || tEar > 60.0 ||
            tAmbient < -20.0 || tAmbient > 60.0)
        {
            return lastValidTemp;
        }

        float tCore = 0.8 * tEar + 0.1 * tAmbient + 5;
        if (tCore < 30.0 || tCore > 50.0)
        {
            return lastValidTemp;
        }

        lastValidTemp = tCore;
        return tCore;
    }

private:
    void scanI2C()
    {
        Serial.println("Scanning I2C bus...");
        int deviceCount = 0;
        for (byte address = 1; address < 127 && deviceCount < 10; address++)
        {
            Wire.beginTransmission(address);
            if (Wire.endTransmission() == 0)
            {
                Serial.print("I2C device found at address 0x");
                if (address < 16)
                    Serial.print("0");
                Serial.println(address, HEX);
                deviceCount++;
            }
            yield();
        }
        Serial.println("I2C scan complete.");
    }
};

class MAX30105Sensor
{
private:
    MAX30105 particleSensor;
    long lastBeat = 0;
    float beatsPerMinute = 0;
    float spo2Value = 98.0;
    unsigned long lastReadTime = 0;
    const unsigned long READ_INTERVAL = 200;

    // SpO2 calculation buffers
    static const uint16_t MX3_BUFFER_SIZE = 100;
    uint32_t irBuffer[MX3_BUFFER_SIZE];
    uint32_t redBuffer[MX3_BUFFER_SIZE];

public:
    bool begin()
    {
        Serial.println("Initializing MAX30105...");
        int retryCount = 0;
        const int maxRetries = 10;

        while (retryCount < maxRetries)
        {
            if (particleSensor.begin(Wire, I2C_SPEED_FAST))
            {
                // Setup for SpO2 and heart rate
                particleSensor.setup(60, 4, 2, 100, 411, 4096); // Configure for SpO2
                Serial.println("MAX30105 Loaded!");
                return true;
            }
            Serial.print("Retry ");
            Serial.print(retryCount + 1);
            Serial.println("/10 for MAX30105...");
            delay(500);
            yield();
            retryCount++;
        }
        Serial.println("Failed to initialize MAX30105.");
        scanI2C();
        return false;
    }

    float readHeartBeat()
    {
        if (millis() - lastReadTime < READ_INTERVAL)
        {
            return beatsPerMinute;
        }
        lastReadTime = millis();

        const int maxSamples = 50;
        int sampleCount = 0;
        static float prevIR = 0;
        unsigned long startTime = millis();
        const unsigned long maxReadTime = 2000;

        while (sampleCount < maxSamples && (millis() - startTime) < maxReadTime)
        {
            long irValue = particleSensor.getIR();
            if (irValue < 20000)
            {
                delay(20);
                sampleCount++;
                yield();
                continue;
            }

            float filteredIR = irValue - 0.99 * prevIR;
            prevIR = irValue;

            if (checkForBeat(filteredIR))
            {
                unsigned long now = millis();
                unsigned long delta = now - lastBeat;
                lastBeat = now;

                float newBPM = 60.0 / (delta / 1000.0);
                if (newBPM > 30 && newBPM < 100)
                {
                    beatsPerMinute = newBPM;
                    return beatsPerMinute;
                }
            }
            sampleCount++;
            delay(20);
            yield();
        }
        return beatsPerMinute;
    }

    float readSpO2()
    {
        Serial.println("Reading SpO2...");

        // Check if finger is placed on sensor
        long irValue = particleSensor.getIR();
        if (irValue < 50000)
        {
            Serial.println("No finger detected for SpO2 measurement");
            return spo2Value; // Return last valid value
        }

        // Collect samples for SpO2 calculation
        for (byte i = 0; i < MX3_BUFFER_SIZE; i++)
        {
            while (particleSensor.available() == false)
            {
                particleSensor.check();
                yield();
            }

            redBuffer[i] = particleSensor.getRed();
            irBuffer[i] = particleSensor.getIR();
            particleSensor.nextSample();

            // Show progress
            if (i % 25 == 0)
            {
                Serial.print("SpO2 sampling: ");
                Serial.print((i * 100) / MX3_BUFFER_SIZE);
                Serial.println("%");
            }

            delay(10);
            yield();
        }

        // Calculate heart rate and SpO2
        int32_t spo2;
        int8_t validSPO2;
        int32_t heartRate;
        int8_t validHeartRate;

        // Use SpO2 algorithm
        maxim_heart_rate_and_oxygen_saturation(irBuffer, MX3_BUFFER_SIZE, redBuffer,
                                               &spo2, &validSPO2, &heartRate, &validHeartRate);

        if (validSPO2 && spo2 >= 70 && spo2 <= 100)
        {
            spo2Value = (float)spo2;
            Serial.print("Valid SpO2 calculated: ");
            Serial.println(spo2Value);
        }
        else
        {
            Serial.println("Invalid SpO2 calculation, using previous value");
        }

        if (validHeartRate && heartRate > 30 && heartRate < 200)
        {
            beatsPerMinute = (float)heartRate;
            Serial.print("SpO2 algorithm heart rate: ");
            Serial.println(beatsPerMinute);
        }

        return spo2Value;
    }

private:
    void scanI2C()
    {
        Serial.println("Scanning I2C bus...");
        int deviceCount = 0;
        for (byte address = 1; address < 127 && deviceCount < 10; address++)
        {
            Wire.beginTransmission(address);
            if (Wire.endTransmission() == 0)
            {
                Serial.print("I2C device found at address 0x");
                if (address < 16)
                    Serial.print("0");
                Serial.println(address, HEX);
                deviceCount++;
            }
            yield();
        }
        Serial.println("I2C scan complete.");
    }
};

class Sensor
{
private:
    MLX90614Sensor mlx;
    MAX30105Sensor max;

public:
    bool begin()
    {
        bool mlxSuccess = mlx.begin();
        delay(1000);
        yield();
        bool maxSuccess = max.begin();
        return mlxSuccess && maxSuccess;
    }

    float readTemperature()
    {
        float temp = mlx.readCoreBodyTemperature();
        if (temp > 100.0 || temp < 30.0)
        {
            return 38.5;
        }
        return temp;
    }

    float readHeartBeat()
    {
        return max.readHeartBeat();
    }

    float readSpO2()
    {
        return max.readSpO2();
    }
};

// Constants
#define AZURE_SDK_CLIENT_USER_AGENT "c%2F" AZ_SDK_VERSION_STRING "(ard;esp8266)"
#define LED_PIN 2
#define MQTT_PACKET_SIZE 512
#define ONE_HOUR_IN_SECS 3600
#define NTP_SERVERS "pool.ntp.org", "time.nist.gov"
#define DATA_POINTS 30
#define COLLECTION_INTERVAL_MS 2000
#define SEND_INTERVAL_MS 60000
#define SPO2_INTERVAL_MS 10000 // Read SpO2 every 10 seconds (less frequent due to processing time)

// IoT Hub configuration
static const char *ssid = IOT_CONFIG_WIFI_SSID;
static const char *password = IOT_CONFIG_WIFI_PASSWORD;
static const char *host = IOT_CONFIG_IOTHUB_FQDN;
static const char *device_id = IOT_CONFIG_DEVICE_ID;
static const char *device_key = IOT_CONFIG_DEVICE_KEY;
static const int port = 8883;

// IoT clients
static WiFiClientSecure wifi_client;
static X509List cert((const char *)ca_pem);
static PubSubClient mqtt_client(wifi_client);
static az_iot_hub_client client;
static char sas_token[300];
static uint8_t signature[512];
static unsigned char encrypted_signature[32];
static char base64_decoded_device_key[32];

// Sensor and state objects
Sensor sensor;
SensorState sensorState;

// Data collection arrays
float tempDats[DATA_POINTS];
float bpmDats[DATA_POINTS];
float spo2Dats[DATA_POINTS];
int currentDat = 0;

// Timing variables
unsigned long lastSendTime = 0;
unsigned long lastCollectionTime = 0;
unsigned long lastSpO2Time = 0;
unsigned long lastConnectionCheck = 0;
const unsigned long CONNECTION_CHECK_INTERVAL = 30000;

// Connection status tracking
bool mqttConnected = false;
int connectionFailures = 0;
const int MAX_CONNECTION_FAILURES = 5;

// WiFi and time setup
void connectToWiFi()
{
    Serial.print("Connecting to WIFI SSID ");
    Serial.println(ssid);
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);
    WiFi.begin(ssid, password);

    int attempts = 0;
    const int maxAttempts = 60;

    while (WiFi.status() != WL_CONNECTED && attempts < maxAttempts)
    {
        delay(500);
        Serial.print(".");
        yield();
        attempts++;
    }

    if (WiFi.status() == WL_CONNECTED)
    {
        Serial.print("\nWiFi connected, IP address: ");
        Serial.println(WiFi.localIP());
        Serial.print("Signal strength: ");
        Serial.print(WiFi.RSSI());
        Serial.println(" dBm");
    }
    else
    {
        Serial.println("\nWiFi connection failed!");
    }
}

void initializeTime()
{
    Serial.print("Setting time using SNTP");
    configTime(0, 0, NTP_SERVERS);
    time_t now = time(NULL);
    int attempts = 0;
    const int maxAttempts = 30;

    while (now < 1510592825 && attempts < maxAttempts)
    {
        delay(500);
        Serial.print(".");
        yield();
        now = time(NULL);
        attempts++;
    }

    if (now >= 1510592825)
    {
        Serial.println(" done!");
        Serial.print("Current time: ");
        Serial.println(ctime(&now));
    }
    else
    {
        Serial.println(" failed!");
    }
}

// Azure IoT Hub connection
void initializeClients()
{
    az_iot_hub_client_options options = az_iot_hub_client_options_default();
    options.user_agent = AZ_SPAN_FROM_STR(AZURE_SDK_CLIENT_USER_AGENT);

    wifi_client.setTrustAnchors(&cert);

    if (az_result_failed(az_iot_hub_client_init(&client,
                                                az_span_create((uint8_t *)host, strlen(host)),
                                                az_span_create((uint8_t *)device_id, strlen(device_id)),
                                                &options)))
    {
        Serial.println("Failed initializing Azure IoT Hub client");
        return;
    }

    mqtt_client.setServer(host, port);
    mqtt_client.setBufferSize(MQTT_PACKET_SIZE);
    mqtt_client.setKeepAlive(60);
}

int generateSasToken(char *sas_token, size_t size)
{
    az_span signature_span = az_span_create((uint8_t *)signature, sizeof(signature));
    az_span out_signature_span;
    az_span encrypted_signature_span = az_span_create((uint8_t *)encrypted_signature, sizeof(encrypted_signature));
    uint32_t expiration = time(NULL) + ONE_HOUR_IN_SECS;

    if (expiration < 1510592825)
    {
        Serial.println("Invalid time for SAS token generation");
        return 1;
    }

    if (az_result_failed(az_iot_hub_client_sas_get_signature(&client, expiration, signature_span, &out_signature_span)))
    {
        Serial.println("Failed getting SAS signature");
        return 1;
    }

    int base64_decoded_device_key_length = base64_decode_chars(device_key, strlen(device_key), base64_decoded_device_key);
    if (base64_decoded_device_key_length == 0)
    {
        Serial.println("Failed base64 decoding device key");
        return 1;
    }

    br_hmac_key_context kc;
    br_hmac_key_init(&kc, &br_sha256_vtable, base64_decoded_device_key, base64_decoded_device_key_length);
    br_hmac_context hmac_ctx;
    br_hmac_init(&hmac_ctx, &kc, 32);
    br_hmac_update(&hmac_ctx, az_span_ptr(out_signature_span), az_span_size(out_signature_span));
    br_hmac_out(&hmac_ctx, encrypted_signature);

    String b64enc_hmacsha256_signature = base64::encode(encrypted_signature, br_hmac_size(&hmac_ctx));
    az_span b64enc_hmacsha256_signature_span = az_span_create((uint8_t *)b64enc_hmacsha256_signature.c_str(), b64enc_hmacsha256_signature.length());

    if (az_result_failed(az_iot_hub_client_sas_get_password(&client, expiration, b64enc_hmacsha256_signature_span, AZ_SPAN_EMPTY, sas_token, size, NULL)))
    {
        Serial.println("Failed getting SAS token");
        return 1;
    }

    Serial.println("SAS token generated successfully");
    return 0;
}

int connectToAzureIoTHub()
{
    if (WiFi.status() != WL_CONNECTED)
    {
        Serial.println("WiFi not connected. Cannot connect to Azure IoT Hub.");
        return 1;
    }

    char mqtt_client_id[128];
    size_t client_id_length;
    if (az_result_failed(az_iot_hub_client_get_client_id(&client, mqtt_client_id, sizeof(mqtt_client_id) - 1, &client_id_length)))
    {
        Serial.println("Failed getting client id");
        return 1;
    }
    mqtt_client_id[client_id_length] = '\0';

    char mqtt_username[256];
    if (az_result_failed(az_iot_hub_client_get_user_name(&client, mqtt_username, sizeof(mqtt_username), NULL)))
    {
        Serial.println("Failed to get MQTT username");
        return 1;
    }

    Serial.print("MQTT Client ID: ");
    Serial.println(mqtt_client_id);
    Serial.print("MQTT Username: ");
    Serial.println(mqtt_username);

    int connectionAttempts = 0;
    const int maxConnectionAttempts = 3;

    while (!mqtt_client.connected() && connectionAttempts < maxConnectionAttempts)
    {
        Serial.print("MQTT connecting ... ");

        if (mqtt_client.connect(mqtt_client_id, mqtt_username, sas_token, NULL, 0, false, NULL, true))
        {
            Serial.println("connected.");
            mqttConnected = true;
            connectionFailures = 0;
            break;
        }
        else
        {
            Serial.print("failed, status code = ");
            Serial.print(mqtt_client.state());
            Serial.print(" (");

            switch (mqtt_client.state())
            {
            case -4:
                Serial.print("MQTT_CONNECTION_TIMEOUT");
                break;
            case -3:
                Serial.print("MQTT_CONNECTION_LOST");
                break;
            case -2:
                Serial.print("MQTT_CONNECT_FAILED");
                break;
            case -1:
                Serial.print("MQTT_DISCONNECTED");
                break;
            case 1:
                Serial.print("MQTT_CONNECT_BAD_PROTOCOL");
                break;
            case 2:
                Serial.print("MQTT_CONNECT_BAD_CLIENT_ID");
                break;
            case 3:
                Serial.print("MQTT_CONNECT_UNAVAILABLE");
                break;
            case 4:
                Serial.print("MQTT_CONNECT_BAD_CREDENTIALS");
                break;
            case 5:
                Serial.print("MQTT_CONNECT_UNAUTHORIZED");
                break;
            default:
                Serial.print("UNKNOWN_ERROR");
                break;
            }
            Serial.println(")");

            Serial.println(". Trying again in 5 seconds.");
            delay(5000);
            yield();
            connectionAttempts++;
        }
    }

    if (mqtt_client.connected())
    {
        String c2d_topic = "devices/" + String(device_id) + "/messages/devicebound/#";
        String methods_topic = "$iothub/methods/POST/#";

        if (mqtt_client.subscribe(c2d_topic.c_str()))
        {
            Serial.println("Subscribed to C2D topic.");
        }
        if (mqtt_client.subscribe(methods_topic.c_str()))
        {
            Serial.println("Subscribed to direct method topic.");
        }
        return 0;
    }
    else
    {
        mqttConnected = false;
        connectionFailures++;
    }

    return 1;
}

void establishConnection()
{
    connectToWiFi();
    if (WiFi.status() != WL_CONNECTED)
    {
        Serial.println("WiFi connection failed. Restarting...");
        ESP.restart();
    }

    initializeTime();
    initializeClients();

    if (generateSasToken(sas_token, sizeof(sas_token)) == 0)
    {
        if (connectToAzureIoTHub() != 0)
        {
            Serial.println("Failed to connect to Azure IoT Hub");
        }
    }
    else
    {
        Serial.println("Failed generating MQTT password");
    }
}

void checkConnections()
{
    unsigned long currentTime = millis();

    if (currentTime - lastConnectionCheck >= CONNECTION_CHECK_INTERVAL)
    {
        lastConnectionCheck = currentTime;

        if (WiFi.status() != WL_CONNECTED)
        {
            Serial.println("WiFi disconnected. Reconnecting...");
            connectToWiFi();
            if (WiFi.status() != WL_CONNECTED)
            {
                Serial.println("WiFi reconnection failed.");
                return;
            }
        }

        if (!mqtt_client.connected())
        {
            Serial.println("MQTT disconnected. Reconnecting...");
            if (generateSasToken(sas_token, sizeof(sas_token)) == 0)
            {
                connectToAzureIoTHub();
            }
        }

        if (connectionFailures >= MAX_CONNECTION_FAILURES)
        {
            Serial.println("Too many connection failures. Restarting...");
            ESP.restart();
        }
    }
}

void collectData()
{
    if (currentDat < DATA_POINTS)
    {
        float temp = sensorState.getTemperature();
        float bpm = sensorState.getBPM();
        float spo2 = sensorState.getSpO2();

        tempDats[currentDat] = temp;
        bpmDats[currentDat] = bpm;
        spo2Dats[currentDat] = spo2;
        currentDat++;

        Serial.println("PETSA | ========================");
        Serial.printf("Temperature = %.2f°C\n", temp);
        if (bpm > 0)
        {
            Serial.printf("BPM = %.2f\n", bpm);
        }
        else
        {
            Serial.println("BPM = No reading");
        }
        Serial.printf("SpO2 = %.1f%%\n", spo2);
        Serial.printf("Free heap: %u bytes\n", ESP.getFreeHeap());
        Serial.printf("Data point %d/%d collected\n", currentDat, DATA_POINTS);
        Serial.printf("WiFi Status: %s, MQTT Status: %s\n",
                      WiFi.status() == WL_CONNECTED ? "Connected" : "Disconnected",
                      mqtt_client.connected() ? "Connected" : "Disconnected");
    }
}

void sendAveragedData()
{
    if (currentDat == 0)
        return;

    float avgTemp = 0, avgBPM = 0, avgSpO2 = 0;
    int validTempCount = 0, validBPMCount = 0, validSpO2Count = 0;

    for (int i = 0; i < currentDat; i++)
    {
        if (tempDats[i] >= 30.0 && tempDats[i] <= 50.0)
        {
            avgTemp += tempDats[i];
            validTempCount++;
        }
        if (bpmDats[i] > 0)
        {
            avgBPM += bpmDats[i];
            validBPMCount++;
        }
        if (spo2Dats[i] >= 70.0 && spo2Dats[i] <= 100.0)
        {
            avgSpO2 += spo2Dats[i];
            validSpO2Count++;
        }
    }

    avgTemp = validTempCount > 0 ? avgTemp / validTempCount : 38.5;
    avgBPM = validBPMCount > 0 ? avgBPM / validBPMCount : 0;
    avgSpO2 = validSpO2Count > 0 ? avgSpO2 / validSpO2Count : 98.0;
    currentDat = 0;

    if (!mqtt_client.connected())
    {
        Serial.println("MQTT not connected. Skipping send.");
        return;
    }

    DynamicJsonDocument doc(256);
    doc["deviceId"] = device_id;
    doc["pulseRate"] = avgBPM;
    doc["temperature"] = avgTemp;
    doc["sp02"] = avgSpO2;

    char timestamp[32];
    time_t now = time(NULL);
    strftime(timestamp, sizeof(timestamp), "%Y-%m-%dT%H:%M:%SZ", gmtime(&now));
    doc["timestamp"] = timestamp;

    String payload;
    serializeJson(doc, payload);
    String topic = "devices/" + String(device_id) + "/messages/events/";

    Serial.print("Sending payload to topic: ");
    Serial.println(topic);
    Serial.print("Payload size: ");
    Serial.println(payload.length());

    if (mqtt_client.publish(topic.c_str(), payload.c_str()))
    {
        Serial.println("Payload sent successfully:");
        Serial.println(payload);
        if (validTempCount > 0)
        {
            Serial.printf("\t[1 min] Average Temperature = %.2f°C\n", avgTemp);
        }
        else
        {
            Serial.println("\t[1 min] No valid temperature readings");
        }
        if (validBPMCount > 0)
        {
            Serial.printf("\t[1 min] Average BPM = %.2f\n", avgBPM);
        }
        else
        {
            Serial.println("\t[1 min] No valid BPM readings");
        }
        if (validSpO2Count > 0)
        {
            Serial.printf("\t[1 min] Average SpO2 = %.1f%%\n", avgSpO2);
        }
        else
        {
            Serial.println("\t[1 min] No valid SpO2 readings");
        }
        connectionFailures = 0;
    }
    else
    {
        Serial.println("Failed to publish message.");
        Serial.print("MQTT state: ");
        Serial.println(mqtt_client.state());
        connectionFailures++;
    }
}

void setup()
{
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);
    Serial.begin(115200);
    while (!Serial)
    {
        yield();
        delay(10);
    }

    Wire.begin();
    Wire.setClock(400000);

    Serial.printf("Initial free heap: %u bytes\n", ESP.getFreeHeap());
    Serial.print("ESP8266 Chip ID: ");
    Serial.println(ESP.getChipId());
    Serial.print("Flash Chip ID: ");
    Serial.println(ESP.getFlashChipId());

    establishConnection();
    if (!sensor.begin())
    {
        Serial.println("Sensor initialization failed. Continuing with limited functionality.");
    }

    lastSendTime = millis();
    lastCollectionTime = millis();
    lastSpO2Time = millis();
    lastConnectionCheck = millis();

    Serial.println("Setup completed. Starting main loop...");
}

void loop()
{
    unsigned long currentTime = millis();

    // Read SpO2 less frequently due to processing time
    if (currentTime - lastSpO2Time >= SPO2_INTERVAL_MS)
    {
        float spo2 = sensor.readSpO2();
        sensorState.setState(sensor.readTemperature(), sensor.readHeartBeat(), spo2);
        lastSpO2Time = currentTime;
    }
    else
    {
        // Update other sensors more frequently
        sensorState.setState(sensor.readTemperature(), sensor.readHeartBeat(), sensorState.getSpO2());
    }

    if (currentTime - lastCollectionTime >= COLLECTION_INTERVAL_MS)
    {
        collectData();
        lastCollectionTime = currentTime;
    }

    if (currentTime - lastSendTime >= SEND_INTERVAL_MS)
    {
        sendAveragedData();
        lastSendTime = currentTime;
    }

    checkConnections();

    if (mqtt_client.connected())
    {
        mqtt_client.loop();
    }

    delay(100);
    yield();
}
