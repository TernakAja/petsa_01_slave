#ifndef EVENT_CARD_UI_H
#define EVENT_CARD_UI_H

#include <Arduino.h>
#include "../../include/ILogger.h"

class EventCardUI {
private:
    ILogger* logger;
    bool useUnicode;
    bool useEmoji;
    static constexpr int CARD_WIDTH = 58;
    bool cardOpen;
    
    struct UIChars {
        const char* topLeft;
        const char* topRight;
        const char* bottomLeft;
        const char* bottomRight;
        const char* horizontal;
        const char* vertical;
        const char* tee;
        const char* end;
        const char* wifiIcon;
        const char* mqttIcon;
        const char* tempIcon;
        const char* heartIcon;
        const char* okIcon;
        const char* errorIcon;
        const char* infoIcon;
    } chars;
    
    void initializeChars();
    void repeat(const char* str, int count) const;

public:
    EventCardUI(ILogger* log = nullptr, bool unicode = true, bool emoji = true);
    
    void cardHeader(const char* title, const char* prefix = "EVENT");
    void cardKV(const char* key, const char* value);
    void cardKVf(const char* key, const char* format, ...);
    void cardFooter();
    
    void pipeStart(bool spacer = true);
    void pipeStepMid(const char* icon, const char* label, const char* status, const char* extra = nullptr);
    void pipeGuide();
    void pipeStepEnd(const char* text, const char* icon = nullptr);
    
    void printSystemStatus(const char* wifiStatus, const char* mqttStatus, 
                          const char* deviceId, int rssi = 0);
    void printSensorData(float temperature, float heartRate, float spo2,
                        float ambientTemp = NAN, float objectTemp = NAN);
    void printCollectionProgress(int current, int total, unsigned long timestamp,
                               unsigned int freeHeap);
    void printTransmissionResult(const char* topic, bool success, 
                               float avgTemp, int tempCount,
                               float avgHR, int hrCount,
                               float avgSpO2, int spo2Count);
    
    void setUseUnicode(bool use);
    void setUseEmoji(bool use);
    bool getUseUnicode() const { return useUnicode; }
    bool getUseEmoji() const { return useEmoji; }
};

#endif