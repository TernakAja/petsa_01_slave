#include "EventCardUI.h"
#include <stdarg.h>

EventCardUI::EventCardUI(ILogger* log, bool unicode, bool emoji)
    : logger(log), useUnicode(unicode), useEmoji(emoji), cardOpen(false) {
    initializeChars();
}

void EventCardUI::initializeChars() {
    if (useUnicode) {
        chars.topLeft = "╭";
        chars.topRight = "╮";
        chars.bottomLeft = "╰";
        chars.bottomRight = "╯";
        chars.horizontal = "─";
        chars.vertical = "│";
        chars.tee = "├";
        chars.end = "└";
        
        if (useEmoji) {
            chars.wifiIcon = "🛰";
            chars.mqttIcon = "☁️";
            chars.tempIcon = "🌡";
            chars.heartIcon = "❤️";
        } else {
            chars.wifiIcon = "WiFi";
            chars.mqttIcon = "MQTT";
            chars.tempIcon = "Temp";
            chars.heartIcon = "HR";
        }
        
        chars.okIcon = "✔";
        chars.errorIcon = "✖";
        chars.infoIcon = "ℹ";
    } else {
        chars.topLeft = "+";
        chars.topRight = "+";
        chars.bottomLeft = "+";
        chars.bottomRight = "+";
        chars.horizontal = "-";
        chars.vertical = "|";
        chars.tee = "|";
        chars.end = "`";
        chars.wifiIcon = "WiFi";
        chars.mqttIcon = "MQTT";
        chars.tempIcon = "Temp";
        chars.heartIcon = "HR";
        chars.okIcon = "[OK]";
        chars.errorIcon = "[X]";
        chars.infoIcon = "[i]";
    }
}

void EventCardUI::repeat(const char* str, int count) const {
    for (int i = 0; i < count; i++) {
        Serial.print(str);
    }
}

void EventCardUI::cardHeader(const char* title, const char* prefix) {
    if (cardOpen) {
        Serial.print(chars.bottomLeft);
        repeat(chars.horizontal, CARD_WIDTH + 2);
        Serial.print(chars.bottomRight);
        Serial.println();
        cardOpen = false;
    }
    
    Serial.print(chars.topLeft);
    Serial.print(chars.horizontal);
    Serial.print(chars.horizontal);
    Serial.print(" ");
    Serial.print(prefix);
    Serial.print(": ");
    Serial.print(title);
    Serial.print(" ");
    
    int used = 2 + 1 + strlen(prefix) + 2 + strlen(title) + 1;
    int remaining = CARD_WIDTH + 2 - used;
    if (remaining < 0) remaining = 0;
    
    repeat(chars.horizontal, remaining);
    Serial.print(chars.topRight);
    Serial.println();
    cardOpen = true;
}

void EventCardUI::cardKV(const char* key, const char* value) {
    char keyFormatted[16];
    snprintf(keyFormatted, sizeof(keyFormatted), "%-10s", key);
    
    char valueBuffer[96];
    snprintf(valueBuffer, sizeof(valueBuffer), "%s", value);
    
    char line[160];
    snprintf(line, sizeof(line), "%s %-10s %-*s %s", 
             chars.vertical, keyFormatted, CARD_WIDTH - 1 - 10, valueBuffer, chars.vertical);
    Serial.println(line);
}

void EventCardUI::cardKVf(const char* key, const char* format, ...) {
    char buffer[120];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    cardKV(key, buffer);
}

void EventCardUI::cardFooter() {
    if (!cardOpen) return;
    
    Serial.print(chars.bottomLeft);
    repeat(chars.horizontal, CARD_WIDTH + 2);
    Serial.print(chars.bottomRight);
    Serial.println();
    cardOpen = false;
}

void EventCardUI::pipeStart(bool spacer) {
    if (spacer) Serial.println("       ");
}

void EventCardUI::pipeStepMid(const char* icon, const char* label, const char* status, const char* extra) {
    Serial.print("       ");
    Serial.print(chars.tee);
    Serial.print(chars.horizontal);
    Serial.print(chars.horizontal);
    Serial.print(" ");
    Serial.print(icon);
    Serial.print(" ");
    Serial.print(label);
    
    if (status && *status) {
        Serial.print("  ");
        Serial.print(status);
    }
    
    if (extra && *extra) {
        Serial.print("  (");
        Serial.print(extra);
        Serial.print(")");
    }
    
    Serial.println();
}

void EventCardUI::pipeGuide() {
    Serial.println("       │");
}

void EventCardUI::pipeStepEnd(const char* text, const char* icon) {
    Serial.print("       ");
    Serial.print(chars.end);
    Serial.print(chars.horizontal);
    Serial.print(chars.horizontal);
    Serial.print(" ");
    
    if (icon && *icon) {
        Serial.print(icon);
        Serial.print(" ");
    }
    
    Serial.println(text);
}

void EventCardUI::printSystemStatus(const char* wifiStatus, const char* mqttStatus, 
                                   const char* deviceId, int rssi) {
    pipeStart(false);
    
    char extraWiFi[40] = {0};
    if (rssi != 0) {
        snprintf(extraWiFi, sizeof(extraWiFi), "RSSI %d dBm", rssi);
    }
    
    pipeStepMid(chars.wifiIcon, "WiFi", wifiStatus, (rssi != 0) ? extraWiFi : nullptr);
    pipeStepMid(chars.mqttIcon, "MQTT", mqttStatus, deviceId);
}

void EventCardUI::printSensorData(float temperature, float heartRate, float spo2,
                                 float ambientTemp, float objectTemp) {
    char tempLine[64];
    if (!isnan(temperature)) {
        if (!isnan(ambientTemp) && !isnan(objectTemp)) {
            snprintf(tempLine, sizeof(tempLine), "%.2f°C (Obj:%.2f Amb:%.2f)", 
                    temperature, objectTemp, ambientTemp);
        } else {
            snprintf(tempLine, sizeof(tempLine), "%.2f°C", temperature);
        }
    } else {
        strcpy(tempLine, "N/A");
    }
    
    char hrLine[64];
    bool haveBPM = (!isnan(heartRate) && heartRate > 0);
    bool haveSpO2 = (!isnan(spo2) && spo2 >= 70.0 && spo2 <= 100.0);
    
    if (haveBPM && haveSpO2) {
        snprintf(hrLine, sizeof(hrLine), "BPM %.1f / %.0f%%", heartRate, spo2);
    } else if (haveBPM) {
        snprintf(hrLine, sizeof(hrLine), "BPM %.1f (no SpO₂)", heartRate);
    } else if (haveSpO2) {
        snprintf(hrLine, sizeof(hrLine), "SpO₂ %.0f%% (no BPM)", spo2);
    } else {
        strcpy(hrLine, "—");
    }
    
    pipeStepMid(chars.tempIcon, "Temp", tempLine);
    pipeStepMid(chars.heartIcon, "HR/SpO₂", hrLine);
}

void EventCardUI::printCollectionProgress(int current, int total, unsigned long timestamp,
                                        unsigned int freeHeap) {
    char timeStr[32];
    unsigned long seconds = timestamp / 1000;
    unsigned long hours = (seconds / 3600) % 24;
    unsigned long minutes = (seconds / 60) % 60;
    seconds = seconds % 60;
    snprintf(timeStr, sizeof(timeStr), "%02lu:%02lu:%02lu", hours, minutes, seconds);
    
    cardHeader("COLLECTION WINDOW");
    cardKVf("window", "%d/%d", current, total);
    cardKV("timestamp", timeStr);
    cardKVf("heap", "%u B", freeHeap);
    cardFooter();
}

void EventCardUI::printTransmissionResult(const char* topic, bool success,
                                        float avgTemp, int tempCount,
                                        float avgHR, int hrCount,
                                        float avgSpO2, int spo2Count) {
    cardHeader("TRANSMIT RESULT");
    cardKV("topic", topic);
    cardKVf("avg temp", "%.2f °C (%d)", avgTemp, tempCount);
    cardKVf("avg BPM", "%.2f (%d)", avgHR, hrCount);
    cardKVf("avg SpO₂", "%.1f %% (%d)", avgSpO2, spo2Count);
    cardFooter();
    
    pipeStart(false);
    pipeStepEnd(success ? "Data sent to Azure IoT Hub" : "Publish failed", 
                success ? chars.okIcon : chars.errorIcon);
}

void EventCardUI::setUseUnicode(bool use) {
    useUnicode = use;
    initializeChars();
}

void EventCardUI::setUseEmoji(bool use) {
    useEmoji = use;
    initializeChars();
}