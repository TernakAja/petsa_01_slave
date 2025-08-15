/***************************************************
  ESP8266 + MLX90614 + MAX30105 + Azure IoT Hub
  - Event Card + Line Connectors UI (flow diagram style)
  - I2C pins (D2/D1), 100 kHz, long clock-stretch, bus recovery
  - MLX double-read validation (no absurd temps in UI)
  - Contact-aware temp model + robust filtering
  - MAX30105 duty-cycled (low LED current, sleep when idle)
  - PERF mode (default here): WiFi stays up; ECO/ULTRA_ECO optional
  - Azure IoT Hub with SAS refresh
  - JSON via snprintf (heap-friendly)
****************************************************/

// ---------------- Sensor libraries ----------------
#include <Wire.h>
#include <Adafruit_MLX90614.h>
#ifndef I2C_BUFFER_LENGTH
#define I2C_BUFFER_LENGTH BUFFER_LENGTH
#endif
#include "MAX30105.h"
#include "heartRate.h"
#include "spo2_algorithm.h"

// ---------------- IoT libraries ----------------
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
#include "iot_configs.h"   // <- your secrets/configs

// ---------------- C std libs ----------------
#include <math.h>
#include <time.h>
#include <stdarg.h>

// ===================== Power Modes =====================
// Choose ONE: PERF (always connected) or ECO/ULTRA_ECO (radio sleeps)
#define MODE_PERF
// #define MODE_ECO
// #define MODE_ULTRA_ECO  // needs D0->RST jumper for deep sleep wake

// ---------------- Board/I2C pins (NodeMCU/D1 mini) ----------------
#ifndef SDA_PIN
  #define SDA_PIN D2  // GPIO4
#endif
#ifndef SCL_PIN
  #define SCL_PIN D1  // GPIO5
#endif

// ---------------- I2C speed & timing ----------------
static const uint32_t I2C_SPEED_SMBUS = 100000; // 100 kHz
static const uint32_t I2C_STRETCH_US  = 250000; // 250 ms stretch (MLX-safe)

// ---------------- LED feedback ----------------
#define LED_PIN 2 // D4 (inverted on ESP8266)
enum LedMode { LED_OFF, LED_FAST, LED_SLOW, LED_ERROR };
static LedMode ledMode = LED_OFF;
static unsigned long lastLedToggle = 0;
static bool ledState = false;
void setLed(LedMode m) { ledMode = m; }
void serviceLed() {
  unsigned long now = millis();
  unsigned long period = (ledMode==LED_FAST?200: ledMode==LED_SLOW?800: ledMode==LED_ERROR?100:0);
  if (!period) { digitalWrite(LED_PIN, HIGH); return; }
  if (now - lastLedToggle >= period) {
    lastLedToggle = now;
    ledState = !ledState;
    digitalWrite(LED_PIN, ledState ? LOW : HIGH); // inverted
  }
}

// ───────────────────────── Event Card + Connectors UI ─────────────────────────
#ifndef EC_USE_UNICODE
  #define EC_USE_UNICODE 1   // set 0 for ASCII borders/icons
#endif
#ifndef EC_USE_EMOJI
  #define EC_USE_EMOJI   1   // set 0 to disable emoji
#endif

namespace ecui {
  #if EC_USE_UNICODE
    const char* TL="╭"; const char* TR="╮";
    const char* BL="╰"; const char* BR="╯";
    const char* H="─";  const char* V="│";
    const char* TEE="├";const char* END="└";
    const char* I_WIFI = EC_USE_EMOJI ? "🛰" : "WiFi";
    const char* I_MQTT = EC_USE_EMOJI ? "☁️" : "MQTT";
    const char* I_TEMP = EC_USE_EMOJI ? "🌡" : "Temp";
    const char* I_HEART= EC_USE_EMOJI ? "❤️" : "HR";
    const char* I_OK   = "✔";
    const char* I_ERR  = "✖";
    const char* I_INFO = "ℹ";
  #else
    const char* TL="+"; const char* TR="+";
    const char* BL="+"; const char* BR="+";
    const char* H="-";  const char* V="|";
    const char* TEE="|";const char* END="`";
    const char* I_WIFI="WiFi"; const char* I_MQTT="MQTT";
    const char* I_TEMP="Temp"; const char* I_HEART="HR";
    const char* I_OK="[OK]";  const char* I_ERR="[X]";
    const char* I_INFO="[i]";
  #endif

  static const int CARD_WIDTH = 58; // inside width
  static bool cardOpen = false;

  void rep(const char* s, int n){ for(int i=0;i<n;i++) Serial.print(s); }

  void cardHeader(const char* title, const char* prefix="EVENT") {
    if (cardOpen) { Serial.print(BL); rep(H, CARD_WIDTH+2); Serial.println(BR); cardOpen=false; }
    Serial.print(TL); Serial.print(H); Serial.print(H);
    Serial.print(" "); Serial.print(prefix); Serial.print(": ");
    Serial.print(title); Serial.print(" ");
    int used = 2 + 1 + (int)strlen(prefix) + 2 + (int)strlen(title) + 1;
    int rem = CARD_WIDTH + 2 - used; if (rem < 0) rem = 0;
    rep(H, rem); Serial.print(TR); Serial.println(); cardOpen=true;
  }

  void cardKV(const char* key, const char* value) {
    char k[14]; snprintf(k, sizeof(k), "%-10s", key);
    char vbuf[96]; snprintf(vbuf, sizeof(vbuf), "%s", value);
    char line[160];
    snprintf(line, sizeof(line), "%s %-10s %-*s %s", V, k, CARD_WIDTH - 1 - 10, vbuf, V);
    Serial.println(line);
  }

  void cardKVf(const char* key, const char* fmt, ...) {
    char buf[120]; va_list args; va_start(args, fmt); vsnprintf(buf, sizeof(buf), fmt, args); va_end(args);
    cardKV(key, buf);
  }

  void cardFooter() {
    if (!cardOpen) return;
    Serial.print(BL); rep(H, CARD_WIDTH+2); Serial.println(BR); cardOpen=false;
  }

  void pipeStart(bool spacer=true) { if (spacer) Serial.println("       "); }
  void pipeStepMid(const char* icon, const char* label, const char* status, const char* extra=nullptr) {
    Serial.print("       "); Serial.print(TEE); Serial.print(H); Serial.print(H); Serial.print(" ");
    Serial.print(icon); Serial.print(" "); Serial.print(label);
    if (status && *status) { Serial.print("  "); Serial.print(status); }
    if (extra && *extra)   { Serial.print("  ("); Serial.print(extra); Serial.print(")"); }
    Serial.println();
  }
  void pipeGuide() { Serial.println("       │"); }
  void pipeStepEnd(const char* text, const char* icon = I_OK) {
    Serial.print("       "); Serial.print(END); Serial.print(H); Serial.print(H); Serial.print(" ");
    if (icon && *icon) { Serial.print(icon); Serial.print(" "); }
    Serial.println(text);
  }
} // namespace ecui

// Safe temperature formatter for UI
const char* fmtTemp(float c, char* out, size_t n) {
  if (isnan(c) || c < -40.0f || c > 125.0f) snprintf(out, n, "(invalid)");
  else snprintf(out, n, "%.2f °C", c);
  return out;
}

// ---- Time sync flag & helpers ----
static bool g_timeSynced = false;
void formatTimestamp(char* out, size_t n) {
  time_t now = time(NULL);
  if (now >= 1510592825) {
    g_timeSynced = true;
    strftime(out, n, "%Y-%m-%d %H:%M:%S", localtime(&now));
  } else {
    g_timeSynced = false;
    unsigned long s = millis() / 1000UL;
    unsigned long hh = (s / 3600UL) % 24UL;
    unsigned long mm = (s / 60UL) % 60UL;
    unsigned long ss = s % 60UL;
    snprintf(out, n, "%02lu:%02lu:%02lu (uptime)", hh, mm, ss);
  }
}

// ---------------- I2C Bus tools (health + scan + recover) ----------------
bool i2cBusRecover() {
  pinMode(SDA_PIN, INPUT_PULLUP); pinMode(SCL_PIN, INPUT_PULLUP); delay(2);
  if (digitalRead(SDA_PIN)==HIGH && digitalRead(SCL_PIN)==HIGH) return true;
  if (digitalRead(SCL_PIN)==LOW) return false; // hard short

  pinMode(SCL_PIN, OUTPUT);
  for (uint8_t i=0; i<16 && digitalRead(SDA_PIN)==LOW; i++) {
    digitalWrite(SCL_PIN, LOW); delayMicroseconds(10);
    digitalWrite(SCL_PIN, HIGH); delayMicroseconds(10); yield();
  }
  pinMode(SDA_PIN, OUTPUT);
  digitalWrite(SDA_PIN, LOW);  delayMicroseconds(10);
  digitalWrite(SCL_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(SDA_PIN, HIGH); delayMicroseconds(10);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(I2C_SPEED_SMBUS);
  Wire.setClockStretchLimit(I2C_STRETCH_US);

  pinMode(SDA_PIN, INPUT_PULLUP); pinMode(SCL_PIN, INPUT_PULLUP);
  return (digitalRead(SDA_PIN)==HIGH && digitalRead(SCL_PIN)==HIGH);
}

void scanI2COnce(const char* tag = "I2C") {
  pinMode(SDA_PIN, INPUT_PULLUP); pinMode(SCL_PIN, INPUT_PULLUP);
  int sda = digitalRead(SDA_PIN), scl = digitalRead(SCL_PIN);
  ecui::cardHeader(tag, "SCAN");
  ecui::cardKVf("lines", "SDA=%d SCL=%d", sda, scl);
  int found=0;
  for (byte address=1; address<127 && found<24; address++) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission()==0) { char v[16]; snprintf(v,sizeof(v),"0x%02X", address); ecui::cardKV("device", v); found++; }
    delay(2); yield();
  }
  ecui::cardKVf("found", "%d", found);
  ecui::cardFooter();
  if (found==0 && (sda==0 || scl==0)) { ecui::pipeStart(false); ecui::pipeStepEnd("I2C line stuck low. Check power/pull-ups/wiring.", ecui::I_ERR); }
}

// ---------------- SensorState class ----------------
class SensorState {
private:
  float bpm = NAN, temperature = NAN, spo2 = NAN; bool hasNewData = false;
public:
  void setState(float t, float b, float s) { bool changed=(bpm!=b)||(temperature!=t)||(spo2!=s); bpm=b; temperature=t; spo2=s; if (changed) hasNewData=true; }
  float getBPM() const { return bpm; }
  float getTemperature() const { return temperature; }
  float getSpO2() const { return spo2; }
  bool hasChanged() { if (hasNewData){hasNewData=false; return true;} return false; }
};

// ---------------- Robust sliding filter ----------------
template<int N>
class RobustTempFilter {
public:
  RobustTempFilter(): _count(0), _idx(0), _filled(false) {}
  void clear(){_count=0; _idx=0; _filled=false;}
  void push(float x){_buf[_idx]=x; _idx=(_idx+1)%N; if(_count<N)_count++; if(_count==N)_filled=true;}
  int size() const { return _count; }
  float median() const {
    if (_count==0) return NAN; float tmp[N]; int n=_count;
    for (int i=0;i<n;i++) tmp[i]=_buf[i];
    for (int i=1;i<n;i++){ float key=tmp[i]; int j=i-1; while(j>=0 && tmp[j]>key){tmp[j+1]=tmp[j]; j--;} tmp[j+1]=key; }
    return (n&1)? tmp[n/2] : 0.5f*(tmp[n/2-1]+tmp[n/2]);
  }
  float mad(float med) const {
    if (_count==0) return NAN; float tmp[N]; int n=_count;
    for (int i=0;i<n;i++) tmp[i]=fabsf(_buf[i]-med);
    for (int i=1;i<n;i++){ float key=tmp[i]; int j=i-1; while(j>=0 && tmp[j]>key){tmp[j+1]=tmp[j]; j--;} tmp[j+1]=key; }
    return (n&1)? tmp[n/2] : 0.5f*(tmp[n/2-1]+tmp[n/2]);
  }
  float clippedMean(float k) const {
    if (_count==0) return NAN; float med=median(); float MAD=mad(med); float sigma=1.4826f*MAD;
    if (!(sigma>0)) return med; float lo=med-k*sigma, hi=med+k*sigma; float sum=0; int cnt=0;
    for (int i=0;i<_count;i++){ float v=_buf[i]; if(v>=lo && v<=hi){sum+=v; cnt++;}}
    return (cnt>0)? (sum/cnt) : med;
  }
private:
  float _buf[N]; int _count,_idx; bool _filled;
};

// ---------------- MLX90614 sensor (robust read + contact-aware model) ----------------
class MLX90614Sensor {
private:
  Adafruit_MLX90614 mlx;
  float lastStableTemp = NAN, lastSeenTemp = NAN;
  unsigned long lastReadTime = 0;
  float lastObjC_display = NAN, lastAmbC_display = NAN;

  static const unsigned long READ_INTERVAL_MS = 200; // reduce bus pressure
  static const int WSIZE = 15;
  RobustTempFilter<WSIZE> filt;
  bool settling = false; unsigned long settleStartMs = 0;

  const float JUMP_THRESH_C = 0.8f, MAD_THRESH_C = 0.15f, CLIP_K = 2.0f;
  const unsigned long SETTLE_MIN_MS = 700, SETTLE_MAX_MS = 4000;
  const float CONTACT_DELTA_C = 0.3f, CONTACT_BIAS_C = 3.0f;

public:
  bool begin() {
    ecui::cardHeader("INIT: MLX90614", "EVENT");
    ecui::cardKV("bus", "I2C 100kHz");
    ecui::cardFooter();

    Wire.setClock(I2C_SPEED_SMBUS);
    Wire.setClockStretchLimit(I2C_STRETCH_US);

    bool ok=false;
    for (int retry=0; retry<10 && !ok; retry++) {
      ecui::pipeStart(false);
      char lab[24]; snprintf(lab, sizeof(lab), "attempt %d", retry+1);
      ecui::pipeStepMid(ecui::I_TEMP, "MLX", lab);
      if (mlx.begin()) {
        float e = mlx.readEmissivity();
        char eb[24];
        if (!isnan(e)) { snprintf(eb, sizeof(eb), "emissivity %.2f", e); ecui::pipeStepMid(ecui::I_INFO, "MLX", eb); }
        ok=true; break;
      }
      delay(250); yield();
    }
    ecui::pipeStepEnd(ok ? "MLX90614 Loaded" : "MLX init failed", ok?ecui::I_OK:ecui::I_ERR);
    if (!ok) scanI2COnce("MLX90614");
    return ok;
  }

  float readCoreBodyTemperature() {
    if (millis() - lastReadTime < READ_INTERVAL_MS) return lastStableTemp;
    lastReadTime = millis();

    const int MAX_ATTEMPTS = 3;
    float tEar = NAN, tAmbient = NAN; bool ok=false;
    for (int attempt=0; attempt<MAX_ATTEMPTS && !ok; ++attempt) {
      float o1 = mlx.readObjectTempC();
      delayMicroseconds(500);
      float o2 = mlx.readObjectTempC();
      float a1 = mlx.readAmbientTempC();
      bool rOk = !(isnan(o1)||isnan(o2)||isnan(a1)) &&
                 (a1 > -40.0f && a1 < 125.0f) &&
                 (o1 > -70.0f && o1 < 380.0f) &&
                 (o2 > -70.0f && o2 < 380.0f) &&
                 (fabsf(o1 - o2) <= 3.0f);
      if (rOk) { tEar=o1; tAmbient=a1; ok=true; } else { delay(2); yield(); }
    }
    if (!ok) return lastStableTemp;

    lastObjC_display = (tEar < -70.0f || tEar > 380.0f) ? NAN : tEar;
    lastAmbC_display = (tAmbient < -40.0f || tAmbient > 125.0f) ? NAN : tAmbient;

    if (tAmbient < -20.0f || tAmbient > 60.0f) return lastStableTemp;

    bool inContact = ((tEar - tAmbient) >= CONTACT_DELTA_C);
    float tCoreRaw = inContact ? (0.8f*tEar + 0.1f*tAmbient + CONTACT_BIAS_C) : tEar;
    if (tCoreRaw < 10.0f || tCoreRaw > 60.0f) return lastStableTemp;

    if (isnan(lastStableTemp)) {
      lastSeenTemp = tCoreRaw; filt.clear(); filt.push(tCoreRaw);
      lastStableTemp = tCoreRaw; settling=false; settleStartMs=millis();
      return lastStableTemp;
    }

    filt.push(tCoreRaw);

    if (!inContact) {
      lastStableTemp = filt.clippedMean(CLIP_K);
      settling=false; lastSeenTemp=tCoreRaw; return lastStableTemp;
    }

    if (isnan(lastSeenTemp) || fabsf(tCoreRaw-lastSeenTemp) > JUMP_THRESH_C) {
      settling = true; settleStartMs=millis(); filt.clear(); filt.push(tCoreRaw);
    }
    lastSeenTemp = tCoreRaw;

    bool ready=false;
    if (filt.size() >= 5) {
      float med=filt.median(), MAD=filt.mad(med);
      if ((MAD < MAD_THRESH_C && (millis()-settleStartMs)>=SETTLE_MIN_MS) ||
          (settling && (millis()-settleStartMs)>=SETTLE_MAX_MS)) ready=true;
    }

    if (settling) { if (ready){ lastStableTemp=filt.clippedMean(CLIP_K); settling=false; } }
    else          { lastStableTemp=filt.clippedMean(CLIP_K); }

    return lastStableTemp;
  }

  float getLastObjC() const { return lastObjC_display; }
  float getLastAmbC() const { return lastAmbC_display; }
};

// ---------------- MAX30105 sensor (duty-cycled + styled logs) ----------------
class MAX30105Sensor {
private:
  MAX30105 particleSensor;
  long lastBeat = 0;
  float beatsPerMinute = 0;
  float spo2Value = 98.0;
  unsigned long lastReadTime = 0;
  static const unsigned long READ_INTERVAL = 200;

  static const uint16_t MX3_BUFFER_SIZE = 100;
  uint32_t irBuffer[MX3_BUFFER_SIZE];
  uint32_t redBuffer[MX3_BUFFER_SIZE];

  bool isAwake = true;
  float bpmBuf[4] = {0,0,0,0};
  uint8_t bpmIdx = 0;

public:
  bool begin() {
    ecui::cardHeader("INIT: MAX30105", "EVENT");
    ecui::cardKV("led", "IR/RED=0x1F");
    ecui::cardFooter();

    Wire.setClock(I2C_SPEED_SMBUS);
    Wire.setClockStretchLimit(I2C_STRETCH_US);

    bool ok=false;
    for (int retry=0; retry<10 && !ok; retry++) {
      ecui::pipeStart(false);
      char lab[24]; snprintf(lab, sizeof(lab), "attempt %d", retry+1);
      ecui::pipeStepMid(ecui::I_HEART, "MAX", lab);
      if (particleSensor.begin(Wire, I2C_SPEED_SMBUS)) {
        particleSensor.setPulseAmplitudeRed(0x1F);
        particleSensor.setPulseAmplitudeIR(0x1F);
        particleSensor.setPulseAmplitudeGreen(0x00);
        particleSensor.setup(60, 4, 2, 100, 411, 4096);
        ok=true; break;
      }
      delay(250); yield();
    }
    ecui::pipeStepEnd(ok ? "MAX30105 Loaded" : "MAX init failed", ok?ecui::I_OK:ecui::I_ERR);
    if (!ok) scanI2COnce("MAX30105");
    return ok;
  }

  inline void sleep() { particleSensor.shutDown(); isAwake = false; }
  inline void wake()  { particleSensor.wakeUp();   isAwake = true;  }

  float readHeartBeat() {
    if (millis()-lastReadTime < READ_INTERVAL) return beatsPerMinute;
    lastReadTime = millis();

    if (!isAwake) wake(); // ensure sensor on

    // Finger threshold aligned closer to SpO2 path
    long ir = particleSensor.getIR();
    if (ir < 35000) { beatsPerMinute = 0; return beatsPerMinute; }

    static float prevIR = 0;
    unsigned long startTime = millis();
    const unsigned long maxReadTime = 2500;

    while ((millis() - startTime) < maxReadTime) {
      long irVal = particleSensor.getIR();

      // remove DC slowly
      float filtered = irVal - 0.95f * prevIR;
      prevIR = irVal;

      if (checkForBeat(filtered)) {
        unsigned long now = millis();
        unsigned long delta = now - lastBeat;
        lastBeat = now;

        if (delta > 250 && delta < 2000) { // 30–240 BPM bounds
          float newBPM = 60.0f / (delta / 1000.0f);
          if (newBPM >= 30.0f && newBPM <= 180.0f) {
            bpmBuf[(bpmIdx++) & 3] = newBPM;
            float s = bpmBuf[0]+bpmBuf[1]+bpmBuf[2]+bpmBuf[3];
            beatsPerMinute = s / 4.0f; // simple smoothing
            return beatsPerMinute;
          }
        }
      }
      delay(15); yield();
    }
    // no new beat → keep previous BPM
    return beatsPerMinute;
  }

  float readSpO2() {
    // Event card
    ecui::cardHeader("SPO2 SESSION", "EVENT");
    ecui::cardKV("mode", "sampling");
    ecui::cardFooter();

    ecui::pipeStart(false);
    ecui::pipeStepMid(ecui::I_HEART, "HR/SpO₂", "Preparing sensor");

    wake();

    long irValue = particleSensor.getIR();
    if (irValue < 40000) {
      char note[48]; snprintf(note, sizeof(note), "No finger (IR=%ld)", irValue);
      ecui::pipeStepEnd(note, ecui::I_ERR);
      sleep();
      return spo2Value;
    }

    // Progress updates
    for (byte i=0; i<MX3_BUFFER_SIZE; i++) {
      while (!particleSensor.available()) { particleSensor.check(); yield(); }
      redBuffer[i] = particleSensor.getRed();
      irBuffer[i]  = particleSensor.getIR();
      particleSensor.nextSample();

      if (i==0 || i==25 || i==50 || i==75 || i==MX3_BUFFER_SIZE-1) {
        char pbuf[24];
        int pct = (i*100)/(MX3_BUFFER_SIZE-1);
        snprintf(pbuf, sizeof(pbuf), "sampling %d%%", pct);
        ecui::pipeStepMid(ecui::I_HEART, "HR/SpO₂", pbuf);
      }
      delay(10); yield();
    }

    int32_t spo2;   int8_t validSPO2;
    int32_t heartRate; int8_t validHeartRate;
    maxim_heart_rate_and_oxygen_saturation(irBuffer, MX3_BUFFER_SIZE, redBuffer,
                                           &spo2, &validSPO2, &heartRate, &validHeartRate);

    bool hrValid = (validHeartRate && heartRate > 30 && heartRate < 200);
    bool spValid = (validSPO2     && spo2 >= 70      && spo2 <= 100);

    if (spValid) { spo2Value = (float)spo2; }
    if (hrValid) { beatsPerMinute = (float)heartRate; } // <-- reuse HR from algo

    if (spValid) {
      if (hrValid) {
        char both[64]; snprintf(both, sizeof(both), "BPM %ld / SpO₂ %.0f%%", (long)heartRate, spo2Value);
        ecui::pipeStepEnd(both, ecui::I_OK);
      } else {
        char done[48]; snprintf(done, sizeof(done), "SpO₂ %.0f%%", spo2Value);
        ecui::pipeStepEnd(done, ecui::I_OK);
      }
    } else {
      ecui::pipeStepEnd("Invalid SpO₂ calc (keeping previous)", ecui::I_ERR);
    }

    sleep();
    return spo2Value;
  }
};

// ---------------- Sensor facade ----------------
class Sensor {
private:
  MLX90614Sensor mlx; MAX30105Sensor max; bool mlxReady=false, maxReady=false;
public:
  bool begin() {
    // I2C stuck check via card
    if (digitalRead(SDA_PIN)==LOW || digitalRead(SCL_PIN)==LOW) {
      ecui::cardHeader("I2C RECOVERY", "EVENT");
      ecui::cardKV("state", "bus appears stuck");
      ecui::cardFooter();
      ecui::pipeStart(false);
      bool ok = i2cBusRecover();
      ecui::pipeStepEnd(ok ? "I2C recovery OK" : "I2C recovery FAILED", ok?ecui::I_OK:ecui::I_ERR);
    }
    mlxReady = mlx.begin(); delay(400); yield();
    maxReady = max.begin();
    return mlxReady && maxReady;
  }
  void retryFailedSensors() {
    if (!mlxReady) { ecui::cardHeader("INIT RETRY", "EVENT"); ecui::cardKV("sensor","MLX90614"); ecui::cardFooter(); mlxReady = mlx.begin(); }
    if (!maxReady) { ecui::cardHeader("INIT RETRY", "EVENT"); ecui::cardKV("sensor","MAX30105"); ecui::cardFooter(); maxReady = max.begin(); }
  }
  bool isMlxReady() const { return mlxReady; }
  bool isMaxReady() const { return maxReady; }
  float readTemperature() { return mlxReady ? mlx.readCoreBodyTemperature() : NAN; }
  float readHeartBeat()   { return maxReady ? max.readHeartBeat() : NAN; }
  float readSpO2()        { return maxReady ? max.readSpO2() : NAN; }
  float getLastObjC()     { return mlxReady ? mlx.getLastObjC() : NAN; }
  float getLastAmbC()     { return mlxReady ? mlx.getLastAmbC() : NAN; }
};

// ---------------- Constants ----------------
#define AZURE_SDK_CLIENT_USER_AGENT "c%2F" AZ_SDK_VERSION_STRING "(ard;esp8266)"
#define MQTT_PACKET_SIZE 512
#define ONE_HOUR_IN_SECS 3600
#define NTP_SERVERS "pool.ntp.org", "time.nist.gov"
#define DATA_POINTS 30
#define COLLECTION_INTERVAL_MS 3000
#define SEND_INTERVAL_MS 60000
#define SPO2_INTERVAL_MS 10000

// ---------------- IoT Hub configuration ----------------
static const char *ssid = IOT_CONFIG_WIFI_SSID;
static const char *password = IOT_CONFIG_WIFI_PASSWORD;
static const char *host = IOT_CONFIG_IOTHUB_FQDN;
static const char *device_id = IOT_CONFIG_DEVICE_ID;
static const char *device_key = IOT_CONFIG_DEVICE_KEY;
static const int port = 8883;

// ---------------- IoT clients ----------------
static WiFiClientSecure wifi_client;
static X509List cert((const char *)ca_pem);
static PubSubClient mqtt_client(wifi_client);
static az_iot_hub_client client;
static char sas_token[300];
static uint8_t signature[512];
static unsigned char encrypted_signature[32];
static char base64_decoded_device_key[32];
static time_t sasExpiryTime = 0;

// ---------------- Globals ----------------
Sensor sensor; SensorState sensorState;
float tempDats[DATA_POINTS], bpmDats[DATA_POINTS], spo2Dats[DATA_POINTS];
int currentDat = 0;
unsigned long lastSendTime = 0, lastCollectionTime = 0, lastSpO2Time = 0, lastConnectionCheck = 0;
const unsigned long CONNECTION_CHECK_INTERVAL = 30000;

bool mqttConnected = false; int connectionFailures = 0;
const int MAX_CONNECTION_FAILURES = 5;

static uint8_t spo2BackoffIdx = 0;
static const uint32_t spo2BackoffMs[] = {0, 5000, 10000, 20000, 30000, 60000, 120000};
static unsigned long lastHeartRateRead = 0;

// ---- Net labels (ECO shows “Sleeping” instead of “Disconnected”) ----
#if defined(MODE_ECO) || defined(MODE_ULTRA_ECO)
static bool netUp = false;
#endif
const char* wifiStateLabel() {
#if defined(MODE_ECO) || defined(MODE_ULTRA_ECO)
  if (!netUp) return "Sleeping";
#endif
  return (WiFi.status() == WL_CONNECTED) ? "Connected" : "Disconnected";
}
const char* mqttStateLabel() {
#if defined(MODE_ECO) || defined(MODE_ULTRA_ECO)
  if (!netUp) return "Sleeping";
#endif
  return mqtt_client.connected() ? "Connected" : "Disconnected";
}

// ---------------- Net power helpers (used only in ECO) ----------------
#if defined(MODE_ECO) || defined(MODE_ULTRA_ECO)
void netConnect() {
  if (netUp) return;
  ecui::cardHeader("WIFI CONNECT", "EVENT");
  ecui::cardKV("ssid", ssid);
  ecui::cardFooter();

  WiFi.forceSleepWake(); delay(1);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(); delay(50);
  WiFi.begin(ssid, password);

  ecui::pipeStart(false);
  for (int attempts=1; WiFi.status()!=WL_CONNECTED && attempts<=6; attempts++) {
    char s[24]; snprintf(s, sizeof(s), "attempt %d", attempts);
    ecui::pipeStepMid(ecui::I_WIFI, "WiFi", s);
    for (int i=0;i<5 && WiFi.status()!=WL_CONNECTED;i++) { delay(200); serviceLed(); yield(); }
  }
  ecui::pipeStepEnd((WiFi.status()==WL_CONNECTED)?"WiFi connected":"WiFi failed",
                    (WiFi.status()==WL_CONNECTED)?ecui::I_OK:ecui::I_ERR);

  WiFi.setSleep(true);
  netUp = (WiFi.status()==WL_CONNECTED);
}
void netSleep() {
  if (!netUp) return;
  if (mqtt_client.connected()) mqtt_client.disconnect();
  WiFi.disconnect(true);
  delay(50);
  WiFi.forceSleepBegin(); // radio off
  delay(1);
  netUp = false;
}
#ifdef MODE_ULTRA_ECO
void goDeepSleep(uint32_t ms) {
  if (mqtt_client.connected()) mqtt_client.disconnect();
  WiFi.disconnect(true);
  WiFi.forceSleepBegin();
  delay(10);
  ESP.deepSleep((uint64_t)ms * 1000ULL); // requires D0->RST
}
#endif
#endif

#ifdef MODE_PERF
void setupPowerPerf() {
  WiFi.setSleep(true);                 // modem sleep idle
  WiFi.setOutputPower(10.5f);          // ~10 dBm
  mqtt_client.setKeepAlive(45);        // fewer pings
}
#endif

// ---------------- WiFi/time/Azure helpers ----------------
void initializeTime() {
  ecui::cardHeader("TIME SYNC", "EVENT");
  ecui::cardKV("ntp", "pool.ntp.org");
  ecui::cardFooter();

  configTime(0, 0, NTP_SERVERS);
  time_t now = time(NULL);
  int attempts=0, maxAttempts=30;
  while (now < 1510592825 && attempts<maxAttempts) {
    delay(300); now=time(NULL); attempts++; serviceLed(); yield();
    if (attempts%5==0) ecui::pipeStart(false), ecui::pipeStepMid(ecui::I_INFO,"Time","syncing…");
  }
  g_timeSynced = (now >= 1510592825);

  char ts[32];
  if (g_timeSynced) strftime(ts, sizeof(ts), "%Y-%m-%d %H:%M:%S", gmtime(&now));
  else snprintf(ts, sizeof(ts), "unsynced");
  ecui::pipeStart(false);
  ecui::pipeStepEnd(g_timeSynced ? ts : "Time: unsynced", g_timeSynced ? ecui::I_OK : ecui::I_ERR);
}

void initializeClients() {
  az_iot_hub_client_options options = az_iot_hub_client_options_default();
  options.user_agent = AZ_SPAN_FROM_STR(AZURE_SDK_CLIENT_USER_AGENT);
  wifi_client.setTrustAnchors(&cert);

  bool ok=true;
  if (az_result_failed(az_iot_hub_client_init(&client,
      az_span_create((uint8_t *)host, strlen(host)),
      az_span_create((uint8_t *)device_id, strlen(device_id)),
      &options))) ok=false;

  ecui::cardHeader("AZURE CLIENT", "EVENT");
  ecui::cardKV("host", host);
  ecui::cardKV("device", device_id);
  ecui::cardKV("result", ok? "init ok":"init fail");
  ecui::cardFooter();

  mqtt_client.setServer(host, port);
  mqtt_client.setBufferSize(MQTT_PACKET_SIZE);
  mqtt_client.setKeepAlive(60);
}

int generateSasToken(char *sas_token, size_t size) {
  az_span signature_span = az_span_create((uint8_t *)signature, sizeof(signature));
  az_span out_signature_span;
  uint32_t expiration = time(NULL) + ONE_HOUR_IN_SECS;
  if (expiration < 1510592825) {
    ecui::cardHeader("AZURE SAS", "EVENT"); ecui::cardKV("status", "invalid time"); ecui::cardFooter(); return 1;
  }
  if (az_result_failed(az_iot_hub_client_sas_get_signature(&client, expiration, signature_span, &out_signature_span))) {
    ecui::cardHeader("AZURE SAS", "EVENT"); ecui::cardKV("status", "sig fail"); ecui::cardFooter(); return 1;
  }
  int key_len = base64_decode_chars(device_key, strlen(device_key), base64_decoded_device_key);
  if (key_len == 0) {
    ecui::cardHeader("AZURE SAS", "EVENT"); ecui::cardKV("status", "key decode fail"); ecui::cardFooter(); return 1;
  }

  br_hmac_key_context kc; br_hmac_key_init(&kc, &br_sha256_vtable, base64_decoded_device_key, key_len);
  br_hmac_context hmac_ctx; br_hmac_init(&hmac_ctx, &kc, 32);
  br_hmac_update(&hmac_ctx, az_span_ptr(out_signature_span), az_span_size(out_signature_span));
  br_hmac_out(&hmac_ctx, encrypted_signature);

  String b64enc_sig = base64::encode(encrypted_signature, br_hmac_size(&hmac_ctx));
  az_span b64enc_sig_span = az_span_create((uint8_t *)b64enc_sig.c_str(), b64enc_sig.length());

  if (az_result_failed(az_iot_hub_client_sas_get_password(&client, expiration, b64enc_sig_span, AZ_SPAN_EMPTY, sas_token, size, NULL))) {
    ecui::cardHeader("AZURE SAS", "EVENT"); ecui::cardKV("status", "password fail"); ecui::cardFooter(); return 1;
  }

  ecui::cardHeader("AZURE SAS", "EVENT"); ecui::cardKV("status", "token ok"); ecui::cardFooter();
  return 0;
}

int connectToAzureIoTHub() {
  if (WiFi.status() != WL_CONNECTED) {
    ecui::cardHeader("MQTT CONNECT", "EVENT"); ecui::cardKV("status", "wifi down"); ecui::cardFooter(); return 1;
  }

  char mqtt_client_id[128]; size_t client_id_length;
  if (az_result_failed(az_iot_hub_client_get_client_id(&client, mqtt_client_id, sizeof(mqtt_client_id)-1, &client_id_length))) {
    ecui::cardHeader("MQTT CONNECT", "EVENT"); ecui::cardKV("status", "client id fail"); ecui::cardFooter(); return 1;
  }
  mqtt_client_id[client_id_length] = '\0';

  char mqtt_username[256];
  if (az_result_failed(az_iot_hub_client_get_user_name(&client, mqtt_username, sizeof(mqtt_username), NULL))) {
    ecui::cardHeader("MQTT CONNECT", "EVENT"); ecui::cardKV("status", "username fail"); ecui::cardFooter(); return 1;
  }

  ecui::cardHeader("MQTT CONNECT", "EVENT");
  ecui::cardKV("client", mqtt_client_id);
  ecui::cardKV("user", mqtt_username);
  ecui::cardFooter();

  ecui::pipeStart(false);
  bool ok=false;
  for (int attempt=1; attempt<=3 && !ok; attempt++) {
    char lab[24]; snprintf(lab, sizeof(lab), "attempt %d", attempt);
    ecui::pipeStepMid(ecui::I_MQTT, "MQTT", lab);
    if (mqtt_client.connect(mqtt_client_id, mqtt_username, sas_token, NULL, 0, false, NULL, true)) {
      ok=true; break;
    }
    delay(1200); yield();
  }

  if (ok) {
    mqtt_client.subscribe(("devices/" + String(device_id) + "/messages/devicebound/#").c_str());
    mqtt_client.subscribe("$iothub/methods/POST/#");
  }

  ecui::pipeStepEnd(ok ? "MQTT connected" : "MQTT connect failed", ok?ecui::I_OK:ecui::I_ERR);
  return ok?0:1;
}

// ---------------- Event cards (readability helpers) ----------------
void printEvent_CollectionWindow(int window, int total) {
  char ts[32]; formatTimestamp(ts, sizeof(ts));

  ecui::cardHeader("COLLECTION WINDOW", "EVENT");
  ecui::cardKVf("window", "%d/%d", window, total);
  ecui::cardKV("timestamp", ts);
  ecui::cardKVf("heap", "%u B", (unsigned)ESP.getFreeHeap());
  ecui::cardFooter();

  ecui::pipeStart(false);

  char extraWiFi[40] = {0};
  if (WiFi.status()==WL_CONNECTED) snprintf(extraWiFi, sizeof(extraWiFi), "RSSI %d dBm", WiFi.RSSI());
  ecui::pipeStepMid(ecui::I_WIFI, "WiFi", wifiStateLabel(),
                    (WiFi.status()==WL_CONNECTED) ? extraWiFi : nullptr);

  ecui::pipeStepMid(ecui::I_MQTT, "MQTT", mqttStateLabel(), device_id);

  char objStr[24], ambStr[24], tline[64];
  fmtTemp(sensor.getLastObjC(), objStr, sizeof(objStr));
  fmtTemp(sensor.getLastAmbC(), ambStr, sizeof(ambStr));
  snprintf(tline, sizeof(tline), "%s   (Amb %s)", objStr, ambStr);
  ecui::pipeStepMid(ecui::I_TEMP, "Temp", tline);

  // HR/SpO2 connector with partials
  char hrline[64];
  bool haveBPM  = (!isnan(sensorState.getBPM())  && sensorState.getBPM()  > 0);
  bool haveSpO2 = (!isnan(sensorState.getSpO2()) && sensorState.getSpO2() >= 70.0 && sensorState.getSpO2() <= 100.0);

  if (haveBPM && haveSpO2) {
    snprintf(hrline, sizeof(hrline), "BPM %.1f / %.0f%%", sensorState.getBPM(), sensorState.getSpO2());
  } else if (haveBPM) {
    snprintf(hrline, sizeof(hrline), "BPM %.1f (no SpO₂)", sensorState.getBPM());
  } else if (haveSpO2) {
    snprintf(hrline, sizeof(hrline), "SpO₂ %.0f%% (no BPM)", sensorState.getSpO2());
  } else {
    snprintf(hrline, sizeof(hrline), "—");
  }
  ecui::pipeStepMid(ecui::I_HEART, "HR/SpO₂", hrline);

  // Dynamic tail
  if (!haveBPM && haveSpO2)       ecui::pipeStepEnd("HR pending (SpO₂ ready)", ecui::I_INFO);
  else if (haveBPM && !haveSpO2)  ecui::pipeStepEnd("SpO₂ pending (HR ready)", ecui::I_INFO);
  else if (!haveBPM && !haveSpO2) ecui::pipeStepEnd("Awaiting sensor contact…", ecui::I_INFO);
  else                            ecui::pipeStepEnd("Awaiting next action…", ecui::I_INFO);
}

void printEvent_TxResult(const char* topic, bool sent, float avgTemp, int cT, float avgBPM, int cB, float avgSpO2, int cS) {
  ecui::cardHeader("TRANSMIT RESULT", "EVENT");
  ecui::cardKV("topic", topic);
  ecui::cardKVf("avg temp", "%.2f °C (%d)", avgTemp, cT);
  ecui::cardKVf("avg BPM",  "%.2f (%d)",   avgBPM,  cB);
  ecui::cardKVf("avg SpO₂", "%.1f %% (%d)",avgSpO2, cS);
  ecui::cardFooter();

  ecui::pipeStart(false);
  char extraWiFi[40] = {0};
  if (WiFi.status()==WL_CONNECTED) snprintf(extraWiFi, sizeof(extraWiFi), "RSSI %d dBm", WiFi.RSSI());
  ecui::pipeStepMid(ecui::I_WIFI, "WiFi", wifiStateLabel(),
                    (WiFi.status()==WL_CONNECTED) ? extraWiFi : nullptr);
  ecui::pipeStepMid(ecui::I_MQTT, "MQTT", mqttStateLabel(), device_id);
  ecui::pipeStepEnd(sent ? "Data sent to Azure IoT Hub" : "Publish failed", sent?ecui::I_OK:ecui::I_ERR);
}

// ---------------- Connections + lifecycle ----------------
void establishConnection() {
  setLed(LED_FAST);

  ecui::cardHeader("WIFI CONNECT", "EVENT");
  ecui::cardKV("ssid", ssid);
  ecui::cardFooter();

  WiFi.mode(WIFI_STA);
  WiFi.disconnect(); delay(50);
  WiFi.begin(ssid, password);

  ecui::pipeStart(false);
  bool wOk=false;
  for (int attempt=1; attempt<=6 && !wOk; attempt++) {
    char s[24]; snprintf(s, sizeof(s), "attempt %d", attempt);
    ecui::pipeStepMid(ecui::I_WIFI, "WiFi", s);
    for (int i=0;i<5 && WiFi.status()!=WL_CONNECTED;i++) { delay(200); serviceLed(); yield(); }
    wOk = (WiFi.status()==WL_CONNECTED);
  }
  ecui::pipeStepEnd(wOk ? "WiFi connected" : "WiFi failed", wOk?ecui::I_OK:ecui::I_ERR);

  ecui::cardHeader("WIFI STATUS", "EVENT");
  if (WiFi.status()==WL_CONNECTED) {
    ecui::cardKV("ip", WiFi.localIP().toString().c_str());
    ecui::cardKVf("rssi", "%d dBm", WiFi.RSSI());
  } else {
    ecui::cardKV("state", "failed");
  }
  ecui::cardFooter();

  initializeTime();
  initializeClients();

  if (generateSasToken(sas_token, sizeof(sas_token)) == 0) {
    sasExpiryTime = time(NULL) + ONE_HOUR_IN_SECS - 60;
    connectToAzureIoTHub();
  } else {
    ecui::pipeStart(false); ecui::pipeStepEnd("SAS generation failed", ecui::I_ERR);
  }
}

void checkConnections() {
  unsigned long currentTime = millis();
  if (currentTime - lastConnectionCheck >= CONNECTION_CHECK_INTERVAL) {
    lastConnectionCheck = currentTime;

    if (WiFi.status() != WL_CONNECTED) {
      ecui::cardHeader("WIFI RECONNECT", "EVENT"); ecui::cardKV("state", "reconnecting"); ecui::cardFooter();
      setLed(LED_FAST);
      WiFi.disconnect(); delay(50); WiFi.begin(ssid, password);
    }

    if (!mqtt_client.connected()) {
      ecui::cardHeader("MQTT RECONNECT", "EVENT"); ecui::cardKV("state", "reconnecting"); ecui::cardFooter();
      if (generateSasToken(sas_token, sizeof(sas_token)) == 0) {
        sasExpiryTime = time(NULL) + ONE_HOUR_IN_SECS - 60;
        connectToAzureIoTHub();
      }
    }

    if (connectionFailures >= MAX_CONNECTION_FAILURES) {
      ecui::cardHeader("SYSTEM", "EVENT"); ecui::cardKV("restart", "too many failures"); ecui::cardFooter();
      setLed(LED_ERROR);
      ESP.restart();
    }
  }
}

// ---------------- Data collection & send ----------------
void collectData() {
  if (currentDat < DATA_POINTS) {
    float temp = sensorState.getTemperature();
    float bpm  = sensorState.getBPM();
    float spo2 = sensorState.getSpO2();

    tempDats[currentDat] = temp;
    bpmDats[currentDat]  = bpm;
    spo2Dats[currentDat] = spo2;
    currentDat++;

    printEvent_CollectionWindow(currentDat, DATA_POINTS);
  }
}

void sendAveragedData() {
  if (currentDat == 0) return;

  float avgTemp=0, avgBPM=0, avgSpO2=0; int cT=0,cB=0,cS=0;
  for (int i=0;i<currentDat;i++) {
    if (!isnan(tempDats[i]) && tempDats[i]>=10.0 && tempDats[i]<=60.0){ avgTemp+=tempDats[i]; cT++; }
    if (!isnan(bpmDats[i])  && bpmDats[i]>0)                         { avgBPM +=bpmDats[i];  cB++; }
    if (!isnan(spo2Dats[i]) && spo2Dats[i]>=70.0 && spo2Dats[i]<=100.0){ avgSpO2+=spo2Dats[i]; cS++; }
  }
  avgTemp = cT>0 ? (avgTemp/cT) : (isnan(sensorState.getTemperature())?27.0f:sensorState.getTemperature());
  avgBPM  = cB>0 ? (avgBPM/cB) : 0.0f;
  avgSpO2 = cS>0 ? (avgSpO2/cS) : 98.0f;
  currentDat = 0;

  String topic = "devices/" + String(device_id) + "/messages/events/";
  char payload[192];
  char timestamp[32]; time_t now = time(NULL);
  strftime(timestamp, sizeof(timestamp), "%Y-%m-%dT%H:%M:%SZ", gmtime(&now));
  snprintf(payload, sizeof(payload),
    "{\"deviceId\":\"%s\",\"pulseRate\":%.2f,\"temperature\":%.2f,"
    "\"sp02\":%.1f,\"timestamp\":\"%s\"}",
    device_id, avgBPM, avgTemp, avgSpO2, timestamp);

  bool sentOk = false;

  #ifdef MODE_ULTRA_ECO
    netConnect(); initializeClients(); generateSasToken(sas_token, sizeof(sas_token)); connectToAzureIoTHub();
    sentOk = mqtt_client.publish(topic.c_str(), payload);
    mqtt_client.loop(); delay(50);
    printEvent_TxResult(topic.c_str(), sentOk, avgTemp, cT, avgBPM, cB, avgSpO2, cS);
    goDeepSleep(SEND_INTERVAL_MS); // never returns
    return;
  #endif

  #ifdef MODE_ECO
    netConnect(); initializeClients(); if (generateSasToken(sas_token, sizeof(sas_token))==0) connectToAzureIoTHub();
    sentOk = mqtt_client.publish(topic.c_str(), payload);
    mqtt_client.loop(); delay(50);
    printEvent_TxResult(topic.c_str(), sentOk, avgTemp, cT, avgBPM, cB, avgSpO2, cS);
    netSleep();
  #else
    if (WiFi.status()!=WL_CONNECTED) establishConnection();
    if (!mqtt_client.connected()) connectToAzureIoTHub();
    sentOk = mqtt_client.publish(topic.c_str(), payload);
    printEvent_TxResult(topic.c_str(), sentOk, avgTemp, cT, avgBPM, cB, avgSpO2, cS);
  #endif

  connectionFailures += sentOk ? 0 : 1;
}

// ---------------- Arduino setup/loop ----------------
void setup() {
  pinMode(LED_PIN, OUTPUT); digitalWrite(LED_PIN, HIGH); setLed(LED_FAST);

  Serial.begin(115200);
  while (!Serial) { yield(); delay(10); }

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(I2C_SPEED_SMBUS);
  Wire.setClockStretchLimit(I2C_STRETCH_US);

  // I2C boot scan card
  scanI2COnce("I2C BOOT SCAN");

  // Boot card
  ecui::cardHeader("BOOT", "EVENT");
  ecui::cardKVf("heap", "%u B", (unsigned)ESP.getFreeHeap());
  char idBuf[16];
  snprintf(idBuf, sizeof(idBuf), "%08X", ESP.getChipId());      ecui::cardKV("chip id", idBuf);
  snprintf(idBuf, sizeof(idBuf), "%08X", ESP.getFlashChipId()); ecui::cardKV("flash id", idBuf);
  ecui::cardFooter();

  #ifdef MODE_PERF
    setupPowerPerf();
    establishConnection();
  #else
    WiFi.forceSleepBegin(); delay(1); // ECO/ULTRA_ECO: radio off until needed
  #endif

  if (!sensor.begin()) {
    ecui::cardHeader("SENSORS", "EVENT"); ecui::cardKV("init", "some sensors failed"); ecui::cardFooter();
  }

  lastSendTime = millis(); lastCollectionTime = millis();
  lastSpO2Time = millis(); lastConnectionCheck = millis();

  ecui::cardHeader("READY", "EVENT"); ecui::cardKV("status", "main loop"); ecui::cardFooter();
  setLed(LED_SLOW);
}

void loop() {
  serviceLed();
  unsigned long currentTime = millis();

  // SAS token proactive refresh
  bool canRefresh = true; // PERF
  if (canRefresh && sasExpiryTime > 0 && time(NULL) >= sasExpiryTime) {
    ecui::cardHeader("AZURE SAS", "EVENT"); ecui::cardKV("action", "refresh"); ecui::cardFooter();
    if (generateSasToken(sas_token, sizeof(sas_token)) == 0) {
      sasExpiryTime = time(NULL) + ONE_HOUR_IN_SECS - 60;
      if (!mqtt_client.connected()) connectToAzureIoTHub();
      else { mqtt_client.disconnect(); delay(100); connectToAzureIoTHub(); }
    } else {
      ecui::pipeStart(false); ecui::pipeStepEnd("SAS refresh failed", ecui::I_ERR);
    }
  }

  // Sensor recovery timer (every 30s)
  static unsigned long lastSensorRetry = 0;
  if ((!sensor.isMlxReady() || !sensor.isMaxReady()) && (currentTime - lastSensorRetry >= 30000UL)) {
    sensor.retryFailedSensors(); lastSensorRetry = currentTime;
  }

  // SpO2 cadence with backoff
  if (currentTime - lastSpO2Time >= (SPO2_INTERVAL_MS + spo2BackoffMs[spo2BackoffIdx])) {
    float newSpo2 = sensor.readSpO2();
    bool validSp = (!isnan(newSpo2) && newSpo2 >= 70.0f && newSpo2 <= 100.0f);

    float tempNow = sensor.readTemperature();
    float bpmNow = sensorState.getBPM();
    if (currentTime - lastHeartRateRead >= 2000UL) { bpmNow = sensor.readHeartBeat(); lastHeartRateRead = currentTime; }

    sensorState.setState(tempNow, bpmNow, validSp ? newSpo2 : sensorState.getSpO2());
    spo2BackoffIdx = validSp ? 0 : (spo2BackoffIdx < 6 ? spo2BackoffIdx + 1 : 6);
    lastSpO2Time = currentTime;
  } else {
    float tempNow = sensor.readTemperature();
    float bpmNow = sensorState.getBPM();
    if (currentTime - lastHeartRateRead >= 2000UL) { bpmNow = sensor.readHeartBeat(); lastHeartRateRead = currentTime; }
    sensorState.setState(tempNow, bpmNow, sensorState.getSpO2());
  }

  if (currentTime - lastCollectionTime >= COLLECTION_INTERVAL_MS) { collectData(); lastCollectionTime = currentTime; }
  if (currentTime - lastSendTime >= SEND_INTERVAL_MS) { sendAveragedData(); lastSendTime = currentTime; }

  checkConnections();

  if (mqtt_client.connected()) mqtt_client.loop();

  delay(10); yield();
}
