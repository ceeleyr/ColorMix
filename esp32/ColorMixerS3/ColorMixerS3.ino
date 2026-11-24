/*
 * ColorMix – ESP32-S3 Paint Mixer
 * Closed-loop colour matching: colour sensor + one PID per pump (Red/Green/Blue), tuned with Cohen-Coon.
 *
 * ─── PROSES SATU ORDER ──────────────────────────────────────────────────────────────
 *  1. BASE   : tuang BASE_FRACTION (80%) dari takaran Node-RED, open-loop (waktu = ml / ML_PER_SEC)
 *  2. PREMIX : aduk supaya campuran rata
 *  3. TRIM   : pengaduk tetap jalan, sensor warna dibaca tiap 100 ms.
 *              Untuk tiap warna c:  error e_c = chroma target - chroma terukur
 *              (chroma = R/(R+G+B), jadi terang/gelapnya cahaya tidak berpengaruh).
 *              Satu PID per pompa -> output = total ml tambahan cat itu.
 *              PID memilih pompa R/G/B yang paling kurang, lalu menuang sedikit demi sedikit.
 *              Cat tidak bisa diambil lagi -> output hanya boleh naik (anti-windup di batas bawah).
 *  4. DONE   : semua |e_c| < TOL selama HOLD_MS, atau jatah trim habis / timeout.
 *
 * ─── COHEN-COON (per pompa, perintah MQTT "tune") ──────────────────────────────────
 *  Uji step: gelas berisi campuran, pengaduk jalan, tambahkan sedikit (mis. 5 ml) satu cat,
 *  rekam respons chroma channel itu -> model FOPDT:  K (chroma/ml), tau (s), theta (s)
 *      metode 2 titik:  tau = 1.5 (t63 - t28),  theta = t63 - tau
 *  Rumus Cohen-Coon PID:
 *      r  = theta / tau
 *      Kp = (1/K) * (1/r) * (4/3 + r/4)
 *      Ti = theta * (32 + 6r) / (13 + 8r)
 *      Td = theta * 4 / (11 + 2r)
 *  K bergantung volume di gelas, jadi disimpan sebagai Kv = K * volume, lalu dihitung ulang saat dipakai.
 *  Cohen-Coon cukup agresif (target decay ratio 1/4), padahal overshoot cat tidak bisa dikoreksi,
 *  jadi gain dikali DETUNE (default 0.5).
 *
 * ─── MQTT ───────────────────────────────────────────────────────────────────────────
 *  Subscribe colormix/order     {"id","name","r","g","b","hex","vol","dose":{"red","green","blue"}}
 *            colormix/cmd       {"cmd":"stop"} | {"cmd":"calwhite"} | {"cmd":"calblack"} | {"cmd":"info"}
 *                               {"cmd":"tune","pump":"red","ml":5,"vol":250,"ms":20000}
 *                               {"cmd":"model","pump":"red","K":0.0024,"vol":250,"tau":3,"theta":1.5}
 *                               {"cmd":"prime","pump":"red","ms":3000}
 *  Publish   colormix/status    {"device","state","order","progress","step","rssi", ...}   (retained, LWT offline)
 *            colormix/telemetry tiap 0.5 s saat TRIM (target vs terukur, ml trim)
 *            colormix/tune      hasil tuning Cohen-Coon / info parameter
 *
 * ─── HARDWARE ───────────────────────────────────────────────────────────────────────
 *  - ESP32-S3 DevKit
 *  - 3 pompa via relay (Merah, Hijau, Biru) + 1 relay pengaduk
 *  - Sensor warna TCS34725 (I2C) ATAU TCS3200/TCS230 (frekuensi), pilih dengan USE_TCS34725
 *  - LED RGB (common cathode) menampilkan warna target
 *
 *  Board  : "ESP32S3 Dev Module", USB CDC On Boot: Enabled (untuk Serial Monitor lewat port USB)
 *  Core   : Arduino-ESP32 2.0.x atau 3.x
 *  Library: PubSubClient, ArduinoJson 7.x, Adafruit TCS34725 (kalau pakai TCS34725)
 */

#include <Arduino.h>
#include <esp_arduino_version.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <Wire.h>

// ===================== CONFIG =====================
#ifndef USE_TCS34725
#define USE_TCS34725 1          // 1 = TCS34725 (I2C),  0 = TCS3200 / TCS230
#endif

const char* WIFI_SSID = "WIFI_SSID";
const char* WIFI_PASS = "WIFI_PASSWORD";
const char* MQTT_HOST = "192.168.1.10";   // IP laptop yang menjalankan docker (ipconfig)
const uint16_t MQTT_PORT = 1883;
const char* DEVICE_ID = "esp32s3-mixer";

const char* T_ORDER  = "colormix/order";
const char* T_CMD    = "colormix/cmd";
const char* T_STATUS = "colormix/status";
const char* T_TELE   = "colormix/telemetry";
const char* T_TUNE   = "colormix/tune";

// Pompa: Merah, Hijau, Biru  (pin aman untuk ESP32-S3)
const int   NUM_PUMPS = 3;
const char* PUMP_KEYS[NUM_PUMPS]  = { "red", "green", "blue" };
const int   PUMP_PINS[NUM_PUMPS]  = { 4, 5, 6 };
const float ML_PER_SEC[NUM_PUMPS] = { 5.0, 5.0, 5.0 };   // KALIBRASI: ml keluar per detik
const int   STIR_PIN = 7;
const bool  RELAY_ACTIVE_LOW = true;

// Sensor warna
#if USE_TCS34725
const int I2C_SDA = 8, I2C_SCL = 9;
#else
const int TCS_S0 = 10, TCS_S1 = 11, TCS_S2 = 12, TCS_S3 = 13, TCS_OUT = 14;
#endif

// LED RGB warna target
const int LED_R = 15, LED_G = 16, LED_B = 17;

// Proses mixing
const float BASE_FRACTION = 0.80;          // porsi takaran yang dituang open-loop dulu
const float OVERFILL      = 0.10;          // boleh lebih 10% dari volume order untuk koreksi
const unsigned long PREMIX_MS = 8000;      // aduk sebelum kontrol dimulai
const unsigned long SAMPLE_MS = 100;       // periode baca sensor = periode PID
const float CHROMA_ALPHA  = 0.3;           // filter EMA pembacaan sensor
const float TOL           = 0.010;         // toleransi error chroma (1%)
const unsigned long HOLD_MS = 4000;        // error harus < TOL selama ini
const unsigned long TRIM_TIMEOUT_MS = 120000;
const float MIN_PULSE_ML  = 0.3;           // tuang minimal per pulsa
const float MAX_PULSE_ML  = 2.0;           // tuang maksimal per pulsa (lalu dievaluasi lagi)
const float DETUNE        = 0.5;           // pengali gain Cohen-Coon (1.0 = asli)
const unsigned long TELE_MS = 500;

// Default model FOPDT sebelum di-tune (Kv = K * volume)
const float DEF_KV = 0.6, DEF_TAU = 3.0, DEF_THETA = 1.5;

// Uji step (tune)
const unsigned long TUNE_BASE_MS = 3000;
const int   TUNE_MAX = 400;                // 400 sampel x 100 ms = 40 s
const float TUNE_MIN_DY = 0.003;
// ==================================================

#if USE_TCS34725
#include <Adafruit_TCS34725.h>
Adafruit_TCS34725 tcs(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
#endif

// LEDC API berbeda antara core 2.x dan 3.x
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  #define LED_SETUP(pin, ch) ledcAttach(pin, 5000, 8)
  #define LED_WRITE(pin, ch, v) ledcWrite(pin, v)
#else
  #define LED_SETUP(pin, ch) do { ledcSetup(ch, 5000, 8); ledcAttachPin(pin, ch); } while (0)
  #define LED_WRITE(pin, ch, v) ledcWrite(ch, v)
#endif

WiFiClient net;
PubSubClient mqtt(net);
Preferences prefs;

enum State { IDLE, BASE, PREMIX, TRIM, DONE, PRIMING, TUNE_BASE, TUNE_RUN };
State state = IDLE;

struct Model { float Kv, tau, theta; };          // FOPDT: ml cat -> chroma
struct Pid   { float kp, ki, kd, I, prevY, dF; bool first; };

Model model[NUM_PUMPS];
Pid   pid[NUM_PUMPS];

// sensor
float calWhite[3] = { 1000, 1000, 1000 }, calBlack[3] = { 0, 0, 0 };
float chroma[3]    = { 1 / 3.f, 1 / 3.f, 1 / 3.f };   // terfilter
float chromaRaw[3] = { 1 / 3.f, 1 / 3.f, 1 / 3.f };
float rgb255[3]    = { 0, 0, 0 };
bool  sensorOk = false, sensorFirst = true, newSample = false;
unsigned long lastSample = 0, lastSampleUsed = 0;

// pompa
int   activePump = -1;
unsigned long pumpStartAt = 0, pumpEndAt = 0;
float doneMl[NUM_PUMPS];

// order
String orderId = "";
int   targetRgb[3];
float tc[3];                 // chroma target
float baseMl[NUM_PUMPS], volTarget = 0;
int   baseIdx = -1;
bool  trimEnabled = false;
unsigned long phaseStart = 0, doneAt = 0;

// trim
float trimStartMl[NUM_PUMPS], uCmd[NUM_PUMPS], trimBudget = 0, initialErr = 1, lastErr = 1;
unsigned long holdSince = 0, budgetSince = 0;
const char* result = "";

// tune
int   tunePump = 0;
float tuneMl = 5, tuneVol = 250, tuneY0 = 0;
unsigned long tuneMs = 20000, tuneT0 = 0;
int   tuneN = 0, tuneBaseN = 0;
float tuneY[TUNE_MAX], tuneT[TUNE_MAX];

unsigned long lastPub = 0, lastTele = 0, lastMqttTry = 0;

// ─────────────────────────── helpers ───────────────────────────
void relay(int pin, bool on) {
  if (pin < 0) return;
  digitalWrite(pin, (on ^ RELAY_ACTIVE_LOW) ? HIGH : LOW);
}

void setLed(int r, int g, int b) {
  LED_WRITE(LED_R, 0, r);
  LED_WRITE(LED_G, 1, g);
  LED_WRITE(LED_B, 2, b);
}

int pumpIndex(const char* key) {
  for (int i = 0; i < NUM_PUMPS; i++) if (strcmp(key, PUMP_KEYS[i]) == 0) return i;
  return -1;
}

bool busy() { return state != IDLE && state != DONE; }

// Cohen-Coon PID dari model FOPDT
void cohenCoon(float K, float tau, float theta, float& kp, float& Ti, float& Td) {
  float r = theta / tau;
  kp = (1.0f / K) * (1.0f / r) * (4.0f / 3.0f + r / 4.0f);
  Ti = theta * (32.0f + 6.0f * r) / (13.0f + 8.0f * r);
  Td = theta * 4.0f / (11.0f + 2.0f * r);
}

// ─────────────────────────── pompa ───────────────────────────
void pumpStart(int i, float ml) {
  if (ml <= 0) return;
  activePump = i;
  pumpStartAt = millis();
  pumpEndAt = pumpStartAt + (unsigned long)(ml / ML_PER_SEC[i] * 1000.0f);
  relay(PUMP_PINS[i], true);
}

void pumpFinish(unsigned long until) {
  if (activePump < 0) return;
  relay(PUMP_PINS[activePump], false);
  doneMl[activePump] += (until - pumpStartAt) / 1000.0f * ML_PER_SEC[activePump];
  activePump = -1;
}

void pumpUpdate() {
  if (activePump >= 0 && millis() >= pumpEndAt) pumpFinish(pumpEndAt);
}

void pumpAbort() { pumpFinish(millis()); }

float liveMl(int i) {
  float ml = doneMl[i];
  if (activePump == i) ml += (millis() - pumpStartAt) / 1000.0f * ML_PER_SEC[i];
  return ml;
}

float totalMl() {
  float s = 0;
  for (int i = 0; i < NUM_PUMPS; i++) s += liveMl(i);
  return s;
}

// ─────────────────────────── sensor ───────────────────────────
bool readRaw(float raw[3]) {
#if USE_TCS34725
  uint16_t r, g, b, c;
  tcs.getRawData(&r, &g, &b, &c);     // blok ~50 ms (waktu integrasi)
  if (c == 0) return false;
  raw[0] = r; raw[1] = g; raw[2] = b;
  return true;
#else
  // TCS3200: frekuensi output sebanding intensitas. S2/S3: R=LL, G=HH, B=LH
  const uint8_t s2[3] = { LOW, HIGH, LOW }, s3[3] = { LOW, HIGH, HIGH };
  for (int c = 0; c < 3; c++) {
    digitalWrite(TCS_S2, s2[c]);
    digitalWrite(TCS_S3, s3[c]);
    delayMicroseconds(200);
    unsigned long per = pulseIn(TCS_OUT, LOW, 40000);
    if (per == 0) return false;
    raw[c] = 1e6f / (2.0f * per);     // Hz
  }
  return true;
#endif
}

void sampleSensor() {
  if (millis() - lastSample < SAMPLE_MS) return;
  lastSample = millis();
  float raw[3];
  if (!readRaw(raw)) { sensorOk = false; return; }
  float v[3], s = 0;
  for (int c = 0; c < 3; c++) {
    float span = calWhite[c] - calBlack[c];
    v[c] = span > 1 ? (raw[c] - calBlack[c]) / span : 0;
    v[c] = constrain(v[c], 0.0f, 1.2f);
    rgb255[c] = constrain(v[c] * 255.0f, 0.0f, 255.0f);
    s += v[c];
  }
  for (int c = 0; c < 3; c++) {
    chromaRaw[c] = s > 0.01f ? v[c] / s : 1 / 3.f;
    chroma[c] = sensorFirst ? chromaRaw[c] : chroma[c] + CHROMA_ALPHA * (chromaRaw[c] - chroma[c]);
  }
  sensorFirst = false;
  sensorOk = true;
  newSample = true;
}

void averageRaw(float out[3], int n) {
  out[0] = out[1] = out[2] = 0;
  int ok = 0;
  for (int k = 0; k < n; k++) {
    float raw[3];
    if (readRaw(raw)) { for (int c = 0; c < 3; c++) out[c] += raw[c]; ok++; }
    delay(20);
  }
  for (int c = 0; c < 3; c++) out[c] = ok ? out[c] / ok : 0;
}

// ─────────────────────────── persistence ───────────────────────────
void loadSettings() {
  prefs.begin("colormix", true);
  for (int i = 0; i < NUM_PUMPS; i++) {
    char k[12];
    snprintf(k, sizeof k, "kv%d", i);  model[i].Kv    = prefs.getFloat(k, DEF_KV);
    snprintf(k, sizeof k, "tau%d", i); model[i].tau   = prefs.getFloat(k, DEF_TAU);
    snprintf(k, sizeof k, "th%d", i);  model[i].theta = prefs.getFloat(k, DEF_THETA);
  }
  prefs.getBytes("white", calWhite, sizeof calWhite);
  prefs.getBytes("black", calBlack, sizeof calBlack);
  prefs.end();
}

void saveModel(int i) {
  prefs.begin("colormix", false);
  char k[12];
  snprintf(k, sizeof k, "kv%d", i);  prefs.putFloat(k, model[i].Kv);
  snprintf(k, sizeof k, "tau%d", i); prefs.putFloat(k, model[i].tau);
  snprintf(k, sizeof k, "th%d", i);  prefs.putFloat(k, model[i].theta);
  prefs.end();
}

void saveCal() {
  prefs.begin("colormix", false);
  prefs.putBytes("white", calWhite, sizeof calWhite);
  prefs.putBytes("black", calBlack, sizeof calBlack);
  prefs.end();
}

// ─────────────────────────── status / publish ───────────────────────────
const char* stateName() {
  switch (state) {
    case BASE: case PREMIX: case TRIM: return "mixing";
    case TUNE_BASE: case TUNE_RUN:     return "tuning";
    case PRIMING:                      return "priming";
    case DONE:                         return "done";
    default:                           return "idle";
  }
}

const char* stepName() {
  static char buf[24];
  switch (state) {
    case BASE:   return baseIdx >= 0 && baseIdx < NUM_PUMPS ? PUMP_KEYS[baseIdx] : "";
    case PREMIX: return "premix";
    case TRIM:
      if (activePump < 0) return "trim";
      snprintf(buf, sizeof buf, "trim-%s", PUMP_KEYS[activePump]);
      return buf;
    case TUNE_BASE: case TUNE_RUN:
      snprintf(buf, sizeof buf, "tune-%s", PUMP_KEYS[tunePump]);
      return buf;
    case PRIMING:
      snprintf(buf, sizeof buf, "prime-%s", activePump >= 0 ? PUMP_KEYS[activePump] : "");
      return buf;
    default: return "";
  }
}

int progress() {
  unsigned long el = millis() - phaseStart;
  float p = 0, base = 0;
  switch (state) {
    case BASE:
      for (int i = 0; i < NUM_PUMPS; i++) base += baseMl[i];
      p = base > 0 ? 60.0f * totalMl() / base : 60;
      break;
    case PREMIX: p = 60 + 10.0f * el / PREMIX_MS; break;
    case TRIM:   p = 70 + 29.0f * (1.0f - lastErr / max(initialErr, 1e-4f)); break;
    case DONE:   p = 100; break;
    case TUNE_BASE: case TUNE_RUN:
      p = 100.0f * (millis() - phaseStart) / (TUNE_BASE_MS + tuneMs); break;
    default: p = 0;
  }
  return (int)constrain(p, 0.0f, 100.0f);
}

void publishStatus(const char* overrideState = nullptr, const char* error = nullptr) {
  JsonDocument doc;
  doc["device"]   = DEVICE_ID;
  doc["state"]    = overrideState ? overrideState : stateName();
  doc["order"]    = orderId;
  doc["progress"] = progress();
  doc["step"]     = stepName();
  doc["rssi"]     = WiFi.RSSI();
  doc["sensor"]   = sensorOk;
  if (error) doc["error"] = error;
  if (state == DONE) {
    doc["result"] = result;
    doc["err"]    = lastErr;
    JsonObject m = doc["measured"].to<JsonObject>();
    m["r"] = (int)rgb255[0]; m["g"] = (int)rgb255[1]; m["b"] = (int)rgb255[2];
    JsonObject ml = doc["ml"].to<JsonObject>();
    for (int i = 0; i < NUM_PUMPS; i++) ml[PUMP_KEYS[i]] = roundf(liveMl(i) * 10) / 10;
  }
  char buf[512];
  size_t n = serializeJson(doc, buf);
  mqtt.publish(T_STATUS, (const uint8_t*)buf, n, true);
  lastPub = millis();
}

void publishTelemetry() {
  JsonDocument doc;
  doc["order"] = orderId;
  doc["phase"] = stepName();
  doc["tr"] = tc[0];        doc["tg"] = tc[1];        doc["tb"] = tc[2];         // chroma target
  doc["yr"] = chroma[0];    doc["yg"] = chroma[1];    doc["yb"] = chroma[2];     // chroma terukur
  doc["mr"] = (int)rgb255[0]; doc["mg"] = (int)rgb255[1]; doc["mb"] = (int)rgb255[2];
  doc["ur"] = liveMl(0) - trimStartMl[0];                                       // ml trim sudah dituang
  doc["ug"] = liveMl(1) - trimStartMl[1];
  doc["ub"] = liveMl(2) - trimStartMl[2];
  doc["err"] = lastErr;
  char buf[384];
  size_t n = serializeJson(doc, buf);
  mqtt.publish(T_TELE, (const uint8_t*)buf, n, false);
  lastTele = millis();
}

void publishInfo() {
  JsonDocument doc;
  doc["type"]   = "info";
  doc["device"] = DEVICE_ID;
  doc["sensor"] = USE_TCS34725 ? "TCS34725" : "TCS3200";
  doc["sensor_ok"] = sensorOk;
  JsonArray w = doc["white"].to<JsonArray>();
  JsonArray b = doc["black"].to<JsonArray>();
  JsonArray m = doc["measured"].to<JsonArray>();
  JsonArray ch = doc["chroma"].to<JsonArray>();
  for (int c = 0; c < 3; c++) { w.add(calWhite[c]); b.add(calBlack[c]); m.add((int)rgb255[c]); ch.add(chroma[c]); }
  JsonObject models = doc["models"].to<JsonObject>();
  for (int i = 0; i < NUM_PUMPS; i++) {
    float kp, Ti, Td, K = model[i].Kv / 250.0f;
    cohenCoon(K, model[i].tau, model[i].theta, kp, Ti, Td);
    JsonObject o = models[PUMP_KEYS[i]].to<JsonObject>();
    o["Kv"] = model[i].Kv; o["tau"] = model[i].tau; o["theta"] = model[i].theta;
    o["kp_at_250ml"] = kp; o["Ti"] = Ti; o["Td"] = Td;
  }
  char buf[1024];
  size_t n = serializeJson(doc, buf);
  mqtt.publish(T_TUNE, (const uint8_t*)buf, n, false);
}

void publishTuneError(const char* err) {
  JsonDocument doc;
  doc["type"] = "tune"; doc["ok"] = false; doc["pump"] = PUMP_KEYS[tunePump]; doc["error"] = err;
  char buf[200];
  size_t n = serializeJson(doc, buf);
  mqtt.publish(T_TUNE, (const uint8_t*)buf, n, false);
  Serial.printf("Tune failed: %s\n", err);
}

// ─────────────────────────── job control ───────────────────────────
void finish(const char* res) {
  pumpAbort();
  relay(STIR_PIN, false);
  result = res;
  state = DONE;
  doneAt = millis();
  Serial.printf("Order %s selesai (%s), err=%.4f, ml R=%.1f G=%.1f B=%.1f\n",
                orderId.c_str(), res, lastErr, liveMl(0), liveMl(1), liveMl(2));
  publishStatus();
}

void stopJob(const char* reason) {
  pumpAbort();
  relay(STIR_PIN, false);
  Serial.printf("Job stopped: %s\n", reason);
  publishStatus("error", reason);
  state = IDLE;
  orderId = "";
  setLed(0, 0, 0);
}

void nextBasePump() {
  do { baseIdx++; } while (baseIdx < NUM_PUMPS && baseMl[baseIdx] <= 0.05f);
  if (baseIdx < NUM_PUMPS) {
    pumpStart(baseIdx, baseMl[baseIdx]);
    Serial.printf("BASE %s %.1f ml\n", PUMP_KEYS[baseIdx], baseMl[baseIdx]);
  } else {
    relay(STIR_PIN, true);
    phaseStart = millis();
    state = PREMIX;
    Serial.println("PREMIX...");
  }
  publishStatus();
}

float maxError() {
  float m = 0;
  for (int c = 0; c < 3; c++) m = max(m, fabsf(tc[c] - chroma[c]));
  return m;
}

void startTrim() {
  float vol = max(totalMl(), 1.0f);
  for (int i = 0; i < NUM_PUMPS; i++) {
    float kp, Ti, Td, K = model[i].Kv / vol;
    cohenCoon(K, model[i].tau, model[i].theta, kp, Ti, Td);
    kp *= DETUNE;
    pid[i] = { kp, kp / Ti, kp * Td, 0, chroma[i], 0, true };
    trimStartMl[i] = liveMl(i);
    uCmd[i] = 0;
    Serial.printf("PID %s: K=%.5f Kp=%.1f Ti=%.2f Td=%.2f\n", PUMP_KEYS[i], K, kp, Ti, Td);
  }
  trimBudget = volTarget * (1.0f - BASE_FRACTION) + volTarget * OVERFILL;
  initialErr = lastErr = maxError();
  holdSince = budgetSince = 0;
  phaseStart = millis();
  state = TRIM;
  Serial.printf("TRIM mulai, err awal %.4f, jatah %.1f ml\n", initialErr, trimBudget);
  publishStatus();
}

// satu langkah PID, dipanggil tiap sampel sensor baru
void trimStep(float dt) {
  float trimTotal = 0;
  for (int i = 0; i < NUM_PUMPS; i++) trimTotal += liveMl(i) - trimStartMl[i];
  float budgetLeft = max(0.0f, trimBudget - trimTotal);

  for (int i = 0; i < NUM_PUMPS; i++) {
    Pid& p = pid[i];
    float y = chroma[i];
    float e = tc[i] - y;
    float added = liveMl(i) - trimStartMl[i];
    float lo = added, hi = added + budgetLeft;

    float dy = p.first ? 0 : (y - p.prevY) / dt;     // derivative on measurement
    p.first = false;
    p.prevY = y;
    p.dF += 0.2f * (dy - p.dF);

    float P = p.kp * e, D = -p.kd * p.dF;
    float Inew = p.I + p.ki * e * dt;
    float u = P + Inew + D;
    bool windHigh = u > hi && e > 0, windLow = u < lo && e < 0;
    if (!windHigh && !windLow) p.I = Inew;           // anti-windup (conditional integration)
    uCmd[i] = constrain(P + p.I + D, lo, hi);
  }

  // pilih pompa yang paling kurang, tuang sedikit
  if (activePump < 0 && budgetLeft > MIN_PULSE_ML) {
    int best = -1; float bestPending = MIN_PULSE_ML;
    for (int i = 0; i < NUM_PUMPS; i++) {
      float pending = uCmd[i] - (liveMl(i) - trimStartMl[i]);
      if (pending > bestPending) { bestPending = pending; best = i; }
    }
    if (best >= 0) pumpStart(best, min(bestPending, MAX_PULSE_ML));
  }

  lastErr = maxError();
  unsigned long now = millis();

  if (lastErr < TOL && activePump < 0) {
    if (!holdSince) holdSince = now;
    if (now - holdSince >= HOLD_MS) return finish("ok");
  } else holdSince = 0;

  if (budgetLeft <= MIN_PULSE_ML && activePump < 0) {
    if (!budgetSince) budgetSince = now;
    if (now - budgetSince >= HOLD_MS) return finish("budget");
  }

  if (now - phaseStart > TRIM_TIMEOUT_MS) return finish("timeout");
  if (now - lastTele >= TELE_MS) publishTelemetry();
}

void handleOrder(JsonDocument& doc) {
  if (busy()) { Serial.println("Busy, order ignored"); return; }
  JsonObject dose = doc["dose"];
  if (dose.isNull()) { publishStatus("error", "order tanpa dose"); return; }

  orderId = (const char*)(doc["id"] | "");
  targetRgb[0] = doc["r"] | 0; targetRgb[1] = doc["g"] | 0; targetRgb[2] = doc["b"] | 0;
  float sum = targetRgb[0] + targetRgb[1] + targetRgb[2];
  trimEnabled = sum > 0 && sensorOk;
  float frac = trimEnabled ? BASE_FRACTION : 1.0f;

  volTarget = 0;
  for (int i = 0; i < NUM_PUMPS; i++) {
    float ml = dose[PUMP_KEYS[i]] | 0.0f;
    volTarget += ml;
    baseMl[i] = ml * frac;
    doneMl[i] = 0;
    tc[i] = sum > 0 ? targetRgb[i] / sum : 1 / 3.f;
  }
  if (!sensorOk) Serial.println("PERINGATAN: sensor warna tidak terbaca, mode open-loop");

  setLed(targetRgb[0], targetRgb[1], targetRgb[2]);
  Serial.printf("Order %s RGB(%d,%d,%d) %.0f ml, chroma target %.3f/%.3f/%.3f, trim=%d\n",
                orderId.c_str(), targetRgb[0], targetRgb[1], targetRgb[2], volTarget,
                tc[0], tc[1], tc[2], trimEnabled);
  baseIdx = -1;
  phaseStart = millis();
  state = BASE;
  nextBasePump();
}

// ─────────────────────────── Cohen-Coon autotune ───────────────────────────
void startTune(int p, float ml, float vol, unsigned long ms) {
  if (busy()) { publishTuneError("mixer sedang sibuk"); return; }
  if (!sensorOk) { publishTuneError("sensor warna tidak terbaca"); return; }
  tunePump = p; tuneMl = ml; tuneVol = vol; tuneMs = ms;
  tuneN = 0; tuneBaseN = 0; tuneY0 = 0;
  for (int i = 0; i < NUM_PUMPS; i++) doneMl[i] = 0;
  orderId = "";
  relay(STIR_PIN, true);
  phaseStart = millis();
  state = TUNE_BASE;
  Serial.printf("TUNE %s: step %.1f ml, volume gelas %.0f ml, rekam %lu ms\n", PUMP_KEYS[p], ml, vol, ms);
  publishStatus();
}

void analyzeTune() {
  relay(STIR_PIN, false);
  state = IDLE;
  int n = tuneN;
  if (n < 20) { publishTuneError("sampel terlalu sedikit"); publishStatus(); return; }

  static float sm[TUNE_MAX];                 // moving average 5 titik
  for (int k = 0; k < n; k++) {
    float s = 0; int c = 0;
    for (int j = max(0, k - 2); j <= min(n - 1, k + 2); j++) { s += tuneY[j]; c++; }
    sm[k] = s / c;
  }
  float yss = 0; int tail = n * 3 / 4;
  for (int k = tail; k < n; k++) yss += sm[k];
  yss /= (n - tail);
  float dy = yss - tuneY0;
  if (dy < TUNE_MIN_DY) { publishTuneError("respons terlalu kecil, perbesar ml step"); publishStatus(); return; }

  float t28 = -1, t63 = -1;
  for (int k = 0; k < n; k++) {
    if (t28 < 0 && sm[k] >= tuneY0 + 0.283f * dy) t28 = tuneT[k];
    if (t63 < 0 && sm[k] >= tuneY0 + 0.632f * dy) { t63 = tuneT[k]; break; }
  }
  if (t28 < 0 || t63 < 0) { publishTuneError("titik 28%/63% tidak ditemukan"); publishStatus(); return; }

  float tau = max(1.5f * (t63 - t28), 0.1f);
  float theta = max(t63 - tau, 0.1f);
  float K = dy / tuneMl;
  model[tunePump] = { K * tuneVol, tau, theta };
  saveModel(tunePump);

  float kp, Ti, Td;
  cohenCoon(K, tau, theta, kp, Ti, Td);
  Serial.printf("TUNE %s: K=%.5f tau=%.2f theta=%.2f -> Kp=%.1f Ti=%.2f Td=%.2f\n",
                PUMP_KEYS[tunePump], K, tau, theta, kp, Ti, Td);

  JsonDocument doc;
  doc["type"] = "tune"; doc["ok"] = true; doc["pump"] = PUMP_KEYS[tunePump];
  doc["ml"] = tuneMl; doc["vol"] = tuneVol;
  doc["y0"] = tuneY0; doc["dy"] = dy; doc["t28"] = t28; doc["t63"] = t63;
  doc["K"] = K; doc["Kv"] = K * tuneVol; doc["tau"] = tau; doc["theta"] = theta;
  doc["kp"] = kp; doc["Ti"] = Ti; doc["Td"] = Td; doc["ki"] = kp / Ti; doc["kd"] = kp * Td;
  doc["detune"] = DETUNE;
  int step = (n + 99) / 100;                 // kurva max 100 titik untuk laporan
  doc["curve_dt"] = tuneT[min(step, n - 1)] - tuneT[0];
  JsonArray cv = doc["curve"].to<JsonArray>();
  for (int k = 0; k < n; k += step) cv.add(roundf(sm[k] * 10000) / 10000);
  String out;
  serializeJson(doc, out);
  mqtt.publish(T_TUNE, out.c_str(), false);
  publishStatus();
}

// ─────────────────────────── MQTT ───────────────────────────
void handleCmd(JsonDocument& doc) {
  const char* cmd = doc["cmd"] | "";
  int p = pumpIndex(doc["pump"] | "");

  if (!strcmp(cmd, "stop")) { stopJob("stopped by command"); return; }
  if (!strcmp(cmd, "info")) { publishInfo(); return; }
  if (busy()) { Serial.printf("Cmd %s ditolak: sibuk\n", cmd); return; }

  if (!strcmp(cmd, "calwhite") || !strcmp(cmd, "calblack")) {
    float avg[3];
    averageRaw(avg, 20);
    memcpy(!strcmp(cmd, "calwhite") ? calWhite : calBlack, avg, sizeof avg);
    saveCal();
    sensorFirst = true;
    Serial.printf("%s = %.0f %.0f %.0f\n", cmd, avg[0], avg[1], avg[2]);
    publishInfo();
  } else if (!strcmp(cmd, "tune") && p >= 0) {
    startTune(p, doc["ml"] | 5.0f, doc["vol"] | 250.0f, doc["ms"] | 20000UL);
  } else if (!strcmp(cmd, "model") && p >= 0) {
    float vol = doc["vol"] | 250.0f;
    model[p].Kv    = doc["Kv"] | ((doc["K"] | 0.0f) * vol);
    model[p].tau   = doc["tau"] | model[p].tau;
    model[p].theta = doc["theta"] | model[p].theta;
    if (model[p].Kv <= 0) model[p].Kv = DEF_KV;
    saveModel(p);
    publishInfo();
  } else if (!strcmp(cmd, "prime") && p >= 0) {
    unsigned long ms = doc["ms"] | 3000UL;
    for (int i = 0; i < NUM_PUMPS; i++) doneMl[i] = 0;
    pumpStart(p, ms / 1000.0f * ML_PER_SEC[p]);
    state = PRIMING;
    publishStatus();
  } else {
    Serial.printf("Cmd tidak dikenal: %s\n", cmd);
  }
}

void onMessage(char* topic, byte* payload, unsigned int len) {
  JsonDocument doc;
  if (deserializeJson(doc, payload, len)) { Serial.println("Invalid JSON"); return; }
  if (!strcmp(topic, T_ORDER)) handleOrder(doc);
  else if (!strcmp(topic, T_CMD)) handleCmd(doc);
}

void mqttReconnect() {
  if (mqtt.connected() || millis() - lastMqttTry < 2000) return;   // non-blocking
  lastMqttTry = millis();
  static const char* LWT = "{\"device\":\"esp32s3-mixer\",\"state\":\"offline\"}";
  Serial.printf("MQTT connecting to %s:%d...\n", MQTT_HOST, MQTT_PORT);
  if (mqtt.connect(DEVICE_ID, nullptr, nullptr, T_STATUS, 1, true, LWT)) {
    mqtt.subscribe(T_ORDER, 1);
    mqtt.subscribe(T_CMD, 1);
    Serial.println("MQTT OK");
    publishStatus();
  } else {
    Serial.printf("MQTT failed rc=%d\n", mqtt.state());
  }
}

// ─────────────────────────── main ───────────────────────────
void setup() {
  Serial.begin(115200);
  delay(300);
  for (int i = 0; i < NUM_PUMPS; i++) { pinMode(PUMP_PINS[i], OUTPUT); relay(PUMP_PINS[i], false); }
  pinMode(STIR_PIN, OUTPUT);
  relay(STIR_PIN, false);

  LED_SETUP(LED_R, 0);
  LED_SETUP(LED_G, 1);
  LED_SETUP(LED_B, 2);
  setLed(0, 0, 0);

  loadSettings();

#if USE_TCS34725
  Wire.begin(I2C_SDA, I2C_SCL);
  if (!tcs.begin(TCS34725_ADDRESS, &Wire)) Serial.println("TCS34725 TIDAK terdeteksi, cek kabel I2C");
#else
  pinMode(TCS_S0, OUTPUT); pinMode(TCS_S1, OUTPUT);
  pinMode(TCS_S2, OUTPUT); pinMode(TCS_S3, OUTPUT);
  pinMode(TCS_OUT, INPUT);
  digitalWrite(TCS_S0, HIGH); digitalWrite(TCS_S1, LOW);   // skala frekuensi 20%
#endif

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.printf("WiFi connecting to %s", WIFI_SSID);
  for (int i = 0; i < 40 && WiFi.status() != WL_CONNECTED; i++) { delay(500); Serial.print("."); }
  Serial.printf("\nWiFi %s, IP %s\n", WiFi.status() == WL_CONNECTED ? "OK" : "belum", WiFi.localIP().toString().c_str());

  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setBufferSize(2048);     // hasil tune berisi kurva respons
  mqtt.setCallback(onMessage);
}

void loop() {
  unsigned long now = millis();

  if (WiFi.status() != WL_CONNECTED) {
    if (busy()) stopJob("wifi lost");                // jangan memompa tanpa pengawasan
  } else {
    mqttReconnect();
    mqtt.loop();
  }

  sampleSensor();
  pumpUpdate();

  float dt = (lastSample - lastSampleUsed) / 1000.0f;
  bool tick = newSample;
  if (tick) { newSample = false; lastSampleUsed = lastSample; }

  switch (state) {
    case BASE:
      if (activePump < 0) nextBasePump();
      break;
    case PREMIX:
      if (now - phaseStart >= PREMIX_MS) {
        if (trimEnabled) startTrim();
        else { lastErr = maxError(); finish("open-loop"); }
      }
      break;
    case TRIM:
      if (!sensorOk) { stopJob("sensor warna hilang saat trim"); break; }
      if (tick && dt > 0) trimStep(dt);
      break;
    case PRIMING:
      if (activePump < 0) { state = IDLE; publishStatus(); }
      break;
    case TUNE_BASE:
      if (tick) { tuneY0 += chromaRaw[tunePump]; tuneBaseN++; }
      if (now - phaseStart >= TUNE_BASE_MS && tuneBaseN > 0) {
        tuneY0 /= tuneBaseN;
        pumpStart(tunePump, tuneMl);                 // STEP input
        tuneT0 = millis();
        state = TUNE_RUN;
      }
      break;
    case TUNE_RUN:
      if (tick && tuneN < TUNE_MAX) {
        tuneY[tuneN] = chromaRaw[tunePump];
        tuneT[tuneN] = (millis() - tuneT0) / 1000.0f;
        tuneN++;
      }
      if (millis() - tuneT0 >= tuneMs || tuneN >= TUNE_MAX) { pumpAbort(); analyzeTune(); }
      break;
    case DONE:
      if (now - doneAt >= 5000) { state = IDLE; orderId = ""; publishStatus(); }
      break;
    default: break;
  }

  if (mqtt.connected() && millis() - lastPub >= (busy() ? 1000UL : 10000UL)) publishStatus();
}
