/*
 * ColorMix – ESP32 Paint Mixer
 *
 * Subscribe : colormix/order   {"id","name","r","g","b","hex","vol","dose":{"red","green","blue"}}  (dose in ml)
 *             colormix/cmd     {"cmd":"stop"}
 * Publish   : colormix/status  {"device","state","order","progress","step","rssi"}   (retained)
 *             state = idle | mixing | done | error | offline (LWT)
 *
 * Hardware (change pins below to match your wiring):
 *   - 3 pumps via relay module: base paint Red, Green, Blue
 *   - 1 stirrer motor via relay (optional)
 *   - 1 RGB LED (common cathode) showing the target colour
 *
 * Board  : ESP32 Dev Module, Arduino-ESP32 core 2.x or 3.x
 * (For ESP32-S3 with colour-sensor PID control, use ../ColorMixerS3/ColorMixerS3.ino)
 * Library: PubSubClient (Nick O'Leary), ArduinoJson 7.x (Benoit Blanchon)
 */

#include <Arduino.h>
#include <esp_arduino_version.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// LEDC API differs between core 2.x and 3.x
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  #define LED_SETUP(pin, ch) ledcAttach(pin, 5000, 8)
  #define LED_WRITE(pin, ch, v) ledcWrite(pin, v)
#else
  #define LED_SETUP(pin, ch) do { ledcSetup(ch, 5000, 8); ledcAttachPin(pin, ch); } while (0)
  #define LED_WRITE(pin, ch, v) ledcWrite(ch, v)
#endif

// ================== CONFIG ==================
const char* WIFI_SSID = "WIFI_SSID";
const char* WIFI_PASS = "WIFI_PASSWORD";
const char* MQTT_HOST = "192.168.1.10";   // IP laptop yang menjalankan docker (ipconfig)
const uint16_t MQTT_PORT = 1883;
const char* DEVICE_ID = "esp32-mixer";

const char* T_ORDER  = "colormix/order";
const char* T_CMD    = "colormix/cmd";
const char* T_STATUS = "colormix/status";

// Pump order: Red, Green, Blue
const int   NUM_PUMPS = 3;
const char* PUMP_KEYS[NUM_PUMPS]  = { "red", "green", "blue" };
const int   PUMP_PINS[NUM_PUMPS]  = { 25, 26, 27 };
// Calibration: how many ml each pump delivers per second. Measure yours!
const float ML_PER_SEC[NUM_PUMPS] = { 5.0, 5.0, 5.0 };

const int  STIR_PIN = 19;                 // set -1 if no stirrer
const unsigned long STIR_MS = 5000;

const bool RELAY_ACTIVE_LOW = true;       // most cheap relay boards are active-LOW

const int LED_R = 16, LED_G = 17, LED_B = 18;
// ============================================

WiFiClient net;
PubSubClient mqtt(net);

enum State { IDLE, DISPENSING, STIRRING, DONE };
State state = IDLE;

String orderId = "";
unsigned long pumpMs[NUM_PUMPS];
int  stepIdx = 0;
unsigned long stepStart = 0, jobStart = 0, jobTotalMs = 0, doneAt = 0, lastPub = 0;

// ---------- hardware helpers ----------
void relay(int pin, bool on) {
  if (pin < 0) return;
  digitalWrite(pin, (on ^ RELAY_ACTIVE_LOW) ? HIGH : LOW);
}

void allOff() {
  for (int i = 0; i < NUM_PUMPS; i++) relay(PUMP_PINS[i], false);
  relay(STIR_PIN, false);
}

void setLed(uint8_t r, uint8_t g, uint8_t b) {
  LED_WRITE(LED_R, 0, r);
  LED_WRITE(LED_G, 1, g);
  LED_WRITE(LED_B, 2, b);
}

// ---------- status ----------
const char* stateName() {
  switch (state) {
    case DISPENSING:
    case STIRRING: return "mixing";
    case DONE:     return "done";
    default:       return "idle";
  }
}

const char* stepName() {
  if (state == DISPENSING) return PUMP_KEYS[stepIdx];
  if (state == STIRRING) return "stir";
  return "";
}

int progress() {
  if (state == DONE) return 100;
  if (state == IDLE || jobTotalMs == 0) return 0;
  unsigned long p = (millis() - jobStart) * 100UL / jobTotalMs;
  return p > 99 ? 99 : (int)p;
}

void publishStatus(const char* overrideState = nullptr, const char* error = nullptr) {
  JsonDocument doc;
  doc["device"]   = DEVICE_ID;
  doc["state"]    = overrideState ? overrideState : stateName();
  doc["order"]    = orderId;
  doc["progress"] = progress();
  doc["step"]     = stepName();
  doc["rssi"]     = WiFi.RSSI();
  if (error) doc["error"] = error;
  char buf[256];
  size_t n = serializeJson(doc, buf);
  mqtt.publish(T_STATUS, (const uint8_t*)buf, n, true);
  lastPub = millis();
}

// ---------- job control ----------
void startStep(int i) {
  // skip pumps with 0 ml
  while (i < NUM_PUMPS && pumpMs[i] == 0) i++;
  allOff();
  if (i < NUM_PUMPS) {
    state = DISPENSING;
    stepIdx = i;
    stepStart = millis();
    relay(PUMP_PINS[i], true);
    Serial.printf("Pump %s ON for %lu ms\n", PUMP_KEYS[i], pumpMs[i]);
  } else if (STIR_PIN >= 0) {
    state = STIRRING;
    stepStart = millis();
    relay(STIR_PIN, true);
    Serial.println("Stirring...");
  } else {
    state = DONE;
    doneAt = millis();
  }
  publishStatus();
}

void stopJob(const char* reason) {
  allOff();
  Serial.printf("Job stopped: %s\n", reason);
  publishStatus("error", reason);
  state = IDLE;
  orderId = "";
  jobTotalMs = 0;
  setLed(0, 0, 0);
}

void handleOrder(JsonDocument& doc) {
  if (state == DISPENSING || state == STIRRING) {
    Serial.println("Busy, order ignored");
    return;
  }
  JsonObject dose = doc["dose"];
  if (dose.isNull()) { publishStatus("error", "order tanpa dose"); return; }

  orderId = doc["id"] | "";
  jobTotalMs = 0;
  for (int i = 0; i < NUM_PUMPS; i++) {
    float ml = dose[PUMP_KEYS[i]] | 0.0f;
    pumpMs[i] = ml > 0 ? (unsigned long)(ml / ML_PER_SEC[i] * 1000.0f) : 0;
    jobTotalMs += pumpMs[i];
  }
  if (STIR_PIN >= 0) jobTotalMs += STIR_MS;

  setLed(doc["r"] | 0, doc["g"] | 0, doc["b"] | 0);
  Serial.printf("Order %s: %s RGB(%d,%d,%d) %d ml, total %lu ms\n",
                orderId.c_str(), (const char*)(doc["name"] | ""), (int)(doc["r"] | 0),
                (int)(doc["g"] | 0), (int)(doc["b"] | 0), (int)(doc["vol"] | 0), jobTotalMs);

  jobStart = millis();
  startStep(0);
}

void onMessage(char* topic, byte* payload, unsigned int len) {
  JsonDocument doc;
  if (deserializeJson(doc, payload, len)) {
    Serial.println("Invalid JSON");
    return;
  }
  if (strcmp(topic, T_ORDER) == 0) handleOrder(doc);
  else if (strcmp(topic, T_CMD) == 0 && strcmp(doc["cmd"] | "", "stop") == 0) stopJob("stopped by command");
}

// ---------- connectivity ----------
void connectWifi() {
  if (WiFi.status() == WL_CONNECTED) return;
  Serial.printf("WiFi connecting to %s", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    if (state != IDLE && state != DONE) stopJob("wifi lost");  // safety: never pump blind
  }
  Serial.printf("\nWiFi OK, IP %s\n", WiFi.localIP().toString().c_str());
}

void connectMqtt() {
  static const char* LWT = "{\"device\":\"esp32-mixer\",\"state\":\"offline\"}";
  while (!mqtt.connected()) {
    Serial.printf("MQTT connecting to %s:%d...\n", MQTT_HOST, MQTT_PORT);
    if (mqtt.connect(DEVICE_ID, nullptr, nullptr, T_STATUS, 1, true, LWT)) {
      mqtt.subscribe(T_ORDER, 1);
      mqtt.subscribe(T_CMD, 1);
      Serial.println("MQTT OK");
      publishStatus();
    } else {
      Serial.printf("MQTT failed rc=%d, retry in 2s\n", mqtt.state());
      delay(2000);
    }
  }
}

// ---------- main ----------
void setup() {
  Serial.begin(115200);
  for (int i = 0; i < NUM_PUMPS; i++) pinMode(PUMP_PINS[i], OUTPUT);
  if (STIR_PIN >= 0) pinMode(STIR_PIN, OUTPUT);
  allOff();

  LED_SETUP(LED_R, 0);
  LED_SETUP(LED_G, 1);
  LED_SETUP(LED_B, 2);
  setLed(0, 0, 0);

  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setBufferSize(512);      // default 256 is too small for order JSON
  mqtt.setCallback(onMessage);
  connectWifi();
}

void loop() {
  connectWifi();
  if (!mqtt.connected()) connectMqtt();
  mqtt.loop();

  unsigned long now = millis();
  if (state == DISPENSING && now - stepStart >= pumpMs[stepIdx]) {
    relay(PUMP_PINS[stepIdx], false);
    startStep(stepIdx + 1);
  } else if (state == STIRRING && now - stepStart >= STIR_MS) {
    relay(STIR_PIN, false);
    state = DONE;
    doneAt = now;
    Serial.println("Done!");
    publishStatus();
  } else if (state == DONE && now - doneAt >= 5000) {
    state = IDLE;
    orderId = "";
    jobTotalMs = 0;
    publishStatus();
  }

  // periodic status: every 1s while working, every 10s when idle (heartbeat + RSSI)
  unsigned long every = (state == DISPENSING || state == STIRRING) ? 1000 : 10000;
  if (now - lastPub >= every) publishStatus();
}
