#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include "DHT.h"
#include <PubSubClient.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ---------- Pin Mapping ----------
#define OLED32_SDA_PIN 25
#define OLED32_SCL_PIN 26
#define OLED64_SDA_PIN 23
#define OLED64_SCL_PIN 22

#define SERVO_PIN  4
#define LED_PIN    17
#define BUZZER_PIN 16
#define SOIL_PIN   32   // ADC1 pin
#define LDR_PIN    33   // digital input (with INPUT_PULLUP)
#define RELAY_PIN  27   // relay control (active LOW)
#define DHTPIN     13
#define DHTTYPE    DHT22
#define USE_WIFI   true

#define SERVO_LEDC_CHANNEL  1   // channel ledc terpisah dari buzzer
#define SERVO_LEDC_TIMER    0
#define SERVO_FREQ_HZ       50  // 50Hz for servo
#define SERVO_RES_BITS      16

// ---------- Display config ----------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT32 32
#define SCREEN_HEIGHT64 64
#define OLED32_RESET -1
#define OLED64_RESET -1
#define OLED_ADDR 0x3C

// ---------- Variables / Configs ----------
const char* WIFI_SSID = ARG_WIFI_SSID;
const char* WIFI_PASS = ARG_WIFI_PASS;
const unsigned long WIFI_RETRY_INTERVAL_MS = 5000UL;
unsigned long _lastWifiAttempt = 0;
bool wifiWanted = USE_WIFI;

const char* OTA_HOST = ARG_OTA_HOST;
const char* OTA_PASS = ARG_OTA_PASS;

const char* TB_HOST = "demo.thingsboard.io";
const int   TB_PORT = 1883;
const char* TB_ACCESS_TOKEN = ARG_TB_ACCESS_TOKEN;
const char* TB_TOPIC = "v1/devices/me/telemetry";

unsigned long _lastMqttAttempt = 0;
const unsigned long MQTT_RETRY_INTERVAL_MS = 5000;

const uint8_t BUZZER_LEDC_CHANNEL = 2;
const uint8_t BUZZER_LEDC_TIMER   = 1;
const uint32_t BUZZER_FREQ = 2000; // Hz, untuk buzzer pasif
const uint8_t BUZZER_DUTY = 128;   // 0-255 (8-bit)
const unsigned long PUMP_BUZZER_TIMEOUT_MS = 1UL * 60UL * 1000UL; // 1 menit

static unsigned long relayOnSince = 0;
static bool _buzzerActive = false;

const int SERVO_OPEN_ANGLE = 90;    // derajat saat "buka"
const int SERVO_CLOSED_ANGLE = 0;   // derajat saat "tutup"
const int SERVO_MIN_PULSE_US = 500;
const int SERVO_MAX_PULSE_US = 2500;
static int _currentServoAngle = SERVO_CLOSED_ANGLE;
static uint32_t angleToPulseUs(int angle) {
  // Konversi angle (0..180) -> pulse width microseconds (SERVO_MIN_PULSE_US .. SERVO_MAX_PULSE_US)
  if (angle < 0) angle = 0;
  if (angle > 180) angle = 180;
  return map(angle, 0, 180, SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US);
}
static uint32_t pulseUsToDuty(uint32_t pulse_us) {
  // Convert microsecond pulse to ledc duty value (resolution bits)
  const uint32_t period_us = 1000000UL / SERVO_FREQ_HZ; // 20,000 us for 50Hz
  const uint32_t maxDuty = ((1UL << SERVO_RES_BITS) - 1);
  // duty = pulse_us / period_us * maxDuty
  uint64_t duty = (uint64_t)pulse_us * maxDuty / period_us;
  if (duty > maxDuty) duty = maxDuty;
  return (uint32_t)duty;
}

// ---------- I2C / Displays (TwoWire) ----------
TwoWire I2C_32 = TwoWire(0);
TwoWire I2C_64 = TwoWire(1);
Adafruit_SSD1306 display32(SCREEN_WIDTH, SCREEN_HEIGHT32, &I2C_32, OLED32_RESET);
Adafruit_SSD1306 display64(SCREEN_WIDTH, SCREEN_HEIGHT64, &I2C_64, OLED64_RESET);

// ---------- Objects ----------
WiFiClient espClient;
PubSubClient mqttClient(espClient);
DHT dht(DHTPIN, DHTTYPE);

// ---------- Sensor struct ----------
struct SensorData {
  int soilRaw;
  int soilPercent;
  int ldrRaw;
  bool ldrBright;
  float temperature;
  float humidity;
  bool dhtOk;
  bool relayOn;
};

// ---------- Utility: Send log to ThingsBoard & Serial ----------
void tbLog(const String &level, const String &message) {
  Serial.printf("[%s] %s\n", level.c_str(), message.c_str());
  if (!mqttClient.connected()) return;
  String payload = "{\"log\":\"[" + level + "] " + message + "\"}";
  mqttClient.publish(TB_TOPIC, payload.c_str());
}
String buildTelemetryPayload(const SensorData &data) {
  bool soilAlarm = (data.soilPercent <= 20);
  bool tempAlarm = (data.dhtOk && data.temperature >= 35.0);

  if (soilAlarm) tbLog("WARN", "Soil moisture critically low (" + String(data.soilPercent) + "%)");
  if (tempAlarm) tbLog("WARN", "High temperature detected (" + String(data.temperature, 1) + "°C)");

  String payload = "{";
  payload += "\"soilPercent\":" + String(data.soilPercent);
  payload += ",\"soilRaw\":" + String(data.soilRaw);
  payload += ",\"ldrBright\":" + String(data.ldrBright ? 1 : 0);
  if (data.dhtOk) {
    payload += ",\"temperature\":" + String(data.temperature, 1);
    payload += ",\"humidity\":" + String(data.humidity, 1);
  }
  payload += ",\"relayOn\":" + String(data.relayOn ? 1 : 0);
  payload += ",\"soilAlarm\":" + String(soilAlarm ? "true" : "false");
  payload += ",\"tempAlarm\":" + String(tempAlarm ? "true" : "false");
  payload += "}";
  return payload;
}

// ---------- Simple icons for 64x128 (draw primitives) ----------
void drawTempIcon(int x, int y) {
  display64.drawRoundRect(x, y, 12, 18, 3, SSD1306_WHITE);
  display64.fillRect(x+4, y+12, 4, 4, SSD1306_WHITE);
}
void drawHumIcon(int x, int y) {
  display64.fillTriangle(x+6, y, x+2, y+10, x+10, y+10, SSD1306_WHITE);
  display64.fillCircle(x+6, y+12, 2, SSD1306_WHITE);
}
void drawSoilIcon(int x, int y) {
  display64.fillRect(x, y+8, 12, 4, SSD1306_WHITE);
  display64.fillCircle(x+3, y+6, 2, SSD1306_WHITE);
  display64.fillCircle(x+9, y+6, 2, SSD1306_WHITE);
}
void drawLdrIcon(int x, int y, bool bright) {
  if (bright) display64.fillCircle(x+6, y+6, 5, SSD1306_WHITE);
  else {
    display64.drawCircle(x+6, y+6, 5, SSD1306_WHITE);
    display64.drawCircle(x+6, y+6, 2, SSD1306_WHITE);
  }
}
void drawRelayIcon(int x, int y, bool on) {
  display64.drawRect(x, y, 14, 10, SSD1306_WHITE);
  if (on) display64.fillRect(x+3, y+3, 8, 4, SSD1306_WHITE);
  else display64.drawLine(x+2, y+6, x+12, y+6, SSD1306_WHITE);
}

// ---------- Sensors Functions ----------
void readSoilSensor(SensorData &data) {
  data.soilRaw = analogRead(SOIL_PIN);
  data.soilPercent = map(data.soilRaw, 1600, 3800, 100, 0);
  data.soilPercent = constrain(data.soilPercent, 0, 100);
}
void readLdrSensor(SensorData &data) {
  data.ldrRaw = digitalRead(LDR_PIN);
  data.ldrBright = (data.ldrRaw == 0); // aktif LOW
}
void readDhtSensor(SensorData &data) {
  data.humidity = dht.readHumidity();
  data.temperature = dht.readTemperature();
  data.dhtOk = !(isnan(data.humidity) || isnan(data.temperature));
}

// ---------- Relay / LED / Buzzer / Servo ----------
void updateLed(const SensorData &data) {
  digitalWrite(LED_PIN, data.ldrBright ? LOW : HIGH); // LED on when dark (or invert as you like)
}
void startBuzzer() {
  if (!_buzzerActive) {
    _buzzerActive = true;
    tbLog("WARN", "Buzzer pattern activated (pump ON > timeout)");
  }
}
void updateBuzzerPattern() {
  static unsigned long lastToggle = 0;
  static int beepCount = 0;
  static bool buzzerOn = false;

  if (!_buzzerActive) {
    ledcWrite(BUZZER_LEDC_CHANNEL, 0);
    beepCount = 0;
    buzzerOn = false;
    return;
  }

  unsigned long now = millis();

  if (beepCount < 6) {  // 3 on-off pairs
    if (now - lastToggle >= 200) {
      buzzerOn = !buzzerOn;
      ledcWrite(BUZZER_LEDC_CHANNEL, buzzerOn ? BUZZER_DUTY : 0);
      lastToggle = now;
      beepCount++;
    }
  } else {
    if (now - lastToggle >= 15000) {
      beepCount = 0;
      lastToggle = now;
    } else {
      ledcWrite(BUZZER_LEDC_CHANNEL, 0);
    }
  }
}
void stopBuzzer() {
  if (_buzzerActive) {
    ledcWrite(BUZZER_LEDC_CHANNEL, 0);
    _buzzerActive = false;
    tbLog("INFO", "Buzzer stopped");
  }
}
void updateBuzzer(const SensorData &data) {
  unsigned long now = millis();

  if (data.relayOn) {
    if (relayOnSince == 0) relayOnSince = now;
    if ((now - relayOnSince) >= PUMP_BUZZER_TIMEOUT_MS) startBuzzer();
  } else {
    if (relayOnSince != 0) relayOnSince = 0;
    stopBuzzer();
  }
}
void servoInit() {
  // Servo: 50 Hz
  ledcSetup(SERVO_LEDC_CHANNEL, SERVO_FREQ_HZ, SERVO_RES_BITS);
  ledcAttachPin(SERVO_PIN, SERVO_LEDC_CHANNEL);
  // set posisi awal ke closed
  uint32_t pulse = angleToPulseUs(SERVO_CLOSED_ANGLE);
  ledcWrite(SERVO_LEDC_CHANNEL, pulseUsToDuty(pulse));
  _currentServoAngle = SERVO_CLOSED_ANGLE;
  Serial.println("[SERVO] init done - set to closed");
}
void servoWriteAngle(int angle) {
  angle = constrain(angle, 0, 180);
  uint32_t pulse = angleToPulseUs(angle);
  uint32_t duty = pulseUsToDuty(pulse);
  ledcWrite(SERVO_LEDC_CHANNEL, duty);
  _currentServoAngle = angle;
  Serial.printf("[SERVO] moved to %d deg (pulse=%u us, duty=%u)\n", angle, (unsigned)pulse, (unsigned)duty);
}
void servoMoveOpen()  { servoWriteAngle(SERVO_OPEN_ANGLE); }
void servoMoveClose() { servoWriteAngle(SERVO_CLOSED_ANGLE); }
void updateRelayState(SensorData &data) {
  const int DRY_THRESHOLD = 20; // persen
  const int WET_THRESHOLD = 60;
  bool shouldOn = false;

  if (data.soilPercent <= DRY_THRESHOLD) {
    shouldOn = true;
  } else if (data.soilPercent >= WET_THRESHOLD) {
    shouldOn = false;
  } else {
    // tetap pada kondisi terakhir
    shouldOn = data.relayOn;
  }

  // Jika status berubah, update relay dan kirim log ke ThingsBoard
  if (shouldOn != data.relayOn) {
    data.relayOn = shouldOn;
    digitalWrite(RELAY_PIN, shouldOn ? LOW : HIGH); // active LOW

    if (shouldOn) {
      tbLog("INFO", "Pump turned ON (soil dry)");
      // gerakkan servo -> buka kran (90°)
      servoMoveOpen();
    } else {
      tbLog("INFO", "Pump turned OFF (soil wet)");
      // gerakkan servo -> tutup kran (0°)
      servoMoveClose();
    }
  }
}

// ---------- OLED: init & render ----------
void oledInitDual() {
  // init two I2C busses at 400kHz
  I2C_32.begin(OLED32_SDA_PIN, OLED32_SCL_PIN, 400000UL);
  I2C_64.begin(OLED64_SDA_PIN, OLED64_SCL_PIN, 400000UL);

  if (!display32.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println(F("ERROR: display32 init failed. Check wiring/address."));
    while (true) delay(1000);
  }
  if (!display64.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println(F("ERROR: display64 init failed. Check wiring/address."));
    while (true) delay(1000);
  }

  display32.clearDisplay();
  display32.setTextWrap(false);
  display32.setTextColor(SSD1306_WHITE);

  display64.clearDisplay();
  display64.setTextWrap(false);
  display64.setTextColor(SSD1306_WHITE);
}
void oledShowStatic32() {
  display32.clearDisplay();
  display32.setTextWrap(false);
  auto drawCentered = [&](const char* txt, int y, uint8_t textSize) {
    size_t len = strlen(txt);                 // panjang string (ASCII)
    int charWidth = 6 * textSize;             // 5 px font + 1 px spacing -> 6 px per char, skalakan dengan textSize
    int textWidth = (int)len * charWidth;
    int x = (SCREEN_WIDTH - textWidth) / 2;
    if (x < 0) x = 0;                         // safety
    display32.setTextSize(textSize);
    display32.setCursor(x, y);
    display32.println(txt);
  };

  // Baris 1: sedikit lebih besar supaya menonjol
  drawCentered("BINUS IoT (Group 5)", 0, 1);   // y = 0, size = 1

  // Baris 2: main title
  drawCentered("Smart Greenhouse", 12, 1);     // y = 12, size = 1

  // Baris 3: nama anggota (jika panjang, masih center; kalau sangat panjang, pertimbangkan size 1)
  drawCentered("Imam / Josh / Marchel", 24, 1); // y = 24, size = 1

  display32.display();
}
void oledShowSensor64(const SensorData &sd) {
  display64.clearDisplay();

  // Temperature (top-left)
  drawTempIcon(2, 2);
  display64.setTextSize(2);
  display64.setCursor(22, 0);
  if (sd.dhtOk) display64.print(String(sd.temperature, 1) + "C"); else display64.print("--C");

  // Humidity (top-right)
  drawHumIcon(86, 2);
  display64.setTextSize(1);
  display64.setCursor(100, 6);
  if (sd.dhtOk) display64.printf("%d%%", (int)sd.humidity); else display64.print("--%");

  drawSoilIcon(2, 25);
  display64.setTextSize(1);
  display64.setCursor(18, 30);
  display64.print(sd.soilPercent);
  display64.print("%");

  drawLdrIcon(40, 25, sd.ldrBright);
  display64.setCursor(55, 30);
  display64.print(sd.ldrBright ? "OFF" : "ON");

  drawRelayIcon(76, 27, sd.relayOn);
  display64.setTextSize(1);
  display64.setCursor(94, 30);
  display64.print(sd.relayOn ? "ON" : "OFF");

  display64.setCursor(20, 50);
  display64.print(WiFi.localIP());

  display64.display();
}

// ---------- Connections Functions ----------
void wifiConnectOnce() {
  if (!wifiWanted) {
    if (WiFi.getMode() != WIFI_MODE_NULL) {
      WiFi.disconnect(true, true);
      WiFi.mode(WIFI_OFF);
      Serial.println("[WiFi] Disabled (wifiWanted=false)");
    }
    return;
  }

  if (WiFi.status() == WL_CONNECTED) return;

  unsigned long now = millis();
  if (now - _lastWifiAttempt < WIFI_RETRY_INTERVAL_MS) return;
  _lastWifiAttempt = now;

  Serial.print("[WiFi] Attempt connect to ");
  Serial.println(WIFI_SSID);

  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true, true);
  delay(100);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  unsigned long waitUntil = millis() + 5000;
  while (millis() < waitUntil) {
    if (WiFi.status() == WL_CONNECTED) break;
    delay(10);
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("[WiFi] Connected, IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.print("[WiFi] Not connected yet (status=");
    Serial.print(WiFi.status());
    Serial.println(") - will retry later");
  }
}
void wifiSetEnabled(bool enable) {
  wifiWanted = enable;
  if (!enable) {
    Serial.println("[WiFi] Requested disable");
    WiFi.disconnect(true, true);
    WiFi.mode(WIFI_OFF);
  } else {
    Serial.println("[WiFi] Requested enable");
  }
}
void mqttConnectOnce() {
  if (!USE_WIFI || WiFi.status() != WL_CONNECTED) return;
  if (mqttClient.connected()) return;

  unsigned long now = millis();
  if (now - _lastMqttAttempt < MQTT_RETRY_INTERVAL_MS) return;
  _lastMqttAttempt = now;

  Serial.print("[MQTT] Connecting to ThingsBoard... ");
  mqttClient.setServer(TB_HOST, TB_PORT);

  String clientId = "ESP32-" + WiFi.macAddress();
  bool ok = mqttClient.connect(clientId.c_str(), TB_ACCESS_TOKEN, NULL);
  if (ok) Serial.println("connected ✅");
  else {
    Serial.print("failed, rc=");
    Serial.println(mqttClient.state());
  }
}
void mqttPublishTelemetry(const SensorData &data) {
  if (!mqttClient.connected()) {
    mqttConnectOnce();
    if (!mqttClient.connected()) {
      Serial.println("[MQTT] Skipped publish (no connection)");
      return;
    }
  }

  String payload = buildTelemetryPayload(data);
  bool ok = mqttClient.publish(TB_TOPIC, payload.c_str());

  if (ok) {
    Serial.print("[MQTT] Published: ");
    Serial.println(payload);
  } else {
    Serial.println("[MQTT] Publish failed ⚠️");
  }
}

// ---------- Print ----------
void printSensorData(const SensorData &data) {
  Serial.println("--------------------------------------------------");
  Serial.print("🌱 Soil: "); Serial.print(data.soilPercent);
  Serial.print("% (raw: "); Serial.print(data.soilRaw); Serial.println(")");
  Serial.print("☀️ LDR: "); Serial.print(data.ldrBright ? "Bright" : "Dark");
  Serial.print(" (raw: "); Serial.print(data.ldrRaw); Serial.println(")");
  Serial.print("🌡️ Temp: ");
  if (data.dhtOk) Serial.print(data.temperature, 1);
  else Serial.print("--");
  Serial.print("°C | 💧 Humidity: ");
  if (data.dhtOk) Serial.print(data.humidity, 1);
  else Serial.print("--");
  Serial.println("%");
  Serial.print("🔌 Relay: ");
  Serial.println(data.relayOn ? "ON" : "OFF");
  Serial.println("--------------------------------------------------");
}

// ---------- OTA ----------
void setupOTA() {
  ArduinoOTA.setHostname(OTA_HOST);
  ArduinoOTA.setPassword(OTA_PASS);

  ArduinoOTA
    .onStart([]() {
      String type = (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem";
      Serial.println("[OTA] Start updating " + type);
      tbLog("INFO", "OTA update started");
    })
    .onEnd([]() {
      Serial.println("\n[OTA] Update complete ✅");
      tbLog("INFO", "OTA update complete");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("[OTA] Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("\n[OTA] Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
      tbLog("ERROR", "OTA update failed");
    });

  ArduinoOTA.begin();
  Serial.println(OTA_HOST);
  Serial.println("[OTA] Ready - waiting for upload...");
}

// ---------- Main ----------
void setup() {
  Serial.begin(115200);
  delay(200);

  // optional: stop bluetooth if needed
  // btStop();

  Serial.println("[INFO] Starting Wi-Fi connection...");

  // try connect briefly for OTA
  unsigned long startWait = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - startWait < 15000UL)) {
    wifiConnectOnce();
    delay(500);
  }

  if (WiFi.status() == WL_CONNECTED) {
    setupOTA();
  } else {
    Serial.println("[WiFi] Not connected yet; OTA disabled until connected");
  }

  // pin setup
  pinMode(LDR_PIN, INPUT_PULLUP);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, HIGH); // relay off (active LOW)

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Buzzer: 2000 Hz
  ledcSetup(BUZZER_LEDC_CHANNEL, BUZZER_FREQ, 8); // 8-bit resolution
  ledcAttachPin(BUZZER_PIN, BUZZER_LEDC_CHANNEL);
  ledcWrite(BUZZER_LEDC_CHANNEL, 0); // awalnya mati
  servoInit();

  dht.begin();

  analogReadResolution(12);
  analogSetPinAttenuation(SOIL_PIN, ADC_11db);

  // init displays
  oledInitDual();
  oledShowStatic32();
  // show initial empty data on 64x128
  SensorData initData = {4095, 0, 1, true, 0.0, 0.0, false, false};
  oledShowSensor64(initData);

  Serial.println("=== Modular Sensor Framework Initialized ===");
  tbLog("INFO", "Device boot complete - sensors initialized");
}

void loop() {
  static SensorData data;
  static unsigned long lastPublish = 0;
  const unsigned long PUBLISH_INTERVAL = 5000UL;

  wifiConnectOnce();
  mqttConnectOnce();
  mqttClient.loop();
  if (WiFi.status() == WL_CONNECTED) ArduinoOTA.handle();

  readSoilSensor(data);
  readLdrSensor(data);
  readDhtSensor(data);

  updateRelayState(data);
  updateLed(data);
  updateBuzzer(data);
  updateBuzzerPattern();

  printSensorData(data);

  unsigned long now = millis();
  if (now - lastPublish >= PUBLISH_INTERVAL) {
    lastPublish = now;
    mqttPublishTelemetry(data);
    oledShowSensor64(data); // update big display
    // oled32 shows static message
  }

  delay(200);
}
