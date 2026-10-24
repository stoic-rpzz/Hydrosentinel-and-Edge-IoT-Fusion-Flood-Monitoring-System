/*
  HydroSentinel Edge IoT Node  -  ESP32 (Arduino IDE)
  ---------------------------------------------------
  Sensors : JSN-SR04T / HC-SR04 ultrasonic (water level)
            BME280 (air temp, humidity, pressure)  - I2C
            Tipping-bucket rain gauge (reed)       - interrupt
            Cup anemometer (reed pulses)           - interrupt
            Capacitive soil moisture               - ADC1
  Edge    : temp/RH-compensated level, rain mm / 1 h / intensity,
            simple quality flags, calibration version.
  Cloud   : HTTPS POST (JSON) to Supabase REST table "sensor_readings"
            every 60 s. The Bolt.new dashboard reads the same table.

  Libraries (Arduino IDE > Library Manager):
    - Adafruit BME280 Library   (also installs Adafruit Unified Sensor, Adafruit BusIO)
  Board: "ESP32 Dev Module"  (Boards Manager: "esp32" by Espressif)

  PIN MAP
    GPIO21 SDA  -> BME280 SDA        GPIO22 SCL -> BME280 SCL
    GPIO5  TRIG -> Ultrasonic TRIG   GPIO18 ECHO <- Ultrasonic ECHO via 1k/2k divider
    GPIO27 -> Rain reed (other side GND)
    GPIO26 -> Anemometer reed (other side GND)
    GPIO34 <- Soil moisture AOUT
*/

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

// ===================== USER SETTINGS =====================
const char* WIFI_SSID     = "YOUR_WIFI_NAME";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";

// Supabase: Project Settings > API
const char* SUPABASE_URL  = "https://YOUR-PROJECT-ID.supabase.co";
const char* SUPABASE_KEY  = "YOUR_SUPABASE_ANON_PUBLIC_KEY";
const char* TABLE_NAME    = "sensor_readings";

const char* NODE_ID       = "node-01";
const char* CAL_VERSION   = "cal-v1";

// Site calibration (re-check on site: see slide "Calibration workflow")
const float SENSOR_MOUNT_HEIGHT_CM = 300.0;  // sensor face to ZERO level (drain bottom / reference)
const float ZERO_OFFSET_CM         = 0.0;    // fine zero offset from manual staff-gauge check
const float DIST_SCALE             = 1.0;    // ultrasonic scale from bench fit
const float RAIN_MM_PER_TIP        = 0.2;    // from pour test
const float RAIN_UNDERCATCH_GAIN   = 1.00;   // >1.0 compensates high-intensity under-catch
const float WIND_MS_PER_HZ         = 0.667;  // 1 pulse/s = 2.4 km/h (typical reed anemometer)
const int   SOIL_RAW_DRY           = 3000;   // ADC in air
const int   SOIL_RAW_WET           = 1300;   // ADC in water

const uint32_t SEND_INTERVAL_MS    = 60000;  // 1 reading per minute
// =========================================================

// Pins
#define PIN_TRIG 5
#define PIN_ECHO 18
#define PIN_RAIN 27
#define PIN_WIND 26
#define PIN_SOIL 34

Adafruit_BME280 bme;
bool bmeOk = false;

// ---------- Interrupt counters ----------
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
volatile uint32_t rainTipsTotal = 0, rainTipsMinute = 0, windPulses = 0;
volatile uint32_t lastRainUs = 0, lastWindUs = 0;

void IRAM_ATTR onRain() {
  uint32_t now = micros();
  if (now - lastRainUs > 100000UL) {            // 100 ms debounce
    portENTER_CRITICAL_ISR(&mux);
    rainTipsTotal++; rainTipsMinute++;
    portEXIT_CRITICAL_ISR(&mux);
    lastRainUs = now;
  }
}
void IRAM_ATTR onWind() {
  uint32_t now = micros();
  if (now - lastWindUs > 5000UL) {              // 5 ms debounce
    portENTER_CRITICAL_ISR(&mux);
    windPulses++;
    portEXIT_CRITICAL_ISR(&mux);
    lastWindUs = now;
  }
}

// ---------- Rain history: 60 one-minute buckets ----------
uint16_t rainBuckets[60] = {0};
uint8_t  bucketIdx = 0;

float rainLastHourMm() {
  uint32_t s = 0; for (int i = 0; i < 60; i++) s += rainBuckets[i];
  return s * RAIN_MM_PER_TIP * RAIN_UNDERCATCH_GAIN;
}
float rainIntensityMmh() {                      // last 10 min scaled to mm/h
  uint32_t s = 0;
  for (int i = 0; i < 10; i++) s += rainBuckets[(bucketIdx + 60 - i) % 60];
  return s * RAIN_MM_PER_TIP * RAIN_UNDERCATCH_GAIN * 6.0;
}

// ---------- Ultrasonic ----------
float speedOfSound(float tC, float rh) {
  float c = 331.3 + 0.606 * tC;                 // slide 7 formula
  c += rh * (0.001 + 0.0006 * tC);              // small secondary humidity term (approx.)
  return c;                                     // m/s
}

float readDistanceRawCm(float cMs) {            // single shot
  digitalWrite(PIN_TRIG, LOW);  delayMicroseconds(4);
  digitalWrite(PIN_TRIG, HIGH); delayMicroseconds(12);
  digitalWrite(PIN_TRIG, LOW);
  unsigned long us = pulseIn(PIN_ECHO, HIGH, 30000UL);   // ~5 m timeout
  if (us == 0) return NAN;
  return (us * 1e-6f * cMs / 2.0f) * 100.0f;    // cm
}

// median of N valid samples (rejects spikes), also returns spread
float readDistanceMedianCm(float cMs, float &spreadCm, int &valid) {
  const int N = 9; float v[N]; valid = 0;
  for (int i = 0; i < N; i++) {
    float d = readDistanceRawCm(cMs);
    if (!isnan(d) && d > 20 && d < 450) v[valid++] = d;
    delay(60);
  }
  if (valid < 3) { spreadCm = NAN; return NAN; }
  for (int i = 0; i < valid - 1; i++)           // insertion sort
    for (int j = i + 1; j < valid; j++)
      if (v[j] < v[i]) { float t = v[i]; v[i] = v[j]; v[j] = t; }
  spreadCm = v[valid - 1] - v[0];
  return v[valid / 2];
}

// ---------- Helpers ----------
float lastLevelCm = NAN;
uint32_t lastSendOk = 0;   // millis() of last successful upload

void connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;
  Serial.printf("Connecting to %s", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000) { delay(500); Serial.print("."); }
  Serial.println(WiFi.status() == WL_CONNECTED ? "\nWiFi connected" : "\nWiFi FAILED");
}

String num(float v, int dec = 2) { return isnan(v) ? "null" : String(v, dec); }

bool postReading(const String &json) {
  if (WiFi.status() != WL_CONNECTED) return false;
  WiFiClientSecure client;
  client.setInsecure();                         // prototype only; pin the certificate for production
  HTTPClient http;
  String url = String(SUPABASE_URL) + "/rest/v1/" + TABLE_NAME;
  if (!http.begin(client, url)) return false;
  http.addHeader("Content-Type", "application/json");
  http.addHeader("apikey", SUPABASE_KEY);
  http.addHeader("Authorization", String("Bearer ") + SUPABASE_KEY);
  http.addHeader("Prefer", "return=minimal");
  int code = http.POST(json);
  Serial.printf("POST -> HTTP %d\n", code);
  if (code < 200 || code >= 300) Serial.println(http.getString());
  http.end();
  bool ok = code >= 200 && code < 300;
  if (ok) lastSendOk = millis();
  return ok;
}

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\nHydroSentinel node starting");

  pinMode(PIN_TRIG, OUTPUT); digitalWrite(PIN_TRIG, LOW);
  pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_RAIN, INPUT_PULLUP);
  pinMode(PIN_WIND, INPUT_PULLUP);
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_SOIL, ADC_11db);

  attachInterrupt(digitalPinToInterrupt(PIN_RAIN), onRain, FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_WIND), onWind, FALLING);

  Wire.begin(21, 22);
  bmeOk = bme.begin(0x76) || bme.begin(0x77);
  Serial.println(bmeOk ? "BME280 OK" : "BME280 NOT FOUND (check wiring)");

  connectWiFi();
  lastSendOk = millis();
}

// ---------- Main loop ----------
uint32_t lastCycle = 0, lastBucketMs = 0;

void loop() {
  uint32_t now = millis();

  // advance rain bucket every minute (independent of send timing)
  if (now - lastBucketMs >= 60000UL) {
    lastBucketMs += 60000UL;
    portENTER_CRITICAL(&mux);
    uint32_t tips = rainTipsMinute; rainTipsMinute = 0;
    portEXIT_CRITICAL(&mux);
    bucketIdx = (bucketIdx + 1) % 60;
    rainBuckets[bucketIdx] = (uint16_t)tips;
  }

  if (now - lastCycle < SEND_INTERVAL_MS) { delay(50); return; }
  uint32_t dtMs = now - lastCycle; lastCycle = now;

  // ---- Weather ----
  float tC = NAN, rh = NAN, hPa = NAN;
  if (bmeOk) {
    tC = bme.readTemperature(); rh = bme.readHumidity(); hPa = bme.readPressure() / 100.0f;
    if (isnan(tC) || isnan(rh)) { tC = NAN; rh = NAN; }
  }

  // ---- Level (compensated) ----
  String flags = "";
  float useT = isnan(tC) ? 25.0f : tC;          // fallback assumption if BME280 fails
  float useRH = isnan(rh) ? 60.0f : rh;
  if (isnan(tC)) flags += "BME_FAIL;";
  float cMs = speedOfSound(useT, useRH);
  float spread; int valid;
  float distCm = readDistanceMedianCm(cMs, spread, valid);
  float levelCm = NAN;
  if (isnan(distCm)) {
    flags += "NO_ECHO;";
  } else {
    distCm = distCm * DIST_SCALE;
    levelCm = SENSOR_MOUNT_HEIGHT_CM - distCm - ZERO_OFFSET_CM;
    if (spread > 5.0) flags += "NOISY_ECHO;";                 // fog / spray / wind
    if (levelCm < -10 || levelCm > SENSOR_MOUNT_HEIGHT_CM) flags += "IMPLAUSIBLE;";
    if (!isnan(lastLevelCm) && fabs(levelCm - lastLevelCm) > 30.0) flags += "SPIKE;";
    lastLevelCm = levelCm;
  }
  if (!isnan(rh) && rh > 95.0) flags += "HIGH_RH;";

  // ---- Rain ----
  portENTER_CRITICAL(&mux);
  uint32_t tipsTotal = rainTipsTotal;
  uint32_t wp = windPulses; windPulses = 0;
  portEXIT_CRITICAL(&mux);
  float rainTotalMm = tipsTotal * RAIN_MM_PER_TIP * RAIN_UNDERCATCH_GAIN;
  float rain1h = rainLastHourMm();
  float rainInt = rainIntensityMmh();

  // ---- Wind ----
  float windMs = (wp / (dtMs / 1000.0f)) * WIND_MS_PER_HZ;

  // ---- Soil ----
  int raw = analogRead(PIN_SOIL);
  float soilPct = constrain((float)(SOIL_RAW_DRY - raw) * 100.0f / (SOIL_RAW_DRY - SOIL_RAW_WET), 0.0f, 100.0f);

  // ---- Simple edge cross-check (full fusion happens in dashboard/backend) ----
  if (rainInt > 5.0 && !isnan(levelCm) && levelCm < 1.0) flags += "RAIN_NO_LEVEL;";
  if (flags.length() == 0) flags = "OK";

  // ---- Build JSON ----
  String json = "{";
  json += "\"node_id\":\"" + String(NODE_ID) + "\",";
  json += "\"level_cm\":" + num(levelCm, 1) + ",";
  json += "\"distance_cm\":" + num(distCm, 1) + ",";
  json += "\"air_temp_c\":" + num(tC, 1) + ",";
  json += "\"humidity_pct\":" + num(rh, 1) + ",";
  json += "\"pressure_hpa\":" + num(hPa, 1) + ",";
  json += "\"rain_total_mm\":" + num(rainTotalMm, 1) + ",";
  json += "\"rain_1h_mm\":" + num(rain1h, 1) + ",";
  json += "\"rain_intensity_mmh\":" + num(rainInt, 1) + ",";
  json += "\"wind_ms\":" + num(windMs, 1) + ",";
  json += "\"soil_pct\":" + num(soilPct, 0) + ",";
  json += "\"quality_flags\":\"" + flags + "\",";
  json += "\"cal_version\":\"" + String(CAL_VERSION) + "\",";
  json += "\"rssi\":" + String(WiFi.RSSI());
  json += "}";

  Serial.println(json);

  // ---- Send (with reconnect + 1 retry) ----
  connectWiFi();
  if (!postReading(json)) { delay(2000); connectWiFi(); postReading(json); }

  // Safety: reboot if nothing has been delivered for 30 min
  if (millis() - lastSendOk > 1800000UL) ESP.restart();
}
