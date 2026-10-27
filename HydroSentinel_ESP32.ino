// SPDX-License-Identifier: MIT
// Copyright (c) 2026 HydroSentinel Contributors
//
// Licensed under the MIT License. See the LICENSE file in the repository root
// for the full license text. This software is provided "AS IS", without warranty
// of any kind. HydroSentinel is a research prototype and must not be relied upon
// as the sole source for flood warnings or emergency decisions.

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

  PIN MAP (same pins as the HydroSentinel test sketch)
    GPIO21 BME_SDA      -> BME280 SDA        GPIO22 BME_SCL   -> BME280 SCL
    GPIO5  TRIG_PIN     -> Ultrasonic TRIG   GPIO18 ECHO_PIN  <- Ultrasonic ECHO via 1k/2k divider
    GPIO27 RAIN_DIGITAL -> Rain input (other side GND)
    GPIO32 RAIN_ANALOG  <- Rain board analog out (reserved, not read yet)
    GPIO26 WIND_PIN     -> Anemometer reed (other side GND)
    GPIO34 SOIL_PIN     <- Soil moisture AOUT
    GPIO25 BUZZER_PIN   -> Active buzzer (set as output, kept off)
*/

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <ESPmDNS.h>

// ===================== USER SETTINGS =====================
// Calibration lives in secrets.h (optional, git-ignored). WiFi and Supabase details can be
// entered on the node itself through the setup portal (see README), or preset in secrets.h.
#if __has_include("secrets.h")
#include "secrets.h"            // your private settings (git-ignored)
#else
#include "secrets.example.h"    // placeholders: the setup portal will ask for WiFi + Supabase
#endif
// =========================================================

// --- PIN DEFINITIONS ---
#define BME_SDA 21
#define BME_SCL 22
#define SOIL_PIN 34        // Analog input for soil moisture (ADC1)
#define RAIN_DIGITAL 27    // Rain input (interrupt)
#define RAIN_ANALOG 32     // Reserved: analog rain intensity (not read yet)
#define TRIG_PIN 5         // Ultrasonic Trigger
#define ECHO_PIN 18        // Ultrasonic Echo (via voltage divider)
#define WIND_PIN 26        // Anemometer reed (kept from original, not in test sketch)
#define BUZZER_PIN 25      // Active buzzer output

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
  digitalWrite(TRIG_PIN, LOW);  delayMicroseconds(4);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(12);
  digitalWrite(TRIG_PIN, LOW);
  unsigned long us = pulseIn(ECHO_PIN, HIGH, 30000UL);   // ~5 m timeout
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

// ---------- Setup portal (WiFi scan dropdown + settings, saved in flash) ----------
Preferences prefs;
WebServer server(80);
DNSServer dnsServer;
String cfgSsid, cfgPass, cfgUrl, cfgKey, cfgNode;
bool forcePortal = false;
const char* PORTAL_AP_NAME = "HydroSentinel-Setup";
const char* PORTAL_AP_PASS = "hydrosetup";     // WPA2 password for the setup hotspot

const char PORTAL_HTML[] PROGMEM = R"rawliteral(<!DOCTYPE html><html><head><meta charset=utf-8>
<meta name=viewport content="width=device-width,initial-scale=1"><title>HydroSentinel Setup</title>
<style>body{font:16px system-ui,sans-serif;background:#f4f7fa;margin:0;padding:14px}.c{max-width:460px;margin:auto;background:#fff;border-radius:12px;padding:18px;box-shadow:0 1px 6px #0002}
h1{font-size:20px;margin:0 0 10px}label{display:block;font-weight:600;margin-top:12px;font-size:14px}input,select{width:100%;padding:10px;margin-top:4px;border:1px solid #c5d2dc;border-radius:8px;font-size:16px;box-sizing:border-box}
button{margin-top:14px;padding:11px;width:100%;border:0;border-radius:8px;background:#0b7fb5;color:#fff;font-size:16px;font-weight:600}button.s{background:#fff;color:#0b7fb5;border:1px solid #0b7fb5;margin-top:8px}
.x{display:flex;gap:8px;align-items:center;font-weight:400}.x input{width:auto;margin:0}small{color:#5b6b78}</style></head><body><div class=c>
<h1>&#127754; HydroSentinel setup</h1><small>Choose your WiFi, enter its password and your Supabase details.</small>
<form method=POST action=/save>
<label>WiFi network</label><select id=s name=ssid_sel></select>
<input id=m name=ssid_man placeholder="Network name" style="display:none;margin-top:8px">
<button type=button class=s onclick=scan()>&#8635; Refresh network list</button>
<label>WiFi password</label><input id=p name=pass type=password autocomplete=off>
<label class=x><input type=checkbox onclick="p.type=this.checked?'text':'password'"> Show password</label>
<label>Supabase project URL</label><input id=u name=url placeholder="https://xxxx.supabase.co" required>
<label>Supabase anon public key</label><input id=k name=key type=password placeholder="(leave empty to keep the saved key)">
<label>Node ID</label><input id=n name=node value=node-01 required>
<button>Save and connect</button></form><p id=st><small></small></p></div>
<script>function esc(t){return t.replace(/&/g,'&amp;').replace(/"/g,'&quot;').replace(/</g,'&lt;')}
async function scan(){st.textContent='Scanning for networks...';try{const r=await(await fetch('/scan')).json();
s.innerHTML=r.map(x=>'<option value="'+esc(x.s)+'">'+esc(x.s)+' ('+x.r+' dBm)'+(x.l?' &#128274;':'')+'</option>').join('')+'<option value="__other">Other / hidden network...</option>';
st.textContent=r.length+' networks found'}catch(e){st.textContent='Scan failed, press Refresh'}chg()}
function chg(){m.style.display=s.value=='__other'?'block':'none'}s.onchange=chg;
fetch('/info').then(r=>r.json()).then(i=>{if(i.url)u.value=i.url;if(i.node)n.value=i.node});scan();</script></body></html>)rawliteral";

String jsonEsc(String v) { v.replace("\\", "\\\\"); v.replace("\"", "\\\""); return v; }

bool configured() {
  return cfgSsid.length() && cfgSsid != "YOUR_WIFI_NAME" &&
         cfgUrl.length() && cfgUrl.indexOf("YOUR-PROJECT-ID") < 0 &&
         cfgKey.length() && cfgKey != "YOUR_SUPABASE_ANON_PUBLIC_KEY";
}

void loadConfig() {                              // saved values win; secrets.h values are the defaults
  prefs.begin("hydro", true);
  cfgSsid = prefs.getString("ssid", WIFI_SSID);
  cfgPass = prefs.getString("pass", WIFI_PASSWORD);
  cfgUrl  = prefs.getString("url",  SUPABASE_URL);
  cfgKey  = prefs.getString("key",  SUPABASE_KEY);
  cfgNode = prefs.getString("node", NODE_ID);
  prefs.end();
}

void runPortal() {
  Serial.printf("\nSETUP PORTAL: join WiFi '%s' (password '%s'), then open http://192.168.4.1\n", PORTAL_AP_NAME, PORTAL_AP_PASS);
  WiFi.disconnect(true);
  WiFi.mode(WIFI_AP_STA);                        // AP for the user + STA so we can scan
  WiFi.softAP(PORTAL_AP_NAME, PORTAL_AP_PASS);
  delay(300);
  dnsServer.start(53, "*", WiFi.softAPIP());     // captive portal: any address opens the form

  server.on("/", HTTP_GET, []() { server.send_P(200, "text/html", PORTAL_HTML); });
  server.on("/info", HTTP_GET, []() {
    server.send(200, "application/json", "{\"url\":\"" + jsonEsc(configured() ? cfgUrl : String("")) + "\",\"node\":\"" + jsonEsc(cfgNode) + "\"}");
  });
  server.on("/scan", HTTP_GET, []() {
    int n = WiFi.scanNetworks();
    String out = "[", seen = "|";
    for (int i = 0; i < n; i++) {
      String name = WiFi.SSID(i);
      if (name.length() == 0 || seen.indexOf("|" + name + "|") >= 0) continue;   // skip hidden + duplicates
      seen += name + "|";
      if (out.length() > 1) out += ",";
      out += "{\"s\":\"" + jsonEsc(name) + "\",\"r\":" + String(WiFi.RSSI(i)) + ",\"l\":" + (WiFi.encryptionType(i) != WIFI_AUTH_OPEN ? "1" : "0") + "}";
    }
    WiFi.scanDelete();
    server.send(200, "application/json", out + "]");
  });
  server.on("/save", HTTP_POST, []() {
    String ssid = server.arg("ssid_sel");
    if (ssid == "__other") ssid = server.arg("ssid_man");
    String url = server.arg("url"); url.trim(); while (url.endsWith("/")) url.remove(url.length() - 1);
    String key = server.arg("key"); key.trim();
    String node = server.arg("node"); node.trim();
    if (key.length() == 0 && configured()) key = cfgKey;          // keep saved key if left empty
    if (ssid.length() == 0 || url.length() == 0 || key.length() == 0 || node.length() == 0) {
      server.send(400, "text/html", "<p>Missing information. Please go back and fill in all fields.</p>");
      return;
    }
    prefs.begin("hydro", false);
    prefs.putString("ssid", ssid); prefs.putString("pass", server.arg("pass"));
    prefs.putString("url", url);   prefs.putString("key", key); prefs.putString("node", node);
    prefs.end();
    server.send(200, "text/html", "<meta name=viewport content='width=device-width,initial-scale=1'><body style='font:18px system-ui;padding:20px'><h2>Saved &#9989;</h2><p>The node is restarting and will connect to <b>" + ssid + "</b>. You can close this page.</p>");
    delay(1500);
    ESP.restart();
  });
  server.onNotFound([]() { server.sendHeader("Location", "http://192.168.4.1/"); server.send(302, "text/plain", ""); });
  server.begin();

  uint32_t t0 = millis();
  while (millis() - t0 < 600000UL) { dnsServer.processNextRequest(); server.handleClient(); delay(2); }
  ESP.restart();                                 // give up after 10 min and retry normal start-up
}

// ---------- Helpers ----------
float lastLevelCm = NAN;
uint32_t lastSendOk = 0;   // millis() of last successful upload

void connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;
  Serial.printf("Connecting to %s", cfgSsid.c_str());
  WiFi.mode(WIFI_STA);
  WiFi.begin(cfgSsid.c_str(), cfgPass.c_str());
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
  String url = cfgUrl + "/rest/v1/" + TABLE_NAME;
  if (!http.begin(client, url)) return false;
  http.addHeader("Content-Type", "application/json");
  http.addHeader("apikey", cfgKey);
  http.addHeader("Authorization", String("Bearer ") + cfgKey);
  http.addHeader("Prefer", "return=minimal");
  int code = http.POST(json);
  Serial.printf("POST -> HTTP %d\n", code);
  if (code < 200 || code >= 300) Serial.println(http.getString());
  http.end();
  bool ok = code >= 200 && code < 300;
  if (ok) lastSendOk = millis();
  return ok;
}

// ---------- Node web app: the page is served by the ESP32 and all processing happens here ----------
#include "node_ui.h"
String lastJson = "";                            // latest sensor reading (served at /api/live)

bool hasA(const char* k) { return server.hasArg(k) && server.arg(k).length() > 0; }
float argF(const char* k, float d) { return hasA(k) ? server.arg(k).toFloat() : d; }

void handleProcess() {                           // POST: raw measurements in, processed result out
  server.sendHeader("Access-Control-Allow-Origin", "*");
  float tC = argF("temp_c", NAN), rh = argF("humidity_pct", NAN);
  String flags = "";
  if (isnan(tC)) flags += "BME_FAIL;";
  float useT = isnan(tC) ? 25.0f : tC, useRH = isnan(rh) ? 60.0f : rh;
  float c = speedOfSound(useT, useRH);

  float rawD = argF("distance_cm", NAN), spread = argF("spread_cm", 0);
  float dist = NAN, level = NAN, corr = NAN;
  if (isnan(rawD) || rawD <= 0) {
    flags += "NO_ECHO;";
  } else {
    float us = (rawD / 100.0f) * 2.0f / 343.0f * 1e6f;          // back to echo time at 343 m/s
    dist = (us * 1e-6f * c / 2.0f) * 100.0f * DIST_SCALE;       // recompute with compensated speed
    corr = dist - rawD;
    level = SENSOR_MOUNT_HEIGHT_CM - dist - ZERO_OFFSET_CM;
    if (spread > 5.0) flags += "NOISY_ECHO;";
    if (level < -10 || level > SENSOR_MOUNT_HEIGHT_CM) flags += "IMPLAUSIBLE;";
    float prev = argF("prev_level_cm", NAN);
    if (!isnan(prev) && fabs(level - prev) > 30.0) flags += "SPIKE;";
  }
  if (!isnan(rh) && rh > 95.0) flags += "HIGH_RH;";

  float rain1h  = argF("rain_tips_60min", 0) * RAIN_MM_PER_TIP * RAIN_UNDERCATCH_GAIN;
  float rainInt = argF("rain_tips_10min", 0) * RAIN_MM_PER_TIP * RAIN_UNDERCATCH_GAIN * 6.0f;
  float dt = argF("interval_s", 60); if (dt <= 0) dt = 60;
  float windMs = (argF("wind_pulses", 0) / dt) * WIND_MS_PER_HZ;
  float soil = NAN;
  if (hasA("soil_raw")) soil = constrain((float)(SOIL_RAW_DRY - argF("soil_raw", 0)) * 100.0f / (SOIL_RAW_DRY - SOIL_RAW_WET), 0.0f, 100.0f);
  if (rainInt > 5.0 && !isnan(level) && level < 1.0) flags += "RAIN_NO_LEVEL;";
  if (flags.length() == 0) flags = "OK";

  String j = "{\"level_cm\":" + num(level, 1) + ",\"corrected_distance_cm\":" + num(dist, 1) +
             ",\"correction_cm\":" + num(corr, 1) + ",\"speed_of_sound_ms\":" + num(c, 1) +
             ",\"rain_1h_mm\":" + num(rain1h, 1) + ",\"rain_intensity_mmh\":" + num(rainInt, 1) +
             ",\"wind_ms\":" + num(windMs, 1) + ",\"soil_pct\":" + num(soil, 0) +
             ",\"flags\":\"" + flags + "\"";
  if (server.arg("send") == "1") {               // optional: store this result in Supabase
    String f2 = (flags == "OK") ? String("MANUAL;") : (flags + "MANUAL;");
    String db = "{\"node_id\":\"" + cfgNode + "\",\"level_cm\":" + num(level, 1) + ",\"distance_cm\":" + num(dist, 1) +
                ",\"air_temp_c\":" + num(tC, 1) + ",\"humidity_pct\":" + num(rh, 1) +
                ",\"rain_1h_mm\":" + num(rain1h, 1) + ",\"rain_intensity_mmh\":" + num(rainInt, 1) +
                ",\"wind_ms\":" + num(windMs, 1) + ",\"soil_pct\":" + num(soil, 0) +
                ",\"quality_flags\":\"" + f2 + "\",\"cal_version\":\"" + String(CAL_VERSION) + "\"}";
    j += String(",\"sent\":") + (postReading(db) ? "true" : "false");
  }
  server.send(200, "application/json", j + "}");
}

void startApp() {                                // routes used in normal operation
  server.on("/", HTTP_GET, []() { server.send_P(200, "text/html", NODE_UI_HTML); });
  server.on("/api/process", HTTP_POST, handleProcess);
  server.on("/api/live", HTTP_GET, []() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "application/json", lastJson.length() ? lastJson : String("{\"waiting\":true}"));
  });
  server.on("/api/info", HTTP_GET, []() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "application/json",
      "{\"node\":\"" + jsonEsc(cfgNode) + "\",\"ip\":\"" + WiFi.localIP().toString() + "\",\"rssi\":" + String(WiFi.RSSI()) +
      ",\"cal_version\":\"" + String(CAL_VERSION) + "\",\"mount_height_cm\":" + num(SENSOR_MOUNT_HEIGHT_CM, 0) +
      ",\"rain_mm_per_tip\":" + num(RAIN_MM_PER_TIP, 2) + ",\"uptime_s\":" + String(millis() / 1000) + "}");
  });
  server.begin();
  if (MDNS.begin("hydrosentinel")) Serial.println("Open the node page at http://hydrosentinel.local");
  Serial.print("Open the node page at http://"); Serial.println(WiFi.localIP());
}

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\nHydroSentinel node starting");

  loadConfig();
  pinMode(0, INPUT_PULLUP);                      // BOOT button: hold 3 s at power-up to redo WiFi setup
  if (digitalRead(0) == LOW) {
    Serial.println("BOOT held... keep holding 3 s to reset WiFi setup");
    uint32_t t = millis();
    while (digitalRead(0) == LOW && millis() - t < 3000) delay(20);
    if (digitalRead(0) == LOW) forcePortal = true;
  }
  if (forcePortal || !configured()) runPortal(); // never returns: restarts after saving

  pinMode(TRIG_PIN, OUTPUT); digitalWrite(TRIG_PIN, LOW);
  pinMode(ECHO_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT); digitalWrite(BUZZER_PIN, LOW);   // buzzer off
  pinMode(RAIN_DIGITAL, INPUT_PULLUP);
  pinMode(WIND_PIN, INPUT_PULLUP);
  analogReadResolution(12);
  analogSetPinAttenuation(SOIL_PIN, ADC_11db);

  attachInterrupt(digitalPinToInterrupt(RAIN_DIGITAL), onRain, FALLING);
  attachInterrupt(digitalPinToInterrupt(WIND_PIN), onWind, FALLING);

  Wire.begin(BME_SDA, BME_SCL);
  bmeOk = bme.begin(0x76) || bme.begin(0x77);
  Serial.println(bmeOk ? "BME280 OK" : "BME280 NOT FOUND (check wiring)");

  connectWiFi();
  startApp();
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

  if (now - lastCycle < SEND_INTERVAL_MS) { server.handleClient(); delay(10); return; }
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
  int raw = analogRead(SOIL_PIN);
  float soilPct = constrain((float)(SOIL_RAW_DRY - raw) * 100.0f / (SOIL_RAW_DRY - SOIL_RAW_WET), 0.0f, 100.0f);

  // ---- Simple edge cross-check (full fusion happens in dashboard/backend) ----
  if (rainInt > 5.0 && !isnan(levelCm) && levelCm < 1.0) flags += "RAIN_NO_LEVEL;";
  if (flags.length() == 0) flags = "OK";

  // ---- Build JSON ----
  String json = "{";
  json += "\"node_id\":\"" + cfgNode + "\",";
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
  lastJson = json;

  // ---- Send (with reconnect + 1 retry) ----
  connectWiFi();
  if (!postReading(json)) { delay(2000); connectWiFi(); postReading(json); }

  // Safety: reboot if nothing has been delivered for 30 min
  if (millis() - lastSendOk > 1800000UL) ESP.restart();
}
