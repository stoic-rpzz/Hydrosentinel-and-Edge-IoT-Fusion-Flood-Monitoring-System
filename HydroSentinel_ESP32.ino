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
            quality flags, calibration version, FLOOD RISK SCORE + CONFIDENCE.
  Serial  : open the Serial Monitor (115200, Newline) and type i to enter all sensor
            values by hand and get the risk score and confidence.
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

// --- SITE FLOOD SETTINGS (edit for your location; can also be changed at run time) ---
const float WARN_LEVEL_CM   = 60.0f;    // water level that starts to worry you
const float DANGER_LEVEL_CM = 120.0f;   // water level at which the area is flooding
const int   AREA_EXPOSURE   = 3;        // 1 = high ground ... 5 = very low-lying / vulnerable zone

void serialCommands();                  // defined below (Serial Monitor input mode)
void printMenu();

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
  while (millis() - t0 < 600000UL) { dnsServer.processNextRequest(); server.handleClient(); serialCommands(); delay(2); }
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

// ---------- Flood risk engine: risk score (0-100) + confidence (0-100 %) ----------
// A transparent, rule-based PROTOTYPE score. It has not been calibrated or validated for real warnings.
struct Inputs {
  float distRawCm = NAN, spreadCm = 0, tempC = NAN, rhPct = NAN, pressHpa = NAN;
  float tips10 = 0, tips60 = 0, windPulses = 0, intervalS = 60, soilRaw = NAN;
  float prevLevelCm = NAN, prevMin = 10;       // earlier level + minutes ago (for the trend)
  float lastReadCm = NAN;                      // previous reading (for the spike check); NAN = use prevLevelCm
  float warnCm = WARN_LEVEL_CM, dangerCm = DANGER_LEVEL_CM;
  int   exposure = AREA_EXPOSURE;
};

struct Result {
  float c = NAN, distCm = NAN, corrCm = NAN, levelCm = NAN, trend = NAN;
  float rain1h = 0, rainInt = 0, windMs = 0, soilPct = NAN;
  float sLevel = 0, sTrend = 0, sRain = 0, sSoil = 0, risk = 0, conf = 100;
  const char* riskLabel = "LOW";
  const char* confLabel = "HIGH";
  String flags, notes;
};

String fmt(float v, int d = 1) { return isnan(v) ? String("n/a") : String(v, d); }

Result assess(const Inputs& in) {
  Result r;
  bool hasT = !isnan(in.tempC), hasRH = !isnan(in.rhPct);
  if (!hasT) r.flags += "BME_FAIL;";
  r.c = speedOfSound(hasT ? in.tempC : 25.0f, hasRH ? in.rhPct : 60.0f);

  if (isnan(in.distRawCm) || in.distRawCm <= 0) {
    r.flags += "NO_ECHO;";
  } else {
    float us = (in.distRawCm / 100.0f) * 2.0f / 343.0f * 1e6f;      // echo time at 343 m/s
    r.distCm  = (us * 1e-6f * r.c / 2.0f) * 100.0f * DIST_SCALE;    // recomputed with the corrected speed
    r.corrCm  = r.distCm - in.distRawCm;
    r.levelCm = SENSOR_MOUNT_HEIGHT_CM - r.distCm - ZERO_OFFSET_CM;
    float spikeRef = isnan(in.lastReadCm) ? in.prevLevelCm : in.lastReadCm;
    if (in.spreadCm > 5.0f) r.flags += "NOISY_ECHO;";
    if (r.levelCm < -10 || r.levelCm > SENSOR_MOUNT_HEIGHT_CM) r.flags += "IMPLAUSIBLE;";
    if (!isnan(spikeRef) && fabsf(r.levelCm - spikeRef) > 30.0f) r.flags += "SPIKE;";
  }
  if (hasRH && in.rhPct > 95.0f) r.flags += "HIGH_RH;";

  r.rain1h  = in.tips60 * RAIN_MM_PER_TIP * RAIN_UNDERCATCH_GAIN;
  r.rainInt = in.tips10 * RAIN_MM_PER_TIP * RAIN_UNDERCATCH_GAIN * 6.0f;
  float dt = in.intervalS > 0 ? in.intervalS : 60.0f;
  r.windMs = (in.windPulses / dt) * WIND_MS_PER_HZ;
  if (!isnan(in.soilRaw))
    r.soilPct = constrain((float)(SOIL_RAW_DRY - in.soilRaw) * 100.0f / (SOIL_RAW_DRY - SOIL_RAW_WET), 0.0f, 100.0f);

  bool haveLevel = !isnan(r.levelCm);
  if (r.rainInt > 5.0f && haveLevel && r.levelCm < 1.0f) r.flags += "RAIN_NO_LEVEL;";
  if (haveLevel && !isnan(in.prevLevelCm) && in.prevMin > 0) r.trend = (r.levelCm - in.prevLevelCm) / in.prevMin * 10.0f;   // cm per 10 min

  // cross-sensor checks: do level, rain and trend tell a consistent story?
  bool rainy = r.rainInt > 5.0f, dry = r.rainInt < 0.5f && r.rain1h < 1.0f;
  if (!isnan(r.trend)) {
    if (r.trend >= 3.0f && dry)    r.flags += "RISE_NO_RAIN;";
    if (rainy && r.trend <= 0.0f)  r.flags += "RAIN_LEVEL_FLAT;";
  }

  // ---- risk score: level 45 %, trend 20 %, rain 25 %, soil 10 %, scaled by area exposure ----
  float warn = fmaxf(in.warnCm, 1.0f), danger = fmaxf(in.dangerCm, warn + 1.0f);
  if (haveLevel) {
    float L = fmaxf(r.levelCm, 0.0f);
    r.sLevel = (L <= warn) ? 50.0f * L / warn : 50.0f + 50.0f * (L - warn) / (danger - warn);   // 50 at warning, 100 at danger
    r.sLevel = constrain(r.sLevel, 0.0f, 100.0f);
  }
  if (!isnan(r.trend)) r.sTrend = constrain(r.trend * 10.0f, 0.0f, 100.0f);                     // +10 cm per 10 min = 100
  r.sRain = fmaxf(constrain(r.rainInt * 2.0f, 0.0f, 100.0f), constrain(r.rain1h * 2.5f, 0.0f, 100.0f));   // 50 mm/h or 40 mm/h-total = 100
  if (!isnan(r.soilPct)) r.sSoil = r.soilPct;
  static const float EXPO[6] = {1.0f, 0.8f, 0.9f, 1.0f, 1.1f, 1.2f};
  int ex = constrain(in.exposure, 1, 5);
  r.risk = (0.45f * r.sLevel + 0.20f * r.sTrend + 0.25f * r.sRain + 0.10f * r.sSoil) * EXPO[ex];
  if (haveLevel && r.levelCm >= danger)    r.risk = fmaxf(r.risk, 80.0f);                         // never understate a high level
  else if (haveLevel && r.levelCm >= warn) r.risk = fmaxf(r.risk, 45.0f);
  r.risk = constrain(r.risk, 0.0f, 100.0f);
  r.riskLabel = r.risk < 25 ? "LOW" : r.risk < 50 ? "MODERATE" : r.risk < 75 ? "HIGH" : "SEVERE";

  if (!haveLevel) r.notes += "No water-level reading, so the level part was scored 0|";
  else r.notes += "Level " + fmt(r.levelCm) + " cm (warning " + fmt(warn, 0) + ", danger " + fmt(danger, 0) + ")|";
  if (!isnan(r.trend) && r.trend >= 3.0f) r.notes += "Level rising " + fmt(r.trend) + " cm per 10 min|";
  if (rainy) r.notes += "Heavy rain: " + fmt(r.rainInt, 0) + " mm/h|";
  else if (r.rain1h >= 10.0f) r.notes += "Rain in the last hour: " + fmt(r.rain1h) + " mm|";
  if (!isnan(r.soilPct) && r.soilPct >= 70.0f) r.notes += "Soil is saturated (" + fmt(r.soilPct, 0) + " %), so more runoff|";
  if (ex >= 4) r.notes += "Low-lying or vulnerable area raises the score|";
  else if (ex <= 2) r.notes += "Higher ground lowers the score|";

  // ---- confidence: start at 100 %, subtract for missing or conflicting evidence ----
  float conf = 100.0f;
  auto has = [&](const char* f) { return r.flags.indexOf(f) >= 0; };
  auto pen = [&](float v, const char* why) { conf -= v; r.notes += "Confidence -" + String((int)v) + ": " + why + "|"; };
  if (has("NO_ECHO"))         pen(55, "no water-level reading");
  if (has("IMPLAUSIBLE"))     pen(40, "level outside the physical range");
  if (has("NOISY_ECHO"))      pen(15, "noisy echo (fog, spray or wind)");
  if (has("SPIKE"))           pen(15, "level jumped suddenly");
  if (has("RAIN_NO_LEVEL"))   pen(20, "heavy rain but no level");
  if (has("RISE_NO_RAIN"))    pen(15, "level rising without rain (upstream release, blocked drain or sensor fault)");
  if (has("RAIN_LEVEL_FLAT")) pen(10, "heavy rain but level not rising (drain lag or sensor fault)");
  if (has("BME_FAIL"))        pen(10, "no temperature or humidity, so the level is less accurate");
  if (has("HIGH_RH"))         pen(5, "humidity above 95 %");
  if (isnan(r.trend) && haveLevel) pen(10, "no previous level, so the trend is unknown");
  if (isnan(r.soilPct))       pen(5, "no soil reading");
  r.conf = constrain(conf, 0.0f, 100.0f);
  r.confLabel = r.conf >= 80 ? "HIGH" : r.conf >= 55 ? "MEDIUM" : "LOW";

  if (r.flags.length() == 0) r.flags = "OK";
  return r;
}

String resultJson(const Result& r) {             // JSON without the closing brace
  return "{\"level_cm\":" + num(r.levelCm, 1) + ",\"corrected_distance_cm\":" + num(r.distCm, 1) +
         ",\"correction_cm\":" + num(r.corrCm, 1) + ",\"speed_of_sound_ms\":" + num(r.c, 1) +
         ",\"rain_1h_mm\":" + num(r.rain1h, 1) + ",\"rain_intensity_mmh\":" + num(r.rainInt, 1) +
         ",\"wind_ms\":" + num(r.windMs, 1) + ",\"soil_pct\":" + num(r.soilPct, 0) +
         ",\"flags\":\"" + r.flags + "\",\"trend_cm10\":" + num(r.trend, 1) +
         ",\"risk_score\":" + num(r.risk, 0) + ",\"risk_level\":\"" + r.riskLabel +
         "\",\"confidence_pct\":" + num(r.conf, 0) + ",\"confidence_level\":\"" + r.confLabel +
         "\",\"notes\":\"" + jsonEsc(r.notes) + "\"";
}

// ---------- Serial Monitor input mode: type sensor values, get risk score + confidence ----------
bool haveLast = false;                           // latest assessment from the real sensors
Inputs lastIn;
Result lastRes;

void printMenu() {
  Serial.println(F("\n========== HYDROSENTINEL SERIAL MENU =========="));
  Serial.println(F("Set the Serial Monitor line ending to 'Newline', type a command, press Enter:"));
  Serial.println(F("  i  = type in temperature, humidity and all sensor values -> flood risk + confidence"));
  Serial.println(F("  l  = show the latest assessment from the real sensors"));
  Serial.println(F("  m  = show this menu"));
}

void printAssessment(const Inputs& in, const Result& r) {
  Serial.println();
  Serial.println(F("=========== HYDROSENTINEL FLOOD ASSESSMENT ==========="));
  Serial.printf("Water level    : %s cm  (corrected distance %s cm, air correction %s cm)\n", fmt(r.levelCm).c_str(), fmt(r.distCm).c_str(), fmt(r.corrCm).c_str());
  Serial.printf("Level trend    : %s cm per 10 min\n", fmt(r.trend).c_str());
  Serial.printf("Rain           : %s mm in the last hour, %s mm/h now\n", fmt(r.rain1h).c_str(), fmt(r.rainInt).c_str());
  Serial.printf("Wind / soil    : %s m/s, soil moisture %s %%\n", fmt(r.windMs).c_str(), fmt(r.soilPct, 0).c_str());
  Serial.printf("Speed of sound : %s m/s\n", fmt(r.c).c_str());
  Serial.printf("Area exposure  : %d of 5   (warning %s cm, danger %s cm)\n", (int)constrain(in.exposure, 1, 5), fmt(in.warnCm, 0).c_str(), fmt(in.dangerCm, 0).c_str());
  Serial.printf("Data flags     : %s\n", r.flags.c_str());
  Serial.printf("Score parts    : level %.0f (45%%) | trend %.0f (20%%) | rain %.0f (25%%) | soil %.0f (10%%)\n", r.sLevel, r.sTrend, r.sRain, r.sSoil);
  Serial.printf("\n>>> FLOOD RISK SCORE : %.0f / 100  (%s)\n", r.risk, r.riskLabel);
  Serial.printf(">>> CONFIDENCE       : %.0f %%  (%s)\n", r.conf, r.confLabel);
  Serial.println(F("Why:"));
  int start = 0; String n = r.notes;
  while (start < (int)n.length()) {
    int e = n.indexOf('|', start); if (e < 0) e = n.length();
    Serial.println("  * " + n.substring(start, e));
    start = e + 1;
  }
  if (r.conf < 55) Serial.println(F("Confidence is LOW: check the sensors before acting on this score."));
  if (r.risk < 25)      Serial.println(F("Advice: normal monitoring."));
  else if (r.risk < 50) Serial.println(F("Advice: watch closely and verify the site if rain continues."));
  else if (r.risk < 75) Serial.println(F("Advice: prepare, alert local responders and verify on site."));
  else                  Serial.println(F("Advice: act now. Follow official flood warnings and evacuation guidance."));
  Serial.println(F("(Prototype score, not an official warning.)"));
}

bool readLineTimeout(String& out, uint32_t ms) {           // reads one typed line; handles \n, \r and \r\n
  static char prevCh = 0;
  out = ""; uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    while (Serial.available()) {
      char ch = Serial.read();
      if (ch == '\n' && prevCh == '\r') { prevCh = ch; continue; }
      prevCh = ch;
      if (ch == '\n' || ch == '\r') return true;
      out += ch;
    }
    server.handleClient(); delay(5);
  }
  return false;
}

bool askFloat(int n, int total, const char* label, const char* hint, float& v) {   // false = cancelled
  Serial.printf("[%d/%d] %s", n, total, label);
  if (hint[0]) Serial.printf(" - %s", hint);
  Serial.printf("  [%s]: ", isnan(v) ? "skip" : fmt(v, 2).c_str());
  String s;
  if (!readLineTimeout(s, 300000UL)) { Serial.println(F("\n(timed out, input cancelled)")); return false; }
  s.trim(); Serial.println(s);
  if (s.length() == 0) return true;                                   // Enter keeps the value in brackets
  if (s.equalsIgnoreCase("q")) { Serial.println(F("Cancelled.")); return false; }
  if (s.equalsIgnoreCase("skip") || s == "-") { v = NAN; return true; }
  char c0 = s[0];
  if (!(isdigit(c0) || c0 == '-' || c0 == '+' || c0 == '.')) { Serial.println(F("  Not a number, keeping the value in brackets.")); return true; }
  v = s.toFloat();
  return true;
}

void serialGuidedInput() {
  Inputs in;                                                          // typical starting values
  in.distRawCm = 150; in.spreadCm = 0; in.tempC = 30; in.rhPct = 70; in.pressHpa = 1006;
  in.tips10 = 0; in.tips60 = 0; in.windPulses = 0; in.intervalS = 60; in.soilRaw = 2500;
  in.prevLevelCm = NAN; in.prevMin = 10;
  float expo = in.exposure;
  struct Field { const char* label; const char* hint; float* v; };
  Field f[] = {
    {"Measured distance (cm)",         "ultrasonic reading, assuming 343 m/s", &in.distRawCm},
    {"Echo spread (cm)",               "how much the samples varied, 0 = steady", &in.spreadCm},
    {"Air temperature (C)",            "skip = sensor failed", &in.tempC},
    {"Humidity (%)",                   "0-100, skip = sensor failed", &in.rhPct},
    {"Pressure (hPa)",                 "optional, not used in the score", &in.pressHpa},
    {"Rain gauge tips, last 10 min",   "", &in.tips10},
    {"Rain gauge tips, last hour",     "", &in.tips60},
    {"Wind pulses",                    "counted during the interval", &in.windPulses},
    {"Interval (s)",                   "how long the wind pulses were counted", &in.intervalS},
    {"Soil sensor raw ADC",            "0-4095, skip = none", &in.soilRaw},
    {"Previous water level (cm)",      "skip = no trend", &in.prevLevelCm},
    {"Minutes since previous level",   "for the trend", &in.prevMin},
    {"Warning level (cm)",             "site threshold", &in.warnCm},
    {"Danger level (cm)",              "site threshold", &in.dangerCm},
    {"Area exposure 1-5",              "1 = high ground, 5 = very low-lying", &expo},
  };
  const int N = sizeof(f) / sizeof(f[0]);
  Serial.println(F("\n=== MANUAL INPUT ===  Type a value and press Enter. Enter alone keeps the value in [brackets],"));
  Serial.println(F("'skip' marks a value as missing, 'q' cancels."));
  for (int i = 0; i < N; i++) if (!askFloat(i + 1, N, f[i].label, f[i].hint, *f[i].v)) return;
  in.exposure = (int)(expo + 0.5f);
  printAssessment(in, assess(in));
  Serial.println(F("\nType i to enter another set of values, or m for the menu."));
}

void serialCommands() {
  if (!Serial.available()) return;
  int pk = Serial.peek();
  if (pk == '\n' || pk == '\r') { Serial.read(); return; }            // stray line ending
  String s;
  if (!readLineTimeout(s, 2000)) return;
  s.trim(); s.toLowerCase();
  if (s == "i" || s == "input")                                    serialGuidedInput();
  else if (s == "l" || s == "live") {
    if (haveLast) printAssessment(lastIn, lastRes);
    else Serial.println(F("No live reading yet. The first one arrives about a minute after start-up."));
  }
  else if (s == "m" || s == "menu" || s == "?" || s == "help")     printMenu();
  else if (s.length())                                             Serial.println(F("Unknown command. Type m for the menu."));
}

// ---------- Node web app: the page is served by the ESP32 and all processing happens here ----------
#include "node_ui.h"
String lastJson = "";                            // latest sensor reading (served at /api/live)

bool hasA(const char* k) { return server.hasArg(k) && server.arg(k).length() > 0; }
float argF(const char* k, float d) { return hasA(k) ? server.arg(k).toFloat() : d; }

void handleProcess() {                           // POST: raw measurements in, processed result + risk out
  server.sendHeader("Access-Control-Allow-Origin", "*");
  Inputs in;
  in.distRawCm = argF("distance_cm", NAN);   in.spreadCm = argF("spread_cm", 0);
  in.tempC     = argF("temp_c", NAN);        in.rhPct    = argF("humidity_pct", NAN);
  in.tips10    = argF("rain_tips_10min", 0); in.tips60   = argF("rain_tips_60min", 0);
  in.windPulses = argF("wind_pulses", 0);    in.intervalS = argF("interval_s", 60);
  in.soilRaw   = argF("soil_raw", NAN);
  in.prevLevelCm = argF("prev_level_cm", NAN); in.prevMin = argF("prev_minutes", 10);
  in.warnCm    = argF("warn_cm", WARN_LEVEL_CM); in.dangerCm = argF("danger_cm", DANGER_LEVEL_CM);
  in.exposure  = (int)argF("exposure", AREA_EXPOSURE);
  Result r = assess(in);

  String j = resultJson(r);
  if (server.arg("send") == "1") {               // optional: store this result in Supabase
    String f2 = (r.flags == "OK") ? String("MANUAL;") : (r.flags + "MANUAL;");
    String db = "{\"node_id\":\"" + cfgNode + "\",\"level_cm\":" + num(r.levelCm, 1) + ",\"distance_cm\":" + num(r.distCm, 1) +
                ",\"air_temp_c\":" + num(in.tempC, 1) + ",\"humidity_pct\":" + num(in.rhPct, 1) +
                ",\"rain_1h_mm\":" + num(r.rain1h, 1) + ",\"rain_intensity_mmh\":" + num(r.rainInt, 1) +
                ",\"wind_ms\":" + num(r.windMs, 1) + ",\"soil_pct\":" + num(r.soilPct, 0) +
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
  printMenu();                                   // type i in the Serial Monitor to enter values by hand

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
float lvlHist[10]; uint32_t lvlT[10]; uint8_t lvlN = 0, lvlHead = 0;   // level history for the trend

void loop() {
  uint32_t now = millis();
  serialCommands();                              // Serial Monitor: type i, l or m

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

  // ---- Raw readings (the risk engine applies the air-temperature correction) ----
  float spread; int valid;
  float distRaw = readDistanceMedianCm(343.0f, spread, valid);

  portENTER_CRITICAL(&mux);
  uint32_t tipsTotal = rainTipsTotal;
  uint32_t wp = windPulses; windPulses = 0;
  portEXIT_CRITICAL(&mux);
  uint32_t tips60 = 0, tips10 = 0;
  for (int i = 0; i < 60; i++) tips60 += rainBuckets[i];
  for (int i = 0; i < 10; i++) tips10 += rainBuckets[(bucketIdx + 60 - i) % 60];
  float rainTotalMm = tipsTotal * RAIN_MM_PER_TIP * RAIN_UNDERCATCH_GAIN;

  // level ~10 minutes ago, for a stable trend (needs at least 5 readings)
  float basePrev = NAN, baseMin = 10;
  if (lvlN >= 5) { int oldest = (lvlN < 10) ? 0 : lvlHead; basePrev = lvlHist[oldest]; baseMin = (now - lvlT[oldest]) / 60000.0f; }

  Inputs in;
  in.distRawCm = distRaw; in.spreadCm = isnan(spread) ? 0.0f : spread;
  in.tempC = tC; in.rhPct = rh; in.pressHpa = hPa;
  in.tips10 = tips10; in.tips60 = tips60; in.windPulses = wp;
  in.intervalS = dtMs / 1000.0f; in.soilRaw = analogRead(SOIL_PIN);
  in.prevLevelCm = basePrev; in.prevMin = baseMin; in.lastReadCm = lastLevelCm;
  Result r = assess(in);
  if (!isnan(r.levelCm)) {
    lastLevelCm = r.levelCm;
    lvlHist[lvlHead] = r.levelCm; lvlT[lvlHead] = now; lvlHead = (lvlHead + 1) % 10; if (lvlN < 10) lvlN++;
  }
  lastIn = in; lastRes = r; haveLast = true;

  // ---- Build JSON (same columns as the database table) ----
  String json = "{";
  json += "\"node_id\":\"" + cfgNode + "\",";
  json += "\"level_cm\":" + num(r.levelCm, 1) + ",";
  json += "\"distance_cm\":" + num(r.distCm, 1) + ",";
  json += "\"air_temp_c\":" + num(tC, 1) + ",";
  json += "\"humidity_pct\":" + num(rh, 1) + ",";
  json += "\"pressure_hpa\":" + num(hPa, 1) + ",";
  json += "\"rain_total_mm\":" + num(rainTotalMm, 1) + ",";
  json += "\"rain_1h_mm\":" + num(r.rain1h, 1) + ",";
  json += "\"rain_intensity_mmh\":" + num(r.rainInt, 1) + ",";
  json += "\"wind_ms\":" + num(r.windMs, 1) + ",";
  json += "\"soil_pct\":" + num(r.soilPct, 0) + ",";
  json += "\"quality_flags\":\"" + r.flags + "\",";
  json += "\"cal_version\":\"" + String(CAL_VERSION) + "\",";
  json += "\"rssi\":" + String(WiFi.RSSI());
  json += "}";

  Serial.println(json);
  lastJson = json;
  Serial.printf("RISK %.0f/100 (%s) | confidence %.0f %% (%s) | type l for details, i to type values by hand\n", r.risk, r.riskLabel, r.conf, r.confLabel);

  // ---- Send (with reconnect + 1 retry) ----
  connectWiFi();
  if (!postReading(json)) { delay(2000); connectWiFi(); postReading(json); }

  // Safety: reboot if nothing has been delivered for 30 min
  if (millis() - lastSendOk > 1800000UL) ESP.restart();
}
