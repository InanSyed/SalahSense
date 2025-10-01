#include <Arduino.h>
#include <Wire.h>
#include <RTClib.h>
#include <BH1750.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <time.h>
#include <math.h>
#include <algorithm>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include "nvs_flash.h"
#include <ESPmDNS.h>

/* ================== Wi-Fi ================== */
const char* WIFI_SSID = "HarryPotter2.4";
const char* WIFI_PASS = "inan1yasa";

/* ================== Pins =================== */
#define SDA_PIN    27
#define SCL_PIN    14

#define FSR_PIN    32          // ADC1_CH4
#define PIEZO_PIN  33          // ADC1_CH5
#define MOTOR_PIN  26          // PWM to 2N2222 base via ~1k; motor+->5V, motor- -> transistor; diode across motor

/* =============== Peripherals =============== */
RTC_DS3231 rtc;
BH1750     lux;
WebServer  http(80);
Preferences prefs;
bool onSTA = false;

/* =============== Utilities ================= */
String pad2(int v){ char b[8]; snprintf(b,sizeof(b),"%02d",v); return String(b); }
String fmtClock(const DateTime& dt){ return pad2(dt.hour())+":"+pad2(dt.minute())+":"+pad2(dt.second()); }
String fmtDate(const DateTime& dt){ char b[16]; snprintf(b,sizeof(b), "%04d-%02d-%02d", dt.year(), dt.month(), dt.day()); return String(b); }

bool connectSTA(uint32_t to=15000){
  WiFi.mode(WIFI_STA); WiFi.begin(WIFI_SSID, WIFI_PASS);
  uint32_t t0=millis();
  while(WiFi.status()!=WL_CONNECTED && (millis()-t0)<to) delay(250);
  return (onSTA=(WiFi.status()==WL_CONNECTED));
}
void startAP(){ WiFi.mode(WIFI_AP); WiFi.softAP("SalahSense","12345678"); }

/* ============= NTP / Timezone ============== */
// We keep RTC in *local* time by applying an explicit UTC offset.
static int tz_offset_min = -240;     // Windsor EDT default
uint32_t lastNtpMillis = 0;

void tz_load(){ tz_offset_min = prefs.getInt("tz_off", -240); }
void tz_save(){ prefs.putInt("tz_off", tz_offset_min); }

bool ntp_sync_local(uint32_t max_wait_ms=25000){
  if(!onSTA) return false;
  configTime(0,0,"pool.ntp.org","time.nrc.ca"); // get UTC
  time_t utc=0; uint32_t t0=millis();
  while(utc < 1700000000 && (millis()-t0)<max_wait_ms){ time(&utc); delay(500); }
  if (utc < 1700000000){ Serial.println("NTP failed"); return false; }
  time_t local_epoch = utc + (time_t)tz_offset_min*60;
  struct tm tmLocal; gmtime_r(&local_epoch, &tmLocal);
  rtc.adjust(DateTime(1900+tmLocal.tm_year, tmLocal.tm_mon+1, tmLocal.tm_mday, tmLocal.tm_hour, tmLocal.tm_min, tmLocal.tm_sec));
  lastNtpMillis = millis();
  Serial.printf("RTC set from NTP with UTC offset %d min\n", tz_offset_min);
  return true;
}

/* ================= Motor =================== */
static bool     motorActiveHigh = true;     // invert if hardware acts opposite
static bool     motorState = false;         // logical state
static uint8_t  motorPower = 255;           // 0..255 PWM
static uint8_t  motor_ledc_ch = 0;

void motor_load(){  motorActiveHigh = prefs.getBool("mot_inv", true); motorPower = prefs.getUChar("mot_pwm", 255); }
void motor_save(){  prefs.putBool("mot_inv", motorActiveHigh); prefs.putUChar("mot_pwm", motorPower); }
void motor_ledc_setup(){ ledcSetup(motor_ledc_ch, 1000 /*Hz*/, 8 /*bits*/); ledcAttachPin(MOTOR_PIN, motor_ledc_ch); }
void motor_applyHW(){ bool driveOn = motorActiveHigh ? motorState : !motorState; ledcWrite(motor_ledc_ch, driveOn ? motorPower : 0); }
void motor_on(){  motorState = true;  motor_applyHW(); }
void motor_off(){ motorState = false; motor_applyHW(); }
bool motor_is_on(){ return motorState; }

/* ============== Modes & Silence ============= */
enum Mode : uint8_t { AUTO=0, FORCE_ON=1, FORCE_OFF=2 };
static Mode mode = AUTO;

static bool     imUp = false;               // piezo “stop”
static uint32_t imUp_until_ms = 0;
static uint32_t bedEmptySinceMs = 0;
static const uint32_t IMUP_TIMEOUT_MS = 5UL*60UL*1000UL;  // 5 minutes

/* ================== FSR ===================== */
static uint16_t fsr_raw=0; static float fsr_avg=0.0f;
static int fsr_empty=800, fsr_occupied=1800, fsr_thresh=1300;  // stored
static bool bedOccupied=false;
static int  fsr_hys = 40; // hysteresis counts

void fsr_load(){ fsr_empty=prefs.getInt("fsr_empty",800); fsr_occupied=prefs.getInt("fsr_occ",1800); fsr_thresh=prefs.getInt("fsr_thr",(fsr_empty+fsr_occupied)/2); fsr_hys=prefs.getInt("fsr_hys",40); }
void fsr_save(){ prefs.putInt("fsr_empty",fsr_empty); prefs.putInt("fsr_occ",fsr_occupied); prefs.putInt("fsr_thr",fsr_thresh); prefs.putInt("fsr_hys",fsr_hys); }
void fsr_recalc(){ int lo=std::min((int)fsr_empty,(int)fsr_occupied), hi=std::max((int)fsr_empty,(int)fsr_occupied); fsr_thresh = lo + (hi-lo)/2; }
void fsr_begin(){
  analogSetWidth(12);
  analogSetAttenuation(ADC_11db);
  analogSetPinAttenuation(FSR_PIN, ADC_11db);
  pinMode(FSR_PIN, INPUT);
  fsr_raw = analogRead(FSR_PIN); fsr_avg = fsr_raw;
}
void fsr_read(){
  uint32_t s=0; for(int i=0;i<8;i++){ s += analogRead(FSR_PIN); delayMicroseconds(800); }
  fsr_raw = s/8; fsr_avg = 0.8f*fsr_avg + 0.2f*fsr_raw;
  int onTh=fsr_thresh+fsr_hys, offTh=fsr_thresh-fsr_hys;
  bool was=bedOccupied;
  if (!was && fsr_avg>=onTh) bedOccupied=true; else if (was && fsr_avg<=offTh) bedOccupied=false;
  if (!bedOccupied){ if(!bedEmptySinceMs) bedEmptySinceMs=millis(); } else bedEmptySinceMs=0;
}

/* ================= Piezo ==================== */
// One tap = I'm up (hard stop)
static uint16_t pz_raw=0; static float pz_avg=0, pz_dev=0, pz_peak=0;
static uint32_t pz_lastHit=0; static uint8_t pz_hits=0;
static int PZ_THR = 35;  // stored
void piezo_load(){ PZ_THR = prefs.getInt("pz_thr", 35); }
void piezo_save(){ prefs.putInt("pz_thr", PZ_THR); }
void piezo_begin(){
  analogSetPinAttenuation(PIEZO_PIN, ADC_11db);
  pinMode(PIEZO_PIN, INPUT);
  pz_raw = analogRead(PIEZO_PIN); pz_avg = pz_raw; pz_dev=0; pz_peak=0; pz_hits=0; pz_lastHit=0;
}
void piezo_finalize(){
  if (!pz_hits) return;
  pz_hits = 0;
  imUp = true;
  imUp_until_ms = millis() + IMUP_TIMEOUT_MS;
  motor_off();                   // HARD stop, regardless of mode
  Serial.println("Piezo TAP → HARD STOP (I'm up)");
}
void piezo_read(){
  uint32_t s=0; for(int i=0;i<8;i++){ s+=analogRead(PIEZO_PIN); delayMicroseconds(300); }
  pz_raw = s/8;
  pz_avg = 0.995f*pz_avg + 0.005f*pz_raw;
  float dev = fabsf((float)pz_raw - pz_avg);
  pz_dev = 0.8f*pz_dev + 0.2f*dev;
  pz_peak = max(pz_peak*0.90f, pz_dev);
  uint32_t now=millis();
  if (pz_dev > PZ_THR){
    if (now - pz_lastHit > 80){ pz_hits++; pz_lastHit = now; }
  }
  if (pz_hits>0 && (now - pz_lastHit) > 260) piezo_finalize();
}

/* =========== Magnetometer (QMC5883L) =========== */
class QMC5883L {
public:
  bool begin(TwoWire& w=Wire, uint8_t addr=0x0D){
    _w=&w; _addr=addr;
    writeReg(0x0B, 0x01); delay(10);        // soft reset
    writeReg(0x0B, 0x01);                   // normal
    writeReg(0x09, 0x01);                   // set/reset period
    writeReg(0x0A, 0x1D); delay(10);        // OSR512, RNG8G, ODR200Hz, CONT
    int16_t x,y,z; return readRaw(x,y,z);
  }
  bool readRaw(int16_t& x,int16_t& y,int16_t& z){
    uint8_t b[6];
    if(!readBytes(0x00,b,6)) return false;
    x=(int16_t)((uint16_t)b[1]<<8 | b[0]);
    y=(int16_t)((uint16_t)b[3]<<8 | b[2]);
    z=(int16_t)((uint16_t)b[5]<<8 | b[4]);
    return true;
  }
private:
  TwoWire* _w=nullptr; uint8_t _addr=0x0D;
  bool writeReg(uint8_t r,uint8_t v){ _w->beginTransmission(_addr); _w->write(r); _w->write(v); return _w->endTransmission()==0; }
  bool readBytes(uint8_t r,uint8_t* d,uint8_t n){
    _w->beginTransmission(_addr); _w->write(r);
    if(_w->endTransmission(false)!=0) return false;
    if(_w->requestFrom((int)_addr,(int)n)!=n) return false;
    for(uint8_t i=0;i<n;i++) d[i]=_w->read(); return true;
  }
};
QMC5883L qmc;

struct MagCal {
  bool running=false;
  int16_t xmin=32767,xmax=-32768, ymin=32767,ymax=-32768, zmin=32767,zmax=-32768;
  int16_t xoff=0,yoff=0,zoff=0;
  void clearMinMax(){ xmin=32767;xmax=-32768; ymin=32767;ymax=-32768; zmin=32767;zmax=-32768; }
} magcal;

static int16_t magX=0, magY=0, magZ=0;
static float   magHeadingDeg = NAN;
static uint32_t lastMagMs = 0;
static float magDeclinationDeg = 0.0f;  // stored

/* Location / Qibla */
// Use float for wider Preferences compatibility (putFloat/getFloat are ubiquitous)
static float userLat = 42.3149f;   // Windsor approx
static float userLon = -83.0364f;
static const double KAABA_LAT = 21.422487;
static const double KAABA_LON = 39.826206;
static float  qiblaBearingDeg = NAN;         // resolved: computed or override

// Qibla override (store current heading as Qibla)
static bool  qiblaOverrideEn = false;  // stored
static float qiblaOverrideDeg = NAN;   // stored

void mag_load(){
  magcal.xoff = prefs.getShort("mag_xoff", 0);
  magcal.yoff = prefs.getShort("mag_yoff", 0);
  magcal.zoff = prefs.getShort("mag_zoff", 0);
  magDeclinationDeg = prefs.getFloat("mag_decl", 0.0f);
  // Prefer float to ensure compatibility across core versions
  userLat = prefs.getFloat("lat", userLat);
  userLon = prefs.getFloat("lon", userLon);
  qiblaOverrideEn = prefs.getBool("qib_ovr_en", false);
  qiblaOverrideDeg = prefs.getFloat("qib_ovr_deg", NAN);
}
void mag_save(){
  prefs.putShort("mag_xoff", magcal.xoff);
  prefs.putShort("mag_yoff", magcal.yoff);
  prefs.putShort("mag_zoff", magcal.zoff);
  prefs.putFloat("mag_decl", magDeclinationDeg);
  prefs.putFloat("lat", userLat);
  prefs.putFloat("lon", userLon);
  prefs.putBool("qib_ovr_en", qiblaOverrideEn);
  prefs.putFloat("qib_ovr_deg", qiblaOverrideDeg);
}
static inline float wrap360(float a){ while(a<0) a+=360.0f; while(a>=360) a-=360.0f; return a; }
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
float qibla_bearing_deg(double latDeg, double lonDeg){
  double lat1 = latDeg * M_PI / 180.0;
  double lat2 = KAABA_LAT * M_PI / 180.0;
  double dLon = (KAABA_LON - lonDeg) * M_PI / 180.0;
  double y = sin(dLon) * cos(lat2);
  double x = cos(lat1)*sin(lat2) - sin(lat1)*cos(lat2)*cos(dLon);
  double b = atan2(y, x) * 180.0 / M_PI; if (b<0) b+=360.0; return (float)b;
}
void qibla_update(){
  if (qiblaOverrideEn && !isnan(qiblaOverrideDeg)) qiblaBearingDeg = wrap360(qiblaOverrideDeg);
  else if (!isnan(userLat) && !isnan(userLon))      qiblaBearingDeg = qibla_bearing_deg(userLat, userLon);
  else                                              qiblaBearingDeg = NAN;
}
void mag_update(){
  if (millis() - lastMagMs < 120) return;
  lastMagMs = millis();
  int16_t x,y,z; if (!qmc.readRaw(x,y,z)) return;
  x -= magcal.xoff; y -= magcal.yoff; z -= magcal.zoff;
  magX=x; magY=y; magZ=z;
  float h = atan2f((float)y,(float)x) * 180.0f / (float)PI;
  h = wrap360(h + magDeclinationDeg);
  magHeadingDeg = h;
    magcal.xmin = std::min(magcal.xmin, (int16_t)(x + magcal.xoff));
    magcal.xmax = std::max(magcal.xmax, (int16_t)(x + magcal.xoff));
    magcal.ymin = std::min(magcal.ymin, (int16_t)(y + magcal.yoff));
    magcal.ymax = std::max(magcal.ymax, (int16_t)(y + magcal.yoff));
    magcal.zmin = std::min(magcal.zmin, (int16_t)(z + magcal.zoff));
    magcal.zmax = std::max(magcal.zmax, (int16_t)(z + magcal.zoff));
}

/* ============ Sunrise-only Alarm ============= */
/* NO daily manual alarm time; only Sunrise - offset. Manual "test alarm" is one-shot. */

static bool sunriseUse = true;          // stored (true = use online sunrise)
static int  sunriseOffsetMin = 20;      // minutes BEFORE sunrise (stored)
static String sunriseTodayStr;          // "06:51"
static String sunriseAlarmStr;          // "06:31"
static uint8_t sunriseAlarmHour=6, sunriseAlarmMin=30;
static uint32_t lastSunriseFetchMs=0;
static uint32_t lastSunriseDayKey=0;

uint32_t dayKey(const DateTime& d){ return (uint32_t)d.year()*372 + d.month()*31 + d.day(); }
void sunrise_load(){ sunriseUse=prefs.getBool("sun_use",true); sunriseOffsetMin=prefs.getInt("sun_off",20); }
void sunrise_save(){ prefs.putBool("sun_use",sunriseUse); prefs.putInt("sun_off",sunriseOffsetMin); }

/* Robust-ish substring parser helpers */
static bool extractQuotedValueAfter(const String& body, int startPos, const char* key, String& out){
  String k="\""; k+=key; k+="\"";
  int pk = body.indexOf(k, startPos); if (pk<0) return false;
  int c1 = body.indexOf('"', pk + k.length()); if (c1<0) return false;
  int c2 = body.indexOf('"', c1+1); if (c2<0) return false;
  out = body.substring(c1+1, c2); out.trim(); return true;
}

/* Primary: Windsor Islamic Association JSON */
bool fetchSunrise_WIA(uint8_t& sh, uint8_t& sm){
  if (!onSTA) return false;
  WiFiClientSecure client; client.setInsecure();
  HTTPClient https;
  const char* URL = "https://windsorislamicassociation.github.io/WIAPrayer.json";
  if (!https.begin(client, URL)) return false;
  int code = https.GET();
  if (code != 200){ https.end(); return false; }
  String body = https.getString();
  https.end();

  DateTime now = rtc.now(); // local
  String mpat = String("\"Month\":") + now.month();
  String dpat = String("\"Day\":")   + now.day();

  int searchFrom = 0;
  while (true){
    int pm = body.indexOf(mpat, searchFrom);
    if (pm < 0) return false;
    int pd = body.indexOf(dpat, pm);
    if (pd < 0 || pd - pm > 120) { searchFrom = pm + mpat.length(); continue; }

    String sunriseStr;
    if (!extractQuotedValueAfter(body, pd, "Sunrise", sunriseStr)) return false;
    sunriseStr.replace("AM",""); sunriseStr.replace("am",""); sunriseStr.replace(" PM",""); sunriseStr.replace(" pm","");
    sunriseStr.trim();
    int colon = sunriseStr.indexOf(':'); if (colon<0) return false;
    sh = (uint8_t) sunriseStr.substring(0,colon).toInt();
    sm = (uint8_t) sunriseStr.substring(colon+1).toInt();
    if (sh==12) sh=0; // 12:xx AM -> 0:xx
    return true;
  }
}

/* Fallback: Aladhan API by city (Windsor, Canada) */
bool fetchSunrise_Aladhan(uint8_t& sh, uint8_t& sm){
  if (!onSTA) return false;
  WiFiClientSecure client; client.setInsecure();
  HTTPClient https;
  const char* URL = "https://api.aladhan.com/v1/timingsByCity?city=Windsor&country=Canada&method=2";
  if (!https.begin(client, URL)) return false;
  int code = https.GET();
  if (code != 200){ https.end(); return false; }
  String body = https.getString();
  https.end();

  int p = body.indexOf("\"Sunrise\"");
  if (p<0) return false;
  int q1 = body.indexOf('"', p+9); if (q1<0) return false;
  int q2 = body.indexOf('"', q1+1); if (q2<0) return false;
  String v = body.substring(q1+1, q2); v.trim();
  int colon = v.indexOf(':'); if (colon<0) return false;
  sh = (uint8_t)v.substring(0,colon).toInt();
  sm = (uint8_t)v.substring(colon+1).toInt();
  return true;
}

void applySunriseToAlarm(uint8_t sh, uint8_t sm){
  int mins = (int)sh*60 + (int)sm - sunriseOffsetMin;
  while (mins < 0) mins += 1440;
  while (mins >= 1440) mins -= 1440;
  sunriseAlarmHour = (uint8_t)(mins/60);
  sunriseAlarmMin  = (uint8_t)(mins%60);
  sunriseTodayStr  = pad2(sh)+":"+pad2(sm);
  sunriseAlarmStr  = pad2(sunriseAlarmHour)+":"+pad2(sunriseAlarmMin);
}

void sunrise_maybe_update(const DateTime& now){
  if (!sunriseUse) return;
  uint32_t key = dayKey(now);
  if (key == lastSunriseDayKey && (millis()-lastSunriseFetchMs) < 6UL*60UL*60UL*1000UL) return; // once per day (or 6h)
  uint8_t sh=0, sm=0;
  bool ok = fetchSunrise_WIA(sh, sm);
  if (!ok) ok = fetchSunrise_Aladhan(sh, sm);
  if (ok){
    applySunriseToAlarm(sh, sm);
    lastSunriseFetchMs = millis();
    lastSunriseDayKey  = key;
    Serial.printf("Sunrise %02u:%02u -> wake %s (offset %d min)\n", sh, sm, sunriseAlarmStr.c_str(), sunriseOffsetMin);
  } else {
    Serial.println("Sunrise fetch failed (keeping previous)");
    lastSunriseFetchMs = millis();
    lastSunriseDayKey  = key;
  }
}

/* ================= Alarm Core ================= */
static bool     alarmActive = false;
static uint32_t alarmStartedMs = 0;
static uint32_t snoozeUntilMs = 0;

/* Manual one-shot testing (no daily manual alarm) */
static uint32_t testOneShotStartAtMs = 0;   // millis timestamp when to start
static uint32_t testAutoStopAfterMs  = 0;   // 0 = no auto stop

bool time_matches_now(uint8_t hh, uint8_t mm, const DateTime& now){ return (now.hour()==hh && now.minute()==mm); }

void pattern_alarm(){
  uint32_t t = millis() - alarmStartedMs;
  if (t < 4000){ motorPower = map(t, 0, 4000, 80, 255); motor_applyHW(); motor_on(); return; }
  uint32_t k = (t - 4000) % 900;
  bool on = (k < 550);
  motor_on(); if (on) motor_applyHW(); else ledcWrite(motor_ledc_ch, 0);
}
void alarm_start(){ if(alarmActive) return; alarmActive=true; alarmStartedMs=millis(); imUp=false; snoozeUntilMs=0; Serial.println("Alarm START"); }
void alarm_stop(){  if(!alarmActive) return; alarmActive=false; motor_off(); Serial.println("Alarm STOP"); }
void alarm_snooze(){ if(!alarmActive) return; alarmActive=false; motor_off(); snoozeUntilMs=millis()+5UL*60UL*1000UL; Serial.println("Snooze 5 min"); }

/* ============ Decision / Orchestration ============ */
void motor_logic_update(const DateTime& now){
  // Clear "I'm up" either after timeout or when bed empty for 10s
  if (imUp){
    bool timeout = (imUp_until_ms && millis()>imUp_until_ms);
    bool clearedByBed = (!bedOccupied && bedEmptySinceMs && (millis()-bedEmptySinceMs > 10000));
    if (timeout || clearedByBed){ imUp=false; imUp_until_ms=0; }
  }

  // Update sunrise-derived wake time daily
  sunrise_maybe_update(now);

  // Trigger the daily sunrise alarm (no other daily alarm)
  static uint32_t lastFiredDayKey = 0; static bool todayFired=false;
  uint32_t key = dayKey(now);
  if (key != lastFiredDayKey){ lastFiredDayKey=key; todayFired=false; }
  if (sunriseUse && !todayFired && time_matches_now(sunriseAlarmHour, sunriseAlarmMin, now)){
    todayFired=true; alarm_start();
  }

  // Manual one-shot alarm (testing)
  if (testOneShotStartAtMs && millis() >= testOneShotStartAtMs){
    testOneShotStartAtMs = 0;
    alarm_start();
    if (testAutoStopAfterMs) testAutoStopAfterMs = millis() + testAutoStopAfterMs;
  }
  if (alarmActive && testAutoStopAfterMs && millis() > testAutoStopAfterMs){
    testAutoStopAfterMs = 0;
    alarm_stop();
  }

  // Snooze window
  if (snoozeUntilMs && millis() < snoozeUntilMs){ motor_off(); return; }
  else if (snoozeUntilMs && millis() >= snoozeUntilMs){ snoozeUntilMs=0; alarm_start(); }

  // When alarm active
  if (alarmActive){
    if (imUp){ alarm_stop(); return; }                                     // one tap piezo = stop
    if (!bedOccupied && bedEmptySinceMs && (millis()-bedEmptySinceMs>10000)){ alarm_stop(); return; }
    pattern_alarm();
    return;
  }

  // HARD stop from piezo wins over everything
  if (imUp){ motor_off(); return; }

  // Normal motor control when not alarming
  switch(mode){
    case FORCE_ON:  motor_on(); break;
    case FORCE_OFF: motor_off(); break;
    case AUTO:
    default:
      if (bedOccupied) motor_on(); else motor_off();
      break;
  }
}

/* ================= Web UI (sunrise-only) ================== */
const char PAGE[] PROGMEM = R"HTML(
<!doctype html><html><head><meta charset=utf-8><meta name=viewport content="width=device-width, initial-scale=1">
<title>SalahSense</title>
<style>
:root{
  --bg:#0b1220; --card:#121a2a; --ink:#eaf2ff; --mut:#9db2d6; --brand:#2563eb; --ok:#10b981; --warn:#f59e0b; --danger:#ef4444;
  --line:#243043;
}
*{box-sizing:border-box} body{background:var(--bg);color:var(--ink);font-family:Inter,system-ui,-apple-system,Segoe UI,Roboto,Helvetica,Arial,sans-serif;margin:0;padding:16px}
.header{display:flex;align-items:center;justify-content:space-between;margin-bottom:10px}
h1{font-size:20px;margin:0} .mut{color:var(--mut)}
.card{background:var(--card);border:1px solid var(--line);border-radius:16px;padding:16px;margin:10px 0;box-shadow:0 8px 24px rgba(0,0,0,.25)}
.tabs{display:flex;gap:8px;flex-wrap:wrap;margin:10px 0}
.tab{padding:8px 12px;border-radius:10px;background:#0f1725;color:#b8ccff;cursor:pointer;border:1px solid #182235}
.tab.active{background:var(--brand);color:#fff;border-color:var(--brand)}
.grid{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:12px}
@media(max-width:980px){.grid{grid-template-columns:repeat(2,minmax(0,1fr))}}
@media(max-width:640px){.grid{grid-template-columns:1fr}}
.kv{padding:12px;border-radius:12px;background:#0e1730;border:1px solid #1a2540}
.kv b{display:block;color:#a8c6ff;margin-bottom:6px}
.btn{display:inline-block;background:var(--brand);border:1px solid var(--brand);color:#fff;padding:10px 12px;border-radius:10px;font-size:14px;text-decoration:none;margin:3px 6px 3px 0}
.btn.alt{background:#1c263b;border-color:#2a3956}
.btn.ok{background:var(--ok);border-color:var(--ok)}
.btn.warn{background:var(--warn);border-color:var(--warn);color:#1f1500}
.btn.danger{background:var(--danger);border-color:var(--danger)}
.row{display:flex;gap:12px;flex-wrap:wrap;align-items:center}
input[type=number],input[type=text]{width:160px;padding:8px;border-radius:10px;border:1px solid #2b3a58;background:#0f1725;color:var(--ink)}
label{display:inline-block;min-width:160px}
.badge{display:inline-block;padding:2px 8px;border-radius:999px;background:#0e1730;border:1px solid #203052;margin-left:6px;color:#c6d7ff}
hr{border:0;border-top:1px solid var(--line);margin:10px 0}
.hide{display:none}
.mono{font-family:ui-monospace,Menlo,Consolas,monospace}
.bigstop{display:inline-block;background:var(--danger);color:#fff;border:0;border-radius:999px;padding:14px 18px;font-size:16px;font-weight:700}
.dial{width:220px;height:220px;border-radius:50%;border:2px solid #2a3a5d;background:conic-gradient(from 0deg, #101a33, #0f172a);position:relative;margin:auto}
.dial .tick{position:absolute;left:50%;top:50%;width:2px;height:14px;background:#2f4066;transform-origin:50% 90%}
.dial .needle{position:absolute;left:50%;top:50%;width:3px;height:80px;background:#9ec1ff;transform-origin:50% 90%;border-radius:2px}
.dial .needle.qibla{height:90px;background:#10b981}
.dial .center{position:absolute;left:50%;top:50%;width:12px;height:12px;background:#fff;border-radius:50%;transform:translate(-50%,-50%)}
.count{font-size:28px;font-weight:700}
small{color:var(--mut)}
</style>
</head><body>
<div class="header">
  <h1>SalahSense <span class="mut">· Sunrise alarm only</span></h1>
  <div>
    <button class="bigstop" onclick="act('imup/set')">STOP (I'm up)</button>
  </div>
</div>

<div class="tabs">
  <div class="tab active" data-t="status">Status</div>
  <div class="tab" data-t="motor">Motor/Test</div>
  <div class="tab" data-t="clock">Clock/NTP</div>
  <div class="tab" data-t="fsr">FSR</div>
  <div class="tab" data-t="piezo">Piezo</div>
  <div class="tab" data-t="qibla">Qibla/Compass</div>
  <div class="tab" data-t="sun">Sunrise</div>
  <div class="tab" data-t="save">Save/Reset</div>
</div>

<!-- STATUS -->
<div id="status" class="card">
  <div class="grid">
    <div class="kv"><b>Date</b><div id="d">—</div></div>
    <div class="kv"><b>Time</b><div id="t">—</div></div>
    <div class="kv"><b>Lux</b><div id="lx">—</div></div>
    <div class="kv"><b>Network</b><div id="net">—</div></div>
    <div class="kv"><b>Mode</b><div id="md">—</div></div>
    <div class="kv"><b>Motor</b><div id="m">—</div></div>
  </div>
  <hr>
  <div class="row">
    <div class="kv" style="flex:1">
      <b>Sunrise wake (−offset)</b>
      <div>Sunrise: <span id="sr">--:--</span> • Alarm: <span id="sra">--:--</span></div>
      <div>Next in: <span id="sr_count" class="count">—</span></div>
    </div>
    <div class="kv" style="flex:1">
      <b>Bed & Piezo</b>
      <div class="mono" id="bp">…</div>
    </div>
    <div class="kv" style="flex:1">
      <b>Debug</b>
      <div class="mono" id="dbg">…</div>
    </div>
  </div>
</div>

<!-- MOTOR / TEST -->
<div id="motor" class="card hide">
  <h3>Motor & Manual Test</h3>
  <div class="row">
    <a class="btn warn" href="#" onclick="act('motor/mode/on')">Force ON</a>
    <a class="btn warn" href="#" onclick="act('motor/mode/off')">Force OFF</a>
    <a class="btn ok"   href="#" onclick="act('motor/mode/auto')">AUTO (FSR)</a>
    <a class="btn" href="#" onclick="act('motor/invert')">Toggle Invert</a>
    <input id="pwm" type="number" min="20" max="255" step="1" value="255">
    <a class="btn" href="#" onclick="setPWM()">Set Power</a>
  </div>
  <hr>
  <div class="row">
    <label>Test: fire alarm in</label>
    <input id="tsec" type="number" min="1" max="3600" value="10">
    <a class="btn" href="#" onclick="testIn()">Start in N sec</a>
    <a class="btn alt" href="#" onclick="act('alarm/stop')">Stop</a>
    <a class="btn" href="#" onclick="act('alarm/snooze')">Snooze 5 min</a>
  </div>
</div>

<!-- CLOCK -->
<div id="clock" class="card hide">
  <h3>Clock / NTP</h3>
  <div class="row">
    <a class="btn ok" href="#" onclick="rtcSetFromBrowser()">Set from this device</a>
    <a class="btn alt" href="#" onclick="act('rtc/ntp')">Sync via NTP</a>
    <span class="badge" id="ntp_age">—</span>
  </div>
  <div class="row" style="margin-top:8px">
    <label>UTC offset (minutes)</label><input id="tz" type="number" min="-720" max="720" value="-240">
    <a class="btn" href="#" onclick="setTZ()">Save offset</a>
    <small>EDT −240, EST −300</small>
  </div>
</div>

<!-- FSR -->
<div id="fsr" class="card hide">
  <h3>FSR (Bed)</h3>
  <div class="row">
    <a class="btn" href="#" onclick="act('fsr/cal/empty')">Calibrate: EMPTY</a>
    <a class="btn" href="#" onclick="act('fsr/cal/occupied')">Calibrate: OCCUPIED</a>
    <label>Hysteresis</label><input id="hys" type="number" min="5" max="200" step="1" value="40">
    <a class="btn" href="#" onclick="setHys()">Save</a>
  </div>
  <div class="mono" id="fsrdbg" style="margin-top:8px">…</div>
</div>

<!-- PIEZO -->
<div id="piezo" class="card hide">
  <h3>Piezo (tap = hard stop)</h3>
  <div class="row">
    <label>Threshold</label><input id="pthr" type="number" min="5" max="500" step="1" value="35">
    <a class="btn" href="#" onclick="setPThr()">Save</a>
    <a class="btn danger" href="#" onclick="act('imup/set')">Emulate STOP</a>
    <a class="btn" href="#" onclick="act('imup/clear')">Clear I'm up</a>
  </div>
  <div class="mono" id="pdbg" style="margin-top:8px">…</div>
</div>

<!-- QIBLA -->
<div id="qibla" class="card hide">
  <h3>Qibla / Compass</h3>
  <div class="row">
    <div style="flex:1">
      <div class="dial" id="dial">
        <div class="tick" style="transform:translate(-50%,-90%) rotate(0deg)"></div>
        <div class="tick" style="transform:translate(-50%,-90%) rotate(90deg)"></div>
        <div class="tick" style="transform:translate(-50%,-90%) rotate(180deg)"></div>
        <div class="tick" style="transform:translate(-50%,-90%) rotate(270deg)"></div>
        <div id="needleH" class="needle" style="transform:translate(-50%,-90%) rotate(0deg)"></div>
        <div id="needleQ" class="needle qibla" style="transform:translate(-50%,-90%) rotate(0deg)"></div>
        <div class="center"></div>
      </div>
    </div>
    <div style="flex:2">
      <div class="row">
        <label>Latitude</label><input id="lat" type="text" value="0">
        <label>Longitude</label><input id="lon" type="text" value="0">
        <a class="btn" href="#" onclick="setLatLon()">Save</a>
      </div>
      <div class="row" style="margin-top:8px">
        <label>Mag Declination</label><input id="decl" type="number" min="-30" max="30" step="0.1" value="0">
        <a class="btn" href="#" onclick="setDecl()">Save</a>
      </div>
      <div class="row" style="margin-top:8px">
        <a class="btn" href="#" onclick="act('mag/cal/start')">Start Cal</a>
        <a class="btn alt" href="#" onclick="act('mag/cal/stop')">Stop & Save</a>
        <a class="btn ok" href="#" onclick="act('qibla/override/set')">Use current heading as Qibla</a>
        <a class="btn" href="#" onclick="act('qibla/override/clear')">Clear override</a>
      </div>
      <div class="mono" id="qdbg" style="margin-top:8px">…</div>
    </div>
  </div>
</div>

<!-- SUNRISE -->
<div id="sun" class="card hide">
  <h3>Sunrise Alarm (only)</h3>
  <div class="row">
    <label>Use online Sunrise (WIA → Aladhan fallback)</label>
    <a class="btn" href="#" onclick="act('sun/toggle')">Toggle</a> <span id="sun_use" class="badge">—</span>
  </div>
  <div class="row" style="margin-top:8px">
    <label>Minutes BEFORE Sunrise</label>
    <input id="sun_off" type="number" min="0" max="120" step="1" value="20">
    <a class="btn" href="#" onclick="setSunOff()">Save</a>
    <a class="btn alt" href="#" onclick="act('sun/fetch')">Fetch Sunrise now</a>
  </div>
  <div class="mono" id="sunDbg" style="margin-top:8px">Sunrise: —  •  Alarm from sunrise: —</div>
</div>

<!-- SAVE / RESET -->
<div id="save" class="card hide">
  <h3>Save / Reset</h3>
  <div class="row">
    <a class="btn ok" href="#" onclick="act('save')">Save all settings</a>
    <a class="btn danger" href="#" onclick="act('reset')">Factory Reset (clear NVS)</a>
  </div>
</div>

<script>
const tabs=[...document.querySelectorAll('.tab')];
function show(t){ tabs.forEach(x=>x.classList.toggle('active',x.dataset.t===t));
  ['status','motor','clock','fsr','piezo','qibla','sun','save'].forEach(id=>{
    document.getElementById(id).classList.toggle('hide', id!==t);
  });
}
tabs.forEach(x=>x.addEventListener('click',()=>show(x.dataset.t)));
async function q(p){ const r=await fetch('/api/'+p); return r.json(); }
async function act(p){ await q(p); refresh(); }

async function setPWM(){ const v=parseInt(pwm.value||'255',10); await q(`motor/pwm?d=${v}`); }
async function setTZ(){ const v=parseInt(tz.value||'-240',10); await q(`rtc/set_tz?min=${v}`); }
async function rtcSetFromBrowser(){
  const d=new Date();
  const url=`rtc/set?y=${d.getFullYear()}&m=${d.getMonth()+1}&d=${d.getDate()}&hh=${d.getHours()}&mm=${d.getMinutes()}&ss=${d.getSeconds()}`;
  await q(url);
}
async function setHys(){ const v=parseInt(hys.value||'40',10); await q(`fsr/hys?x=${v}`); }
async function setPThr(){ const v=parseInt(pthr.value||'35',10); await q(`pz/set_thr?thr=${v}`); }
async function setLatLon(){ const la=encodeURIComponent(lat.value), lo=encodeURIComponent(lon.value); await q(`loc/set?lat=${la}&lon=${lo}`); }
async function setDecl(){ const v=parseFloat(decl.value||'0'); await q(`mag/decl?deg=${v}`); }
async function setSunOff(){ const v=parseInt(sun_off.value||'20',10); await q(`sun/set_off?m=${v}`); }
async function testIn(){ const v=parseInt(tsec.value||'10',10); await q(`test/in?sec=${v}`); }

function modeName(m){ return m===0?'AUTO':(m===1?'FORCE_ON':'FORCE_OFF'); }
function fmt2(n){ return (n<10?'0':'')+n; }
function secsToDHMS(sec){
  if (isNaN(sec) || sec<0) return '--:--:--';
  const d=Math.floor(sec/86400); sec%=86400;
  const h=Math.floor(sec/3600); sec%=3600;
  const m=Math.floor(sec/60); const s=sec%60;
  return (d>0?d+'d ':'')+fmt2(h)+':'+fmt2(m)+':'+fmt2(s);
}

async function refresh(){
  const s = await q('status');
  d.textContent = s.date; t.textContent = s.time;
  lx.textContent = s.lux.toFixed(1);
  net.textContent = s.net + ` • NTP ${s.ntp_age}s ago`;
  md.innerHTML = modeName(s.mode) + (s.imup?` <span class="badge">I'm up</span>`:'');
  m.textContent = (s.motor_on?'ON':'OFF') + ` @${s.motor_pwm}${s.motor_inv?' (HIGH)':' (LOW)'}`
  pwm.value = s.motor_pwm;

  tz.value = s.tz_off;
  pthr.value = s.pz_thr;
  pdbg.textContent = `raw:${s.pz_raw} avg:${Math.round(s.pz_avg)} dev:${Math.round(s.pz_dev)} peak:${Math.round(s.pz_peak)} thr:${s.pz_thr}`;

  hys.value = s.fsr_hys;
  fsrdbg.textContent = `raw:${s.fsr_raw} avg:${Math.round(s.fsr_avg)} thr:${s.fsr_thr} hys:${s.fsr_hys} occ:${s.fsr_occ?'YES':'NO'}`;
  bp.textContent = `Bed:${s.fsr_occ?'OCCUPIED':'EMPTY'} | Piezo dev:${Math.round(s.pz_dev)} thr:${s.pz_thr}`;

  lat.value = s.lat.toFixed(6);
  lon.value = s.lon.toFixed(6);
  decl.value = s.mag_decl.toFixed(1);

  if (s.mag_heading != null) document.getElementById('needleH').style.transform = `translate(-50%,-90%) rotate(${s.mag_heading}deg)`;
  if (s.qibla != null)       document.getElementById('needleQ').style.transform = `translate(-50%,-90%) rotate(${s.qibla}deg)`;
  qdbg.textContent = `heading:${(s.mag_heading==null?'--':s.mag_heading.toFixed(1))}°  qibla:${(s.qibla==null?'--':s.qibla.toFixed(1))}°  `
                    + (s.qibla_delta==null?'':`Δ:${s.qibla_delta.toFixed(1)}°`) + (s.qibla_ovr?'  [OVERRIDE]':'');

  sun_off.value = s.sun_off;
  sun_use.textContent = s.sun_use ? 'ONLINE' : 'OFF';
  sr.textContent = s.sunrise;
  sra.textContent = s.sunrise_alarm;
  sunDbg.textContent = `Sunrise: ${s.sunrise}  •  Alarm from sunrise: ${s.sunrise_alarm}`;

  if (s.sunrise_alarm && s.sunrise_alarm.includes(':')) {
    const now = new Date();
    const parts = s.sunrise_alarm.split(':');
    let goal = new Date(now.getFullYear(), now.getMonth(), now.getDate(), parseInt(parts[0]), parseInt(parts[1]), 0);
    let diff = Math.floor((goal - now)/1000);
    if (diff < -60) diff = NaN;
    sr_count.textContent = secsToDHMS(diff);
  }
}
setInterval(refresh, 1000); refresh();

</script>
</body></html>
)HTML";

/* ============== JSON & API ============== */
String jsonStatus(){
  DateTime now=rtc.now(); float lx=lux.readLightLevel();
  String net = onSTA ? (String("STA IP: ")+WiFi.localIP().toString())
                     : (String("AP IP: ")+WiFi.softAPIP().toString()+" (SSID: SalahSense)");
  uint32_t age = lastNtpMillis ? (uint32_t)((millis()-lastNtpMillis)/1000) : 999999;

  float qDelta = NAN;
  if (!isnan(magHeadingDeg) && !isnan(qiblaBearingDeg)){
    float d = qiblaBearingDeg - magHeadingDeg;
    while(d<-180) d+=360; while(d>180) d-=360;
    qDelta = d;
  }

  String s; s.reserve(1400);
  s+='{';
  s+=F("\"date\":\""); s+=fmtDate(now); s+=F("\",");
  s+=F("\"time\":\""); s+=fmtClock(now); s+=F("\",");
  s+=F("\"lux\":"); s+=String(lx,1); s+=F(",");
  s+=F("\"net\":\""); s+=net; s+=F("\",");

  s+=F("\"fsr_raw\":"); s+=fsr_raw; s+=F(",");
  s+=F("\"fsr_avg\":"); s+=String(fsr_avg,1); s+=F(",");
  s+=F("\"fsr_thr\":"); s+=fsr_thresh; s+=F(",");
  s+=F("\"fsr_hys\":"); s+=fsr_hys; s+=F(",");
  s+=F("\"fsr_occ\":"); s+=(bedOccupied?F("true"):F("false")); s+=F(",");

  s+=F("\"pz_raw\":"); s+=pz_raw; s+=F(",");
  s+=F("\"pz_avg\":"); s+=String(pz_avg,1); s+=F(",");
  s+=F("\"pz_dev\":"); s+=String(pz_dev,1); s+=F(",");
  s+=F("\"pz_peak\":"); s+=String(pz_peak,1); s+=F(",");
  s+=F("\"pz_thr\":"); s+=PZ_THR; s+=F(",");

  s+=F("\"mag_x\":"); s+=magX; s+=F(",");
  s+=F("\"mag_y\":"); s+=magY; s+=F(",");
  s+=F("\"mag_z\":"); s+=magZ; s+=F(",");
  s+=F("\"mag_heading\":"); if(isnan(magHeadingDeg)) s+=F("null"); else s+=String(magHeadingDeg,1); s+=F(",");
  s+=F("\"mag_decl\":"); s+=String(magDeclinationDeg,1); s+=F(",");

  s+=F("\"lat\":"); s+=String(userLat,6); s+=F(",");
  s+=F("\"lon\":"); s+=String(userLon,6); s+=F(",");
  s+=F("\"qibla\":"); if(isnan(qiblaBearingDeg)) s+=F("null"); else s+=String(qiblaBearingDeg,1); s+=F(",");
  s+=F("\"qibla_delta\":"); if(isnan(qDelta)) s+=F("null"); else s+=String(qDelta,1); s+=F(",");
  s+=F("\"qibla_ovr\":"); s+=(qiblaOverrideEn?F("true"):F("false")); s+=F(",");

  s+=F("\"motor_on\":"); s+=(motor_is_on()?F("true"):F("false")); s+=F(",");
  s+=F("\"motor_inv\":"); s+=(motorActiveHigh?F("true"):F("false")); s+=F(",");
  s+=F("\"motor_pwm\":"); s+=motorPower; s+=F(",");
  s+=F("\"mode\":"); s+=(int)mode; s+=F(",");
  s+=F("\"imup\":"); s+=(imUp?F("true"):F("false")); s+=F(",");

  s+=F("\"sun_use\":"); s+=(sunriseUse?F("true"):F("false")); s+=F(",");
  s+=F("\"sun_off\":"); s+=sunriseOffsetMin; s+=F(",");
  s+=F("\"sunrise\":\""); s+=(sunriseTodayStr.length()?sunriseTodayStr:String("--:--")); s+=F("\",");
  s+=F("\"sunrise_alarm\":\""); s+=(sunriseAlarmStr.length()?sunriseAlarmStr:String("--:--")); s+=F("\",");

  s+=F("\"tz_off\":"); s+=tz_offset_min; s+=F(",");
  s+=F("\"ntp_age\":"); s+=age; s+=F(",");
  s+=F("\"uptime\":"); s+=(uint32_t)(millis()/1000); s+=F(",");
  s+=F("\"heap\":"); s+=ESP.getFreeHeap();
  s+='}';
  return s;
}

void sendPage(){ http.send_P(200,"text/html",PAGE); }
void sendJSON(const String& j){ http.send(200,"application/json",j); }

/* Handlers */
void apiStatus(){ sendJSON(jsonStatus()); }
void apiSave(){ fsr_save(); piezo_save(); tz_save(); motor_save(); mag_save(); sunrise_save(); sendJSON(String("{\"ok\":true}")); }
void apiReset(){ prefs.end(); nvs_flash_erase(); nvs_flash_init(); sendJSON(String("{\"reset\":true}")); delay(400); ESP.restart(); }
void apiReboot(){ sendJSON(String("{\"reboot\":true}")); delay(300); ESP.restart(); }

/* Motor */
void apiMotorInvert(){ motorActiveHigh=!motorActiveHigh; motor_applyHW(); motor_save(); sendJSON(jsonStatus()); }
void apiModeAuto(){ mode=AUTO;  sendJSON(jsonStatus()); }
void apiModeOn(){   mode=FORCE_ON;  sendJSON(jsonStatus()); }
void apiModeOff(){  mode=FORCE_OFF; sendJSON(jsonStatus()); }
void apiMotorPWM(){ int d=constrain(http.arg("d").toInt(),20,255); motorPower=(uint8_t)d; motor_applyHW(); motor_save(); sendJSON(jsonStatus()); }

/* I'm up (hard stop) */
void apiImUpSet(){ imUp=true; imUp_until_ms=millis()+IMUP_TIMEOUT_MS; motor_off(); alarm_stop(); sendJSON(jsonStatus()); }
void apiImUpClear(){ imUp=false; imUp_until_ms=0; sendJSON(jsonStatus()); }

/* FSR */
void apiFSREmpty(){ fsr_empty=(int)lrintf(fsr_avg); fsr_recalc(); fsr_save(); sendJSON(jsonStatus()); }
void apiFSROcc(){   fsr_occupied=(int)lrintf(fsr_avg); fsr_recalc(); fsr_save(); sendJSON(jsonStatus()); }
void apiFSRHys(){   fsr_hys = constrain(http.arg("x").toInt(),5,200); fsr_save(); sendJSON(jsonStatus()); }

/* Piezo */
void apiPZSetThr(){ int v=constrain(http.arg("thr").toInt(),5,500); PZ_THR=v; piezo_save(); sendJSON(jsonStatus()); }

/* Clock / NTP */
void apiRTCSet(){
  int y=http.arg("y").toInt(), m=http.arg("m").toInt(), d=http.arg("d").toInt();
  int hh=http.arg("hh").toInt(), mm=http.arg("mm").toInt(), ss=http.arg("ss").toInt();
  if(y>=2020 && m>=1 && d>=1) rtc.adjust(DateTime(y,m,d,hh,mm,ss));
  sendJSON(jsonStatus());
}
void apiRTCNTP(){ ntp_sync_local(); sendJSON(jsonStatus()); }
void apiRTCSetTZ(){ tz_offset_min = constrain(http.arg("min").toInt(), -720, 720); tz_save(); sendJSON(jsonStatus()); }

/* Alarm control */
void apiAlarmStop(){ alarm_stop(); sendJSON(jsonStatus()); }
void apiAlarmSnooze(){ alarm_snooze(); sendJSON(jsonStatus()); }

/* Manual test one-shot */
void apiTestIn(){
  // Fix: ensure both arguments to std::max are of the same type
  // Alternative: cast http.arg("sec").toInt() to long
  // uint32_t sec = (uint32_t)std::max(1L, (long)http.arg("sec").toInt());
  uint32_t sec = (uint32_t)std::max(1L, static_cast<long>(http.arg("sec").toInt()));
  testOneShotStartAtMs = millis() + sec*1000UL;
  testAutoStopAfterMs  = 0; // keep buzzing until stop/snooze/imUp/bed-empty
  sendJSON(jsonStatus());
}

/* Magnetometer + Qibla */
void apiMagCalStart(){ magcal.running=true; magcal.clearMinMax(); sendJSON(jsonStatus()); }
void apiMagCalStop(){
  magcal.running=false;
  if (magcal.xmax>magcal.xmin) magcal.xoff = (int16_t)((magcal.xmin + magcal.xmax)/2);
  if (magcal.ymax>magcal.ymin) magcal.yoff = (int16_t)((magcal.ymin + magcal.ymax)/2);
  if (magcal.zmax>magcal.zmin) magcal.zoff = (int16_t)((magcal.zmin + magcal.zmax)/2);
  mag_save(); qibla_update(); sendJSON(jsonStatus());
}
void apiMagDecl(){ magDeclinationDeg = constrain(http.arg("deg").toFloat(), -30.0f, 30.0f); mag_save(); qibla_update(); sendJSON(jsonStatus()); }
void apiLocSet(){ userLat = http.arg("lat").toFloat(); userLon = http.arg("lon").toFloat(); mag_save(); qibla_update(); sendJSON(jsonStatus()); }
void apiQiblaOverrideSet(){ if (!isnan(magHeadingDeg)){ qiblaOverrideEn=true; qiblaOverrideDeg=magHeadingDeg; mag_save(); qibla_update(); } sendJSON(jsonStatus()); }
void apiQiblaOverrideClear(){ qiblaOverrideEn=false; qiblaOverrideDeg=NAN; mag_save(); qibla_update(); sendJSON(jsonStatus()); }

/* Sunrise endpoints */
void apiSunToggle(){ sunriseUse = !sunriseUse; sunrise_save(); sendJSON(jsonStatus()); }
void apiSunSetOff(){
  sunriseOffsetMin = constrain(http.arg("m").toInt(), 0, 120); sunrise_save();
  if (sunriseTodayStr.length()>=4){
    int sh=sunriseTodayStr.substring(0,2).toInt(); int sm=sunriseTodayStr.substring(3).toInt();
    applySunriseToAlarm((uint8_t)sh,(uint8_t)sm);
  }
  sendJSON(jsonStatus());
}
void apiSunFetch(){
  uint8_t sh=0, sm=0;
  bool ok = fetchSunrise_WIA(sh,sm);
  if (!ok) ok = fetchSunrise_Aladhan(sh,sm);
  if (ok) applySunriseToAlarm(sh,sm);
  sendJSON(jsonStatus());
}

/* ============== Setup / Loop ============== */
static uint32_t lastPrint=0;

void setup(){
  Serial.begin(115200); delay(200);

  prefs.begin("salahsense", false);
  tz_load(); fsr_load(); piezo_load(); motor_load(); mag_load(); sunrise_load();

  Wire.begin(SDA_PIN, SCL_PIN);
  if(!rtc.begin()){ Serial.println("RTC not found"); while(1){} }
  if(!lux.begin(BH1750::CONTINUOUS_HIGH_RES_MODE)){ Serial.println("BH1750 init failed"); }
  if(qmc.begin(Wire)) Serial.println("QMC5883L ready"); else Serial.println("QMC5883L not found");

  motor_ledc_setup(); motor_applyHW();
  fsr_begin(); piezo_begin();
  qibla_update();

  if(connectSTA()){ Serial.print("Wi-Fi STA: "); Serial.println(WiFi.localIP()); ntp_sync_local(); }
  else { startAP(); Serial.print("AP IP: "); Serial.println(WiFi.softAPIP()); }

  // mDNS for easier discovery (http://salahsense.local)
  if (MDNS.begin("salahsense")) {
    MDNS.addService("http", "tcp", 80);
    Serial.println("mDNS responder started: http://salahsense.local");
  } else {
    Serial.println("mDNS start failed");
  }

  // Routes
  http.on("/", sendPage);
  http.on("/api/status", apiStatus);
  http.on("/api/save",   apiSave);
  http.on("/api/reset",  apiReset);
  http.on("/api/reboot", apiReboot);

  http.on("/api/motor/invert", apiMotorInvert);
  http.on("/api/motor/mode/auto", apiModeAuto);
  http.on("/api/motor/mode/on",   apiModeOn);
  http.on("/api/motor/mode/off",  apiModeOff);
  http.on("/api/motor/pwm",       apiMotorPWM);

  http.on("/api/imup/set",   apiImUpSet);
  http.on("/api/imup/clear", apiImUpClear);

  http.on("/api/fsr/cal/empty", apiFSREmpty);
  http.on("/api/fsr/cal/occupied", apiFSROcc);
  http.on("/api/fsr/hys", apiFSRHys);

  http.on("/api/pz/set_thr", apiPZSetThr);

  http.on("/api/rtc/set", apiRTCSet);
  http.on("/api/rtc/ntp", apiRTCNTP);
  http.on("/api/rtc/set_tz", apiRTCSetTZ);

  http.on("/api/alarm/stop",   apiAlarmStop);
  http.on("/api/alarm/snooze", apiAlarmSnooze);

  http.on("/api/test/in", apiTestIn);

  http.on("/api/mag/cal/start", apiMagCalStart);
  http.on("/api/mag/cal/stop",  apiMagCalStop);
  http.on("/api/mag/decl",      apiMagDecl);
  http.on("/api/loc/set",       apiLocSet);
  http.on("/api/qibla/override/set",   apiQiblaOverrideSet);
  http.on("/api/qibla/override/clear", apiQiblaOverrideClear);

  http.on("/api/sun/toggle",  apiSunToggle);
  http.on("/api/sun/set_off", apiSunSetOff);
  http.on("/api/sun/fetch",   apiSunFetch);

  http.begin();

  // If using sunrise, fetch once at boot
  if (sunriseUse) {
    uint8_t sh, sm;
    if (fetchSunrise_WIA(sh,sm) || fetchSunrise_Aladhan(sh,sm)) applySunriseToAlarm(sh,sm);
  }

  Serial.println("SalahSense (sunrise-only) ready.");
}

void loop(){
  http.handleClient();

  DateTime now = rtc.now();
  fsr_read();
  piezo_read();
  mag_update();
  motor_logic_update(now);

  if (millis()-lastPrint>1000){
    float lx=lux.readLightLevel();
    Serial.printf("T=%s Lux=%.1f FSR:%u/%.0f thr=%d hys=%d %s  PZ dev=%.0f thr=%d  Motor:%s pwm=%u inv:%s  Mode:%d  ImUp:%d  Sun:%s -> %s use:%d\n",
      fmtClock(now).c_str(), lx, fsr_raw, fsr_avg, fsr_thresh, fsr_hys, bedOccupied?"OCC":"EMP",
      pz_dev, PZ_THR, motor_is_on()?"ON":"OFF", motorPower, motorActiveHigh?"HIGH":"LOW",
      (int)mode, (int)imUp, sunriseTodayStr.c_str(), sunriseAlarmStr.c_str(), sunriseUse);
    lastPrint=millis();
  }
}
