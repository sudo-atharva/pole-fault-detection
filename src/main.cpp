#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <TinyGPSPlus.h>
#include <WiFi.h>
#include <WebServer.h>
#include <LittleFS.h>

// ---- pins ----
#define RELAY_PIN     25   // AC relay (line contactor)
#define ACS712_PIN    34   // AC current sensor, ADC1_CH6
#define ZMPT_PIN      35   // AC voltage sensor (ZMPT101B), ADC1_CH7 -- pin not given by user, assumed free ADC1 input
#define SDA_PIN       21
#define SCL_PIN       22
#define SIM_RX_PIN    27   // ESP32 RX1 <- SIM800L TX
#define SIM_TX_PIN    26   // ESP32 TX1 -> SIM800L RX (through divider)
#define GPS_RX_PIN    16   // ESP32 RX2 <- NEO-6M TX
#define GPS_TX_PIN    17   // ESP32 TX2 -> NEO-6M RX

#define RELAY_ACTIVE_LOW false // ponytail: flip if your relay module trips backwards

// ---- calibration knobs (tune per hardware, mains is never textbook 230V/50Hz) ----
const float ADC_MAX          = 4095.0f;
const float ADC_VREF         = 3.3f;
const float ACS712_OFFSET_V  = 1.65f;   // sensor bias at 0A, measure yours and adjust
const float ACS712_SENS_V_PER_A = 0.100f; // 100mV/A for 20A module; change for your module (66/100/185 mV/A)
const float ZMPT_OFFSET_V    = 1.65f;   // sensor bias at 0V, measure yours and adjust
const float ZMPT_SENS_V_PER_V = 0.0090f; // volts-out per volt-mains, calibrate against a known mains reading

const float CURRENT_TRIP_A   = 15.0f;   // short-circuit / overcurrent trip
const float VOLTAGE_LOW_V    = 180.0f;  // undervoltage (monitoring only, no SMS requested)
const float VOLTAGE_HIGH_V   = 260.0f;  // overvoltage trip
const float TILT_TRIP_DEG    = 25.0f;   // pole lean from install baseline
const int   FAULT_DEBOUNCE   = 3;       // consecutive bad samples before tripping
const unsigned long SAMPLE_PERIOD_MS = 500;

#define ALERT_NUMBER "+910000000000" // put real recipient number here
#define AP_SSID "PoleFaultAP"
#define AP_PASS "12345678"           // WPA2 needs >=8 chars

Adafruit_MPU6050 mpu;
TinyGPSPlus gps;
HardwareSerial simSerial(1);
HardwareSerial gpsSerial(2);
WebServer server(80);

float tiltBaselineDeg = 0.0f;
int tiltStreak = 0, overVStreak = 0, overCStreak = 0;
bool tripped = false;
String tripReason = "";

// latest readings, shown on the AP dashboard
float lastVoltage = 0, lastCurrent = 0, lastTiltDelta = 0;

void relaySet(bool closed) {
  digitalWrite(RELAY_PIN, (closed != RELAY_ACTIVE_LOW) ? HIGH : LOW);
}

// RMS over ~5 mains cycles (assumes 50Hz; adjust window for 60Hz mains)
float readRmsVoltage(int pin, float offsetV, float sensVPerV) {
  const int samples = 500;
  double sumSq = 0;
  unsigned long start = millis();
  int n = 0;
  while (millis() - start < 100 && n < samples) {
    float v = analogRead(pin) * (ADC_VREF / ADC_MAX);
    float centered = v - offsetV;
    sumSq += (double)centered * centered;
    n++;
  }
  float rmsV = sqrt(sumSq / n);
  return rmsV / sensVPerV;
}

float readMainsVoltage() {
  return readRmsVoltage(ZMPT_PIN, ZMPT_OFFSET_V, ZMPT_SENS_V_PER_V);
}

float readMainsCurrent() {
  return readRmsVoltage(ACS712_PIN, ACS712_OFFSET_V, ACS712_SENS_V_PER_A);
}

float readTiltDeg() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float ax = a.acceleration.x, ay = a.acceleration.y, az = a.acceleration.z;
  return atan2(sqrt(ax * ax + ay * ay), az) * 180.0f / PI;
}

void sendSMS(const String &msg) {
  simSerial.print("AT+CMGF=1\r");
  delay(200);
  simSerial.print("AT+CMGS=\"" ALERT_NUMBER "\"\r");
  delay(200);
  simSerial.print(msg);
  simSerial.write(0x1A); // Ctrl+Z sends
  delay(2000);
}

String locationString() {
  while (gpsSerial.available()) gps.encode(gpsSerial.read());
  if (gps.location.isValid()) {
    return "https://maps.google.com/?q=" + String(gps.location.lat(), 6) +
           "," + String(gps.location.lng(), 6);
  }
  return "GPS: no fix";
}

void raiseFault(const String &reason) {
  relaySet(false); // isolate the line
  tripped = true;
  tripReason = reason;
  String msg = "POLE FAULT: " + reason + "\n" + locationString();
  sendSMS(msg);
  Serial.println(msg);
}

void handleData() {
  String loc = "no fix";
  if (gps.location.isValid()) {
    loc = String(gps.location.lat(), 6) + "," + String(gps.location.lng(), 6);
  }
  String json = "{";
  json += "\"voltage\":" + String(lastVoltage, 1) + ",";
  json += "\"current\":" + String(lastCurrent, 2) + ",";
  json += "\"tilt\":" + String(lastTiltDelta, 1) + ",";
  json += "\"relay\":\"" + String(tripped ? "OPEN" : "CLOSED") + "\",";
  json += "\"tripped\":" + String(tripped ? "true" : "false") + ",";
  json += "\"reason\":\"" + tripReason + "\",";
  json += "\"gps\":\"" + loc + "\"";
  json += "}";
  server.send(200, "application/json", json);
}

void setup() {
  Serial.begin(115200);

  pinMode(RELAY_PIN, OUTPUT);
  relaySet(true); // start energized

  Wire.begin(SDA_PIN, SCL_PIN);
  if (!mpu.begin()) {
    Serial.println("MPU6050 not found");
  } else {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  }

  simSerial.begin(9600, SERIAL_8N1, SIM_RX_PIN, SIM_TX_PIN);
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  delay(1000); // let SIM800L settle
  simSerial.print("AT\r");

  // baseline tilt: pole assumed vertical & undisturbed at install time
  delay(200);
  tiltBaselineDeg = readTiltDeg();

  LittleFS.begin(true); // format on first boot if no filesystem yet
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());

  server.serveStatic("/", LittleFS, "/index.html");
  server.on("/data", handleData);
  server.begin();
}

void loop() {
  server.handleClient();

  // debug passthrough: type AT commands into Serial Monitor to talk to
  // SIM800L directly, and watch raw NMEA from the GPS go by
  while (Serial.available()) simSerial.write(Serial.read());
  while (simSerial.available()) Serial.write(simSerial.read());
  while (gpsSerial.available()) {
    char c = gpsSerial.read();
    gps.encode(c);
    Serial.write(c);
  }

  static unsigned long lastSample = 0;
  if (millis() - lastSample < SAMPLE_PERIOD_MS) return;
  lastSample = millis();

  lastVoltage = readMainsVoltage();
  lastCurrent = readMainsCurrent();
  float tiltDeg = readTiltDeg();
  lastTiltDelta = fabs(tiltDeg - tiltBaselineDeg);

  Serial.printf("V=%.1f I=%.2f tilt=%.1f\n", lastVoltage, lastCurrent, lastTiltDelta);

  if (tripped) return; // latched until manually reset (power cycle / reset button)

  bool tiltBad = lastTiltDelta > TILT_TRIP_DEG;
  bool overVBad = lastVoltage > VOLTAGE_HIGH_V;
  bool overCBad = lastCurrent > CURRENT_TRIP_A;

  tiltStreak  = tiltBad  ? tiltStreak + 1  : 0;
  overVStreak = overVBad ? overVStreak + 1 : 0;
  overCStreak = overCBad ? overCStreak + 1 : 0;

  if (overCStreak >= FAULT_DEBOUNCE) {
    raiseFault("short circuit " + String(lastCurrent, 1) + "A");
  } else if (tiltStreak >= FAULT_DEBOUNCE) {
    raiseFault("pole down, tilt " + String(lastTiltDelta, 1) + "deg");
  } else if (overVStreak >= FAULT_DEBOUNCE) {
    raiseFault("overvoltage " + String(lastVoltage, 1) + "V");
  } else if (lastVoltage < VOLTAGE_LOW_V) {
    Serial.println("undervoltage (monitoring only, no trip)");
  }
}
