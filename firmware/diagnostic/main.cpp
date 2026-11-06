// Bench diagnostics only: ENABLE is permanently LOW. No heater-control mode.
#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <esp_arduino_version.h>
#include <math.h>
#include "BenchPolicy.h"

namespace {
WebServer web(80);
constexpr uint8_t adsAddress = 0x49, tmpAddress = 0x48, bhAddress = 0x23;
constexpr uint8_t pwmChannel = 0;
bool adsPresent = false, tmpPresent = false, bhPresent = false, lcdReady = false;
bool adcValid = false, tmpValid = false, lightValid = false, probe = false, pwmReady = false;
bool adcPending = false, backlight = true;
uint8_t lcdAddress = 0, probePercent = 0;
uint32_t adcStarted = 0, lastSample = 0, lastActivity = 0, probeStarted = 0;
int32_t adcRaw = 0, encoderTicks = 0;
float adcMv = NAN, ambientC = NAN, lux = NAN;
uint8_t previousEncoder = 0;
bool previousButton = false;
String command;

void inhibit() {
    // Never write HIGH to this pin, including probe, error and timeout paths.
    digitalWrite(bench::enablePin, LOW);
}
void pwmWrite(uint8_t duty) {
    if (!pwmReady) return;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(bench::pwmPin, duty);
#else
    ledcWrite(pwmChannel, duty);
#endif
}
void stopProbe() {
    inhibit(); pwmWrite(0); probe = false; probePercent = 0;
}
bool ack(uint8_t address) {
    Wire.beginTransmission(address);
    return Wire.endTransmission() == 0;
}
bool writeByte(uint8_t address, uint8_t value) {
    Wire.beginTransmission(address); Wire.write(value);
    return Wire.endTransmission() == 0;
}
bool writeRegister(uint8_t address, uint8_t reg, uint16_t value) {
    Wire.beginTransmission(address); Wire.write(reg);
    Wire.write(uint8_t(value >> 8)); Wire.write(uint8_t(value));
    return Wire.endTransmission() == 0;
}
bool readRegister(uint8_t address, uint8_t reg, uint16_t &value) {
    Wire.beginTransmission(address); Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom(address, uint8_t(2)) != 2) return false;
    value = uint16_t(Wire.read()) << 8; value |= uint8_t(Wire.read());
    return true;
}
void lcdNibble(uint8_t upper, bool data) {
    const uint8_t value = (upper & 0xf0) | (data ? 1 : 0) | (backlight ? 8 : 0);
    writeByte(lcdAddress, value); writeByte(lcdAddress, value | 4);
    delayMicroseconds(1); writeByte(lcdAddress, value); delayMicroseconds(40);
}
void lcdByte(uint8_t value, bool data = false) {
    lcdNibble(value, data); lcdNibble(uint8_t(value << 4), data);
}
void lcdLine(uint8_t row, const String &text) {
    const uint8_t offsets[] = {0x00, 0x40, 0x14, 0x54};
    lcdByte(0x80 | offsets[row]);
    for (uint8_t i = 0; i < 20; ++i) lcdByte(i < text.length() ? text[i] : ' ', true);
}
void discover() {
    stopProbe(); adcPending = false;
    adsPresent = ack(adsAddress); tmpPresent = ack(tmpAddress); bhPresent = ack(bhAddress);
    adcValid = tmpValid = lightValid = false;
    if (bhPresent) { writeByte(bhAddress, 0x01); writeByte(bhAddress, 0x10); }
    // Freenove PCF8574 mapping: P0 RS, P1 RW, P2 E, P3 backlight, P4..7 data.
    lcdAddress = ack(0x27) ? 0x27 : ack(0x3f) ? 0x3f : 0;
    lcdReady = lcdAddress != 0;
    if (lcdReady) {
        delay(50); lcdNibble(0x30, false); delay(5);
        lcdNibble(0x30, false); delay(1); lcdNibble(0x30, false);
        lcdNibble(0x20, false); lcdByte(0x28); lcdByte(0x0c);
        lcdByte(0x01); delay(2); lcdByte(0x06);
    }
    lastSample = millis();
}
void scan() {
    stopProbe(); Serial.println("I2C ACK addresses (not proof of device identity):");
    for (uint8_t a = 8; a < 0x78; ++a) if (ack(a)) Serial.printf("  0x%02X\n", a);
    discover();
}
String numberOrNull(float number, bool valid) {
    return valid && isfinite(number) ? String(number, 3) : String("null");
}
String status() {
    String s; s.reserve(640);
    s = "{\"firmware\":\"bench-diagnostic-1\",\"heater_enable_command\":0,";
    s += "\"physical_heater_state_verified\":false,\"uptime_ms\":" + String(millis());
    s += ",\"fault_n\":" + String(digitalRead(bench::faultPin));
    s += ",\"sda_level\":" + String(digitalRead(21)) + ",\"scl_level\":" + String(digitalRead(22));
    s += ",\"encoder_ticks\":" + String(encoderTicks);
    s += ",\"button_pressed\":" + String(digitalRead(bench::encoderButton) == LOW ? "true" : "false");
    s += ",\"ads_ack\":" + String(adsPresent ? "true" : "false");
    s += ",\"adc_valid\":" + String(adcValid ? "true" : "false");
    s += ",\"adc_raw\":" + String(adcRaw) + ",\"adc_mV\":" + numberOrNull(adcMv, adcValid);
    s += ",\"ambient_C\":" + numberOrNull(ambientC, tmpValid);
    s += ",\"illuminance_lx\":" + numberOrNull(lux, lightValid);
    s += ",\"lcd_address\":" + String(lcdAddress);
    s += ",\"probe_pwm_percent\":" + String(probe ? probePercent : 0) + "}";
    return s;
}
void sample() {
    const uint32_t now = millis();
    if (adcPending && uint32_t(now - adcStarted) >= 12) {
        uint16_t config = 0, raw = 0;
        // Require OS ready and the expected conversion configuration.
        adcValid = readRegister(adsAddress, 1, config) && (config & 0x8000) &&
                   (config & 0x7fff) == 0x4383 && readRegister(adsAddress, 0, raw);
        if (adcValid) { adcRaw = bench::signedAdc(raw); adcMv = adcRaw * 0.125f; }
        else adcMv = NAN;
        adcPending = false;
    }
    if (uint32_t(now - lastSample) < 1000) return;
    lastSample = now;
    uint16_t raw = 0;
    tmpValid = tmpPresent && readRegister(tmpAddress, 0, raw);
    if (tmpValid) ambientC = bench::signedAdc(raw) / 128.0f;
    lightValid = false;
    if (bhPresent && Wire.requestFrom(bhAddress, uint8_t(2)) == 2) {
        raw = uint16_t(Wire.read()) << 8; raw |= uint8_t(Wire.read());
        lux = raw / 1.2f; lightValid = true;
    }
    // ADS1115: AIN0/GND, +/-4.096V range, single shot,128SPS, comparator disabled.
    // FAULT_N is therefore a raw input observation, not an enabled ADC trip.
    adcValid = false;
    adcPending = adsPresent && writeRegister(adsAddress, 1, 0xc383);
    adcStarted = now;
    if (lcdReady) {
        lcdLine(0, "LAB DIAG - HEAT OFF");
        lcdLine(1, isfinite(adcMv) ? "ADC0 " + String(adcMv, 1) + " mV" : "ADC0 not measured");
        lcdLine(2, tmpValid ? "TMP " + String(ambientC, 2) + " C" : "TMP117 absent/error");
        lcdLine(3, "Enc " + String(encoderTicks) + (digitalRead(bench::encoderButton) == LOW ? " BTN" : ""));
    }
}
void userInput() {
    static constexpr int8_t transitions[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
    const uint8_t state = (digitalRead(bench::encoderA) << 1) | digitalRead(bench::encoderB);
    const bool button = digitalRead(bench::encoderButton) == LOW;
    const int8_t step = transitions[(previousEncoder << 2) | state];
    encoderTicks += step; previousEncoder = state;
    if (step || (button && !previousButton)) {
        lastActivity = millis(); backlight = true;
        if (lcdReady) writeByte(lcdAddress, 8);
    }
    previousButton = button;
    if (backlight && uint32_t(millis() - lastActivity) >= 60000) {
        backlight = false; if (lcdReady) writeByte(lcdAddress, 0);
    }
}
void serialCommand() {
    while (Serial.available()) {
        const char c = char(Serial.read());
        if (c == '\r') continue;
        if (c != '\n') {
            if (command.length() < 64) command += c;
            else { command = ""; stopProbe(); }
            continue;
        }
        command.trim();
        if (command == "stop") stopProbe();
        else if (command == "status") Serial.println(status());
        else if (command == "scan") scan();
        else if (command.startsWith("probe ")) {
            const String value = command.substring(6); bool digits = value.length() > 0;
            for (unsigned i = 0; i < value.length(); ++i) digits &= value[i] >= '0' && value[i] <= '9';
            const int percent = digits ? value.toInt() : -1;
            stopProbe();
            if (pwmReady && percent >= 1 && percent <= bench::maxProbePercent) {
                probeStarted = millis(); probePercent = uint8_t(percent); probe = true;
                pwmWrite(bench::dutyForPercent(probePercent));
                Serial.println("Logic PWM only,500Hz,2s. ENABLE stays LOW. Verify Q1 gate physically.");
            } else Serial.println("Rejected. probe accepts1..20 percent; ENABLE stays LOW.");
        } else Serial.println("Commands: status | scan | stop | probe <1..20> (logic only,2s)");
        command = "";
    }
}
const char page[] PROGMEM = R"HTML(<!doctype html><html lang="fr"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>Diagnostic labo</title>
<style>body{font:18px system-ui;max-width:850px;margin:2em auto;padding:1em}pre{white-space:pre-wrap;background:#eef3f6;padding:1em}</style>
<h1>Diagnostic A / B température</h1><p>ENABLE maintenu à zéro. Aucun mode de commande du chauffage.</p>
<p>Cette page observe les signaux ; vérifier au multimètre que la puissance reste coupée.</p><pre id="s">Lecture…</pre>
<script>async function poll(){try{let r=await fetch('/status');if(!r.ok)throw Error(r.status);document.getElementById('s').textContent=JSON.stringify(await r.json(),null,2)}catch(e){document.getElementById('s').textContent='Connexion perdue — mesures indisponibles'}}setInterval(poll,1000);poll()</script></html>)HTML";
}

void setup() {
    digitalWrite(bench::enablePin, LOW); pinMode(bench::enablePin, OUTPUT); inhibit();
    digitalWrite(bench::pwmPin, LOW); pinMode(bench::pwmPin, OUTPUT);
    Serial.begin(115200); command.reserve(65);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    pwmReady = ledcAttach(bench::pwmPin, 500, 8);
#else
    pwmReady = ledcSetup(pwmChannel, 500, 8) > 0;
    if (pwmReady) ledcAttachPin(bench::pwmPin, pwmChannel);
#endif
    stopProbe();
    pinMode(bench::faultPin, INPUT);  // External R7 pull-up on B; floating if disconnected.
    pinMode(bench::encoderA, INPUT_PULLUP); pinMode(bench::encoderB, INPUT_PULLUP);
    pinMode(bench::encoderButton, INPUT_PULLUP);
    previousEncoder = (digitalRead(bench::encoderA) << 1) | digitalRead(bench::encoderB);
    Wire.begin(21, 22, 100000); Wire.setTimeOut(25);
    discover(); lastActivity = millis();
    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(IPAddress(192,168,4,1), IPAddress(192,168,4,1), IPAddress(255,255,255,0));
    String ssid = "LabDiag-" + WiFi.softAPmacAddress().substring(12); ssid.replace(":", "");
    const bool apReady = WiFi.softAP(ssid.c_str(), "LabDiag2026");
    web.on("/", HTTP_GET, [] { web.send_P(200, "text/html; charset=utf-8", page); });
    web.on("/status", HTTP_GET, [] { web.sendHeader("Cache-Control", "no-store"); web.send(200, "application/json", status()); });
    web.onNotFound([] { web.send(404, "text/plain", "Read-only diagnostics"); });
    web.begin();
    Serial.println("BENCH DIAGNOSTICS ONLY - ENABLE PERMANENTLY LOW - NO HEATER ATTACHED");
    Serial.printf("AP ready:%d SSID:%s password:LabDiag2026\n", apReady, ssid.c_str());
    Serial.println(WiFi.softAPIP()); Serial.println("Commands: status | scan | stop | probe <1..20>");
}
void loop() {
    inhibit();
    if (probe && bench::probeExpired(millis(), probeStarted)) stopProbe();
    userInput(); serialCommand(); sample(); web.handleClient();
    inhibit(); delay(1);
}
