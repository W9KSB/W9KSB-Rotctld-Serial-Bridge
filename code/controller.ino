/*  GS-232 <-> rotctld Bridge (ESP32-S3)
    ------------------------------------------------------------
    Targets:
      - ESP32-S3 (USB CDC serial @ 57600) receives GS-232 commands from SatPC32 / PC software
      - Forwards position set/get to a rotctld instance over TCP (Hamlib rotctld)
      - Optional EC11 encoder acts as USB keyboard (+ / -) for TX frequency adjust
      - 20x4 I2C LCD status display
      - Web UI: status + settings + 3 debug panes with per-pane Refresh and Start/Stop polling

    Notes / defaults:
      - I2C pins: SDA=17, SCL=18 (your confirmed working pins)
      - LCD I2C addr default 0x27 (change below if needed)
      - Serial baud: 57600 (per your project preference)
*/

#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <Wire.h>

// LCD
#include <LiquidCrystal_I2C.h>++++-----++++

// USB HID keyboard (ESP32-S3)
// If your Arduino-ESP32 core doesn't have this header, install/enable TinyUSB in Tools.
#include <USB.h>
#include <USBHIDKeyboard.h>

// ----------------------------
// WiFi credentials (edit here)
// ----------------------------
static const char* WIFI_SSID = "WIFI-SSID-HERE";
static const char* WIFI_PASS = "WIFI-PASS-HERE";

// Optional fallback AP if STA fails
static const bool  ENABLE_FALLBACK_AP = true;
static const char* AP_SSID = "GS232-BRIDGE";
static const char* AP_PASS = "rotator123"; // min 8 chars

// ----------------------------
// Hardware pins (edit here)
// ----------------------------
// I2C pins (confirmed)
static const int I2C_SDA_PIN = 17;
static const int I2C_SCL_PIN = 18;

// EC11 encoder pins (A/B) + pushbutton (P1->GPIO, P2->GND)
// Change these GPIOs to match your wiring.
static const int ENC_A_PIN  = 4;
static const int ENC_B_PIN  = 5;
static const int ENC_SW_PIN = 6;   // connect to encoder P1

// Encoder mode (toggles with encoder button)
// CALIBRATE: encoder sends '+' / '-'
// FREQ CTRL: encoder sends Up/Down arrows (lane select)
static volatile bool encFreqCtrlMode = false; // false=CALIBRATE, true=FREQ CTRL

// Encoder button electrical behavior:
// - If you wired P2->GND and P1->GPIO (recommended), keep ACTIVE_LOW=true (uses internal pull-up).
// - If you wired P2->3V3 and P1->GPIO, set ACTIVE_LOW=false (uses internal pull-down).
static const bool ENC_BUTTON_ACTIVE_LOW = true;

// Encoder button debounce state
// Button ISR + debounce (active-low press to GND)
static volatile bool     btn_isrPending = false;
static volatile uint32_t btn_isrAtMs    = 0;
static volatile int      btn_isrLevel   = 1;   // raw level read in ISR (HIGH=not pressed when pullup)

static bool btn_stablePressed = false;          // debounced pressed state
static uint32_t btn_lastEventMs = 0;
static const uint32_t BTN_DEBOUNCE_MS = 25;
// encoder P2 goes to GND (shared with LCD GND)

// ----------------------------
// LCD config
// ----------------------------
static const uint8_t LCD_I2C_ADDR = 0x27;
static const uint8_t LCD_COLS = 20;
static const uint8_t LCD_ROWS = 4;

// ----------------------------
// Behavior tuning
// ----------------------------

// LCD "ticker" (line 4) - short event summaries

// AP-mode line-2 horizontal scroll state
static uint32_t apScrollLastMs = 0;
static uint16_t apScrollPos = 0;
static const uint32_t AP_SCROLL_STEP_MS = 350;


static String fmt3(int v) {
  char b[5];
  snprintf(b, sizeof(b), "%03d", v);
  return String(b);
}

// WiFi mode flags (declared here so LCD helpers can reference them)
static bool wifiSTAConnected = false;
static bool wifiAPActive = false;

static String lcdLineWifiState() {
  // LCD line 1
  if (wifiAPActive) return "Wifi: AP Mode";
  if (wifiSTAConnected) return "Wifi: Connected";
  return "Wifi: Disconnected";
}

static String lcdLineIpOrApScroll() {
  // LCD line 2
  if (!wifiAPActive) {
    if (wifiSTAConnected) return String("IP: ") + WiFi.localIP().toString();
    return "IP: --";
  }

  const String full = String("IP: 192.168.4.1  User: ") + AP_SSID + " Pass: " + AP_PASS;

  const uint32_t now = millis();
  if (now - apScrollLastMs >= AP_SCROLL_STEP_MS) {
    apScrollLastMs = now;
    apScrollPos++;
    if (apScrollPos >= full.length() + 20) apScrollPos = 0; // wrap
  }

  String padded = full + String("                    "); // 20 spaces gap
  if (apScrollPos > padded.length()) apScrollPos = 0;
  return padded.substring(apScrollPos, apScrollPos + 20);
}
static const uint32_t WIFI_CONNECT_TIMEOUT_MS = 12000;

static const uint16_t HTTP_PORT = 80;

// GS-232 serial
static const uint32_t PC_BAUD = 57600;

// rotctld socket timeouts
static const uint32_t ROT_TCP_CONNECT_TIMEOUT_MS = 1200;
static const uint32_t ROT_TCP_RW_TIMEOUT_MS      = 700;

// Encoder behavior
// EC11 encoders are bouncy. We decode using a transition table and only emit one event per detent.
static const uint16_t ENC_STEP_MIN_MS    = 25;   // min time between emitted +/- events
static const uint8_t  ENC_DETENT_STEPS   = 4;    // typical EC11: 4 valid transitions per detent
static const uint16_t ENC_SW_DEBOUNCE_MS = 45;   // button debounce
static const uint16_t ENC_ISR_GUARD_US   = 250;  // ignore ISR bursts faster than this (helps with bounce)

// Debug logs
static const size_t LOG_LINES = 20;
static const size_t LOG_LINE_MAX = 120;

// ----------------------------
// Globals
// ----------------------------
WebServer server(HTTP_PORT);
Preferences prefs;
// USB HID Keyboard (ESP32-S3 TinyUSB)
USBHIDKeyboard Keyboard;
static void toggleEncoderMode();

// Encoder button HID action (configurable)
// Accepts: ENTER, SPACE, TAB, ESC, BACKSPACE, or a single character (e.g. R)
static const char* DEFAULT_BTN_KEY = "ENTER";
static String btnKey = DEFAULT_BTN_KEY;


LiquidCrystal_I2C lcd(LCD_I2C_ADDR, LCD_COLS, LCD_ROWS);

// Cache last rendered LCD lines to avoid re-printing (reduces flicker)
static String lcdLastPrinted[LCD_ROWS];

WiFiClient rotClient;

static String rotHost = "192.168.1.50";
static uint16_t rotPort = 4533;
static bool safetyEnabled = true;

// Last command received from PC software
static float lastPcAz = NAN;
static float lastPcEl = NAN;

// Last known/queried rotator position
static float lastRotAz = NAN;
static float lastRotEl = NAN;

// WiFi mode flags are declared near the LCD helpers above.

// Encoder state
static volatile int16_t encSteps = 0;     // quarter-steps from ISR (+/-1)
static volatile uint8_t encPrevAB = 0;    // last AB state (0..3)
static volatile uint32_t encLastIsrUs = 0;

static int8_t detentAcc = 0;              // accumulated quarter-steps in task context
static uint32_t lastEncEmitMs = 0;        // rate limit emitted events

// Button debounce state (pullup => idle HIGH, pressed LOW)
static bool swStable = true;
static bool swLastRead = true;
static uint32_t swLastChangeMs = 0;

// LCD timing (per-line refresh to keep UI responsive)
static uint32_t lastLcdLine0Ms = 0; // Wifi status
static uint32_t lastLcdLine1Ms = 0; // IP line (incl AP scroll)
static uint32_t lastLcdLine2Ms = 0; // Position line (real rotator AZ/EL)
static uint32_t lastLcdLine3Ms = 0; // Mode line

static const uint32_t LCD_LINE0_MS = 1000;
static const uint32_t LCD_LINE1_MS_STA = 1000;
static const uint32_t LCD_LINE1_MS_AP_SCROLL = 350;
static const uint32_t LCD_LINE2_POS_MS = 1500;
static const uint32_t LCD_LINE3_MODE_MS = 500;
// ----------------------------
// Simple ring buffer logger
// ----------------------------
struct RingLog {
  String lines[LOG_LINES];
  size_t head = 0;
  size_t count = 0;

  void add(const String& s) {
    String t = s;
    if (t.length() > LOG_LINE_MAX) t = t.substring(0, LOG_LINE_MAX);
    lines[head] = t;
    head = (head + 1) % LOG_LINES;
    if (count < LOG_LINES) count++;
  }

  // Get newest->oldest or oldest->newest; we want oldest->newest for display
  String get(size_t maxLines) const {
    size_t n = (maxLines > count) ? count : maxLines;
    String out;
    out.reserve(n * 64);
    size_t start = (head + LOG_LINES - count) % LOG_LINES; // oldest
    for (size_t i = 0; i < n; i++) {
      size_t idx = (start + i) % LOG_LINES;
      out += lines[idx];
      out += "\n";
    }
    return out;
  }
};

static RingLog logPC;
static RingLog logROT;
static RingLog logENC;
static RingLog logSYS;

// ----------------------------
// Helpers
// ----------------------------
static String ipToString(const IPAddress& ip) {
  return String(ip[0]) + "." + String(ip[1]) + "." + String(ip[2]) + "." + String(ip[3]);
}

static String fmt3(float v) {
  if (!isfinite(v)) return String("---");
  int iv = (int)lroundf(v);
  if (iv < 0) iv = 0;
  if (iv > 999) iv = 999;
  char buf[8];
  snprintf(buf, sizeof(buf), "%03d", iv);
  return String(buf);
}

static String fmt2(float v) {
  if (!isfinite(v)) return String("--");
  int iv = (int)lroundf(v);
  if (iv < 0) iv = 0;
  if (iv > 99) iv = 99;
  char buf[6];
  snprintf(buf, sizeof(buf), "%02d", iv);
  return String(buf);
}

static String htmlEscape(const String& s) {
  String out;
  out.reserve(s.length() + 8);
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    switch (c) {
      case '&': out += "&amp;"; break;
      case '<': out += "&lt;"; break;
      case '>': out += "&gt;"; break;
      case '"': out += "&quot;"; break;
      case '\'': out += "&#39;"; break;
      default: out += c; break;
    }
  }
  return out;
}

static void lcdWriteLine(uint8_t row, const String& text) {
  String t = text;
  // Normalize to exactly 20 chars (pad or trim) WITHOUT String(char, n) ctor ambiguity
  if (t.length() > LCD_COLS) t = t.substring(0, LCD_COLS);
  while (t.length() < LCD_COLS) t += ' ';

  // Avoid re-printing identical content (reduces visible LCD "wipe"/ghosting)
  if (row < LCD_ROWS && lcdLastPrinted[row] == t) return;
  if (row < LCD_ROWS) lcdLastPrinted[row] = t;

  lcd.setCursor(0, row);
  lcd.print(t);
}


static String lcdCenter20(const String &s) {
  if (s.length() >= 20) return s.substring(0, 20);
  int pad = (20 - (int)s.length()) / 2;
  String out;
  for (int i = 0; i < pad; i++) out += ' ';
  out += s;
  while (out.length() < 20) out += ' ';
  return out;
}

static String lcdLine4Mode() {
  // Matches your requested labels
  return encFreqCtrlMode ? lcdCenter20("Frequency Ctrl") : lcdCenter20("CALIBRATE");
}

// LCD line 3 (row 2): real rotator position (from rotctld feedback)
static String lcdLineRotatorPos() {
  // If we haven't received a valid position yet, show dashes
  if (!isfinite(lastRotAz) || !isfinite(lastRotEl)) {
    return "AZ:--- EL:---";
  }

  // Example: "AZ:010 EL:010" (<= 20 chars)
  int az = (int)lround(lastRotAz);
  int el = (int)lround(lastRotEl);
  if (az < 0) az = 0; if (az > 360) az = 360;
  if (el < 0) el = 0; if (el > 180) el = 180;
  return String("AZ:") + fmt3(az) + String(" EL:") + fmt3(el);
}


static void serviceLCD(uint32_t nowMs) {
  // Line 1 (row 0): Wifi state
  if (nowMs - lastLcdLine0Ms >= LCD_LINE0_MS) {
    lastLcdLine0Ms = nowMs;
    lcdWriteLine(0, lcdCenter20(lcdLineWifiState()));
  }

  // Line 2 (row 1): IP line (scrolls in AP mode)
  const uint32_t ipInterval = wifiAPActive ? LCD_LINE1_MS_AP_SCROLL : LCD_LINE1_MS_STA;
  if (nowMs - lastLcdLine1Ms >= ipInterval) {
    lastLcdLine1Ms = nowMs;
    lcdWriteLine(1, lcdLineIpOrApScroll());
  }

  // Line 3 (row 2): Rotator position (can be slower)
  if (nowMs - lastLcdLine2Ms >= LCD_LINE2_POS_MS) {
    lastLcdLine2Ms = nowMs;
    lcdWriteLine(2, lcdLineRotatorPos());
  }

  // Line 4 (row 3): Encoder mode (more responsive)
  if (nowMs - lastLcdLine3Ms >= LCD_LINE3_MODE_MS) {
    lastLcdLine3Ms = nowMs;
    lcdWriteLine(3, lcdLine4Mode());
  }
}


static void toggleEncoderMode() {
  encFreqCtrlMode = !encFreqCtrlMode;
  if (encFreqCtrlMode) {
    logENC.add("ENC Mode -> Frequency Ctrl");
  } else {
    logENC.add("ENC Mode -> CALIBRATE");
  }
}


static void setEvent(const String& s) {
  // LCD line 4 shows last event (trimmed)
  logENC.add(String("EV ") + s);
}

static bool rotEnsureConnected() {
  if (rotClient.connected()) return true;

  rotClient.stop();
  rotClient.setTimeout(ROT_TCP_RW_TIMEOUT_MS / 1000); // WiFiClient uses seconds for setTimeout
  uint32_t start = millis();
  while (millis() - start < ROT_TCP_CONNECT_TIMEOUT_MS) {
    if (rotClient.connect(rotHost.c_str(), rotPort)) {
      logROT.add(String("ROT Connected ") + rotHost + ":" + String(rotPort));
      return true;
    }
    delay(10);
  }
  logROT.add("ROT connect failed");
  return false;
}

static void rotDisconnectIfOpen() {
  if (rotClient.connected()) {
    rotClient.stop();
    logROT.add("ROT disconnected");
  }
}

static bool rotSendLine(const String& line) {
  if (!rotEnsureConnected()) return false;
  // rotctld expects LF line endings
  size_t n = rotClient.print(line);
  rotClient.print("\n");
  rotClient.flush();
  logROT.add(String(">> ") + line);
  return (n > 0);
}

static bool rotReadLine(String& out, uint32_t timeoutMs) {
  out = "";
  if (!rotEnsureConnected()) return false;

  uint32_t start = millis();
  while (millis() - start < timeoutMs) {
    while (rotClient.available()) {
      char c = (char)rotClient.read();
      if (c == '\r') continue;
      if (c == '\n') {
        if (out.length() == 0) continue; // skip blank lines
        return true;
      }
      if (out.length() < 200) out += c;
    }
    delay(1);
  }
  return false;
}

static bool rotGetPosition(float& az, float& el) {
  az = NAN; el = NAN;
  if (!rotSendLine("p")) return false;

  // rotctld "p" returns az line then el line (commonly)
  String l1, l2;
  if (!rotReadLine(l1, ROT_TCP_RW_TIMEOUT_MS)) {
    logROT.add("<< (timeout az)");
    return false;
  }
  if (!rotReadLine(l2, ROT_TCP_RW_TIMEOUT_MS)) {
    logROT.add("<< (timeout el)");
    return false;
  }
  logROT.add(String("<< ") + l1);
  logROT.add(String("<< ") + l2);

  az = l1.toFloat();
  el = l2.toFloat();
  lastRotAz = az;
  lastRotEl = el;
  return true;
}

static bool rotSetPosition(float az, float el) {
  // safety: if enabled, constrain
  if (safetyEnabled) {
    if (az < 0) az = 0;
    if (az > 360) az = 360;
    if (el < 0) el = 0;
    if (el > 90) el = 90;
  }
  String cmd = String("P ") + String(az, 1) + " " + String(el, 1);
  return rotSendLine(cmd);
}

// ----------------------------
// GS-232 parsing
// ----------------------------
static void pcSendReply(const String& s) {
  Serial.print(s);
  Serial.print("\r\n");
  logPC.add(String("<< ") + s);
}

static void handlePcCommand(const String& lineRaw) {
  String line = lineRaw;
  line.trim();
  if (line.length() == 0) return;

  logPC.add(String(">> ") + line);

  // Uppercase normalize
  String up = line;
  up.toUpperCase();

  if (up == "C2" || up == "C" || up == "W") {
    float az, el;
    if (rotGetPosition(az, el)) {
      String resp = String("AZ=") + fmt3(az) + " EL=" + fmt3(el);
      pcSendReply(resp);
    } else {
      pcSendReply("ERR");
    }
    return;
  }

  if (up.length() >= 2 && up[0] == 'W') {
    String rest = up.substring(1);
    rest.trim();

    String aStr, bStr;
    int sp = rest.indexOf(' ');
    if (sp < 0) sp = rest.indexOf('\t');

    if (sp >= 0) {
      aStr = rest.substring(0, sp);
      bStr = rest.substring(sp + 1);
      aStr.trim();
      bStr.trim();
    } else {
      if (rest.length() >= 6) {
        aStr = rest.substring(0, 3);
        bStr = rest.substring(3, 6);
      } else {
        pcSendReply("ERR");
        return;
      }
    }

    if (aStr.length() == 0 || bStr.length() == 0) {
      pcSendReply("ERR");
      return;
    }

    float az = aStr.toFloat();
    float el = bStr.toFloat();

    lastPcAz = az;
    lastPcEl = el;

    rotSetPosition(az, el);
    pcSendReply("OK");
    return;
  }

  if (up == "S" || up == "STOP") {
    pcSendReply("OK");
    return;
  }

  float az = lastPcAz;
  float el = lastPcEl;
  bool gotAz = false;
  bool gotEl = false;

  int i = 0;
  while (i < up.length()) {
    while (i < up.length() && (up[i] == ' ' || up[i] == ',' || up[i] == ';' || up[i] == '\t')) i++;
    if (i >= up.length()) break;

    int j = i;
    while (j < up.length() && up[j] != ' ' && up[j] != ',' && up[j] != ';' && up[j] != '\t') j++;
    String tok = up.substring(i, j);
    i = j;

    if (tok.startsWith("AZ")) {
      String num = tok.substring(2);
      num.trim();
      if (num.length() > 0) {
        az = num.toFloat();
        gotAz = true;
      }
      continue;
    }
    if (tok.startsWith("EL")) {
      String num = tok.substring(2);
      num.trim();
      if (num.length() > 0) {
        el = num.toFloat();
        gotEl = true;
      }
      continue;
    }
  }

  if (gotAz) lastPcAz = az;
  if (gotEl) lastPcEl = el;

  if (gotAz || gotEl) {
    float sendAz = isfinite(lastPcAz) ? lastPcAz : 0.0f;
    float sendEl = isfinite(lastPcEl) ? lastPcEl : 0.0f;
    rotSetPosition(sendAz, sendEl);
    pcSendReply("OK");
    return;
  }

  pcSendReply("ERR");
}

// ----------------------------
// Encoder ISR (quadrature, bounce-tolerant)
// ----------------------------
static const int8_t ENC_TRANSITION[16] = {
  0, -1, +1,  0,
 +1,  0,  0, -1,
 -1,  0,  0, +1,
  0, +1, -1,  0
};

static void IRAM_ATTR encISR() {
  uint32_t us = micros();
  if ((uint32_t)(us - encLastIsrUs) < ENC_ISR_GUARD_US) return;
  encLastIsrUs = us;

  uint8_t a = (uint8_t)digitalRead(ENC_A_PIN);
  uint8_t b = (uint8_t)digitalRead(ENC_B_PIN);
  uint8_t curr = (a << 1) | b;

  uint8_t idx = (encPrevAB << 2) | curr;
  int8_t step = ENC_TRANSITION[idx];
  if (step != 0) {
    int16_t v = encSteps + step;
    if (v > 32760) v = 32760;
    if (v < -32760) v = -32760;
    encSteps = v;
  }

  encPrevAB = curr;
}

static void IRAM_ATTR encBtnISR() {
  btn_isrLevel = digitalRead(ENC_SW_PIN);
  btn_isrAtMs = millis();
  btn_isrPending = true;
}

static void sendKeyPlus() {
  Keyboard.write('+');
  logENC.add("TX Freq +");
}

static void sendKeyMinus() {
  Keyboard.write('-');
  logENC.add("TX Freq -");
}

static void sendKeyLaneUp() {
  Keyboard.press(KEY_UP_ARROW);
  delay(5);
  Keyboard.releaseAll();
  logENC.add("TX Lane +");
}

static void sendKeyLaneDown() {
  Keyboard.press(KEY_DOWN_ARROW);
  delay(5);
  Keyboard.releaseAll();
  logENC.add("TX Lane -");
}

static inline bool encButtonPressedRaw() {
  const int v = digitalRead(ENC_SW_PIN);
  if (ENC_BUTTON_ACTIVE_LOW) return (v == LOW);
  return (v == HIGH);
}

static void encoderTask() {
  int16_t steps = 0;
  noInterrupts();
  if (encSteps != 0) {
    steps = encSteps;
    encSteps = 0;
  }
  interrupts();

  if (steps != 0) {
    detentAcc += steps;

    uint32_t now = millis();

    if (now - lastEncEmitMs >= ENC_STEP_MIN_MS) {
      if (detentAcc >= (int8_t)ENC_DETENT_STEPS) {
        detentAcc -= (int8_t)ENC_DETENT_STEPS;
        lastEncEmitMs = now;
        if (encFreqCtrlMode) sendKeyLaneUp(); else sendKeyPlus();
      } else if (detentAcc <= -(int8_t)ENC_DETENT_STEPS) {
        detentAcc += (int8_t)ENC_DETENT_STEPS;
        lastEncEmitMs = now;
        if (encFreqCtrlMode) sendKeyLaneDown(); else sendKeyMinus();
      }
    }

    if (detentAcc > 32) detentAcc = 32;
    if (detentAcc < -32) detentAcc = -32;
  }

  const uint32_t nowMs = millis();
  if (btn_isrPending) {
    if ((uint32_t)(nowMs - btn_isrAtMs) >= BTN_DEBOUNCE_MS) {
      btn_isrPending = false;
      const bool pressed = encButtonPressedRaw();
      if (pressed != btn_stablePressed) {
        btn_stablePressed = pressed;
        if (pressed) {
          if ((uint32_t)(nowMs - btn_lastEventMs) >= 150) {
            btn_lastEventMs = nowMs;
            logENC.add("ENC Button");
            toggleEncoderMode();
          }
        }
      }
    }
  }
}

// ----------------------------
// Web UI
// ----------------------------
static void sendHeader(const char* title) {
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/html; charset=utf-8", "");

  server.sendContent("<!doctype html><html><head><meta charset='utf-8'>");
  server.sendContent("<meta name='viewport' content='width=device-width,initial-scale=1'>");
  server.sendContent("<title>");
  server.sendContent(title);
  server.sendContent("</title>");
  server.sendContent("<style>"
    "body{font-family:system-ui,-apple-system,Segoe UI,Roboto,Arial,sans-serif;margin:0;background:#0b0d10;color:#e9eef5}"
    ".wrap{max-width:980px;margin:0 auto;padding:18px}"
    ".card{background:#121722;border:1px solid #202a3a;border-radius:14px;padding:14px;margin:12px 0}"
    ".row{display:flex;gap:12px;flex-wrap:wrap}"
    ".col{flex:1;min-width:280px}"
    "h1{font-size:20px;margin:0 0 6px 0}"
    "h2{font-size:14px;margin:0 0 10px 0;color:#b8c4d6;font-weight:600}"
    "a{color:#8cc4ff;text-decoration:none} a:hover{text-decoration:underline}"
    "input,select{width:100%;padding:10px;border-radius:10px;border:1px solid #2a3a52;background:#0f1420;color:#e9eef5}"
    "button{padding:10px 12px;border:0;border-radius:10px;background:#2b6cff;color:white;font-weight:700;cursor:pointer}"
    "button.secondary{background:#263247}"
    ".pill{display:inline-block;padding:4px 10px;border-radius:999px;background:#1a2436;border:1px solid #2a3a52;color:#cfe0ff;font-size:12px}"
    ".mono{font-family:ui-monospace,SFMono-Regular,Menlo,Consolas,monospace;white-space:pre;overflow:auto;max-height:280px;background:#0f1420;border:1px solid #2a3a52;border-radius:12px;padding:10px}"
    "</style></head><body><div class='wrap'>");
  server.sendContent("<div class='card'><h1>GS-232 &lt;-&gt; rotctld Bridge</h1>");
  server.sendContent("<div class='row'>");
  server.sendContent("<div class='col'><span class='pill'>USB Serial: 57600</span></div>");
  server.sendContent("<div class='col'><a href='/'>Status</a> &nbsp;|&nbsp; <a href='/settings'>Settings</a> &nbsp;|&nbsp; <a href='/debug'>Debug</a></div>");
  server.sendContent("</div></div>");
}

static void sendFooter() {
  server.sendContent("</div></body></html>");
  server.client().stop();
}

static String wifiStatusLine() {
  if (wifiSTAConnected) {
    return String("WiFi STA ") + ipToString(WiFi.localIP());
  }
  if (wifiAPActive) {
    return String("WiFi AP ") + ipToString(WiFi.softAPIP());
  }
  return "WiFi DOWN";
}

static String rotStatusLine() {
  if (rotClient.connected()) {
    String s = "ROT OK ";
    s += rotHost;
    s += ":";
    s += String(rotPort);
    return s;
  }
  String s = "ROT -- ";
  s += rotHost;
  s += ":";
  s += String(rotPort);
  return s;
}

static void handleRoot() {
  sendHeader("GS-232 <-> rotctld Bridge");

  server.sendContent("<div class='card'><h2>Live Status</h2>");
  server.sendContent("<div class='row'>");

  // WiFi
  server.sendContent("<div class='col'><div class='pill'>");
  server.sendContent(htmlEscape(wifiStatusLine()));
  server.sendContent("</div></div>");

  // ROT
  server.sendContent("<div class='col'><div class='pill'>");
  server.sendContent(htmlEscape(rotStatusLine()));
  server.sendContent("</div></div>");

  // Safety
  server.sendContent("<div class='col'><div class='pill'>Safety: ");
  server.sendContent(safetyEnabled ? "ON" : "OFF");
  server.sendContent("</div></div>");

  server.sendContent("</div>"); // row

  server.sendContent("<div style='margin-top:10px' class='row'>");
  server.sendContent("<div class='col'><b>Last from PC</b><div class='mono'>AZ: ");
  server.sendContent(fmt3(lastPcAz));
  server.sendContent("  EL: ");
  server.sendContent(fmt3(lastPcEl));
  server.sendContent("</div></div>");

  server.sendContent("<div class='col'><b>Last from ROT</b><div class='mono'>AZ: ");
  server.sendContent(fmt3(lastRotAz));
  server.sendContent("  EL: ");
  server.sendContent(fmt3(lastRotEl));
  server.sendContent("</div></div>");
  server.sendContent("</div>");

  server.sendContent("</div>"); // card

  // Quick controls
  server.sendContent("<div class='card'><h2>Quick Set</h2>");
  server.sendContent("<form method='POST' action='/set'>");
  server.sendContent("<div class='row'>");
  server.sendContent("<div class='col'>AZ (0-360)<br><input name='az' value='0'></div>");
  server.sendContent("<div class='col'>EL (0-90)<br><input name='el' value='0'></div>");
  server.sendContent("<div class='col' style='display:flex;align-items:flex-end;gap:10px'>"
                     "<button type='submit'>Send</button>"
                     "<a class='pill' href='/settings'>Settings</a>"
                     "</div>");
  server.sendContent("</div></form></div>");

  sendFooter();
}

static void handleSet() {
  if (!server.hasArg("az") || !server.hasArg("el")) {
    server.send(400, "text/plain", "missing args");
    return;
  }
  float az = server.arg("az").toFloat();
  float el = server.arg("el").toFloat();

  lastPcAz = az;
  lastPcEl = el;
  logROT.add(String("WEB set AZ=") + String(az, 1) + " EL=" + String(el, 1));

  rotSetPosition(az, el);

  server.sendHeader("Location", "/");
  server.send(303, "text/plain", "OK");
}

static void handleSettings() {
  sendHeader("Settings");

  server.sendContent("<div class='card'><h2>Network</h2>");
  server.sendContent("<div class='row'>");
  server.sendContent("<div class='col'><b>WiFi SSID</b><div class='mono'>");
  server.sendContent(htmlEscape(String(WIFI_SSID)));
  server.sendContent("</div></div>");
  server.sendContent("<div class='col'><b>Mode</b><div class='mono'>");
  server.sendContent(wifiSTAConnected ? "STA (client)" : (wifiAPActive ? "AP (fallback)" : "Down"));
  server.sendContent("</div></div>");
  server.sendContent("</div></div>");

  server.sendContent("<div class='card'><h2>Configuration</h2>");
  server.sendContent("<form method='POST' action='/save'>");

  server.sendContent("<div class='row'>");
  server.sendContent("<div class='col'>rotctld Host/IP<br><input name='rhost' value='");
  server.sendContent(htmlEscape(rotHost));
  server.sendContent("'></div>");
  server.sendContent("<div class='col'>rotctld Port<br><input name='rport' value='");
  server.sendContent(String(rotPort));
  server.sendContent("'></div>");
  server.sendContent("<div class='col'>Safety<br><select name='safety'>");
  server.sendContent(String("<option value='1'") + (safetyEnabled ? " selected" : "") + ">ON</option>");
  server.sendContent(String("<option value='0'") + (!safetyEnabled ? " selected" : "") + ">OFF</option>");
  server.sendContent("</select></div>");
  server.sendContent("</div>");

  server.sendContent("<div style='height:10px'></div>");

  server.sendContent("<div style='margin-top:10px'><button type='submit'>Save</button> ");
  server.sendContent("<a class='pill' href='/'>Back</a></div>");
  server.sendContent("</form></div>");

  sendFooter();
}

static void handleSave() {
  if (server.hasArg("rhost")) rotHost = server.arg("rhost");
  if (server.hasArg("rport")) rotPort = (uint16_t)server.arg("rport").toInt();
  if (server.hasArg("safety")) safetyEnabled = (server.arg("safety").toInt() != 0);
  prefs.begin("gs232bridge", false);
  prefs.putString("rhost", rotHost);
  prefs.putUShort("rport", rotPort);
  prefs.putBool("safety", safetyEnabled);
  prefs.end();

  rotDisconnectIfOpen();

  server.sendHeader("Location", "/settings");
  server.send(303, "text/plain", "Saved");
}

static void handleDebug() {
  sendHeader("Debug");

  server.sendContent("<div class='card'><h2>Debug Console</h2>");
  server.sendContent("<div class='row'>");

  auto pane = [&](const char* id, const char* title, const char* endpoint) {
    server.sendContent("<div class='col'>");
    server.sendContent("<b>");
    server.sendContent(title);
    server.sendContent("</b>");
    server.sendContent("<div style='display:flex;gap:8px;margin:8px 0'>");
    server.sendContent("<button class='secondary' type='button' onclick=\"refreshLog('");
    server.sendContent(id);
    server.sendContent("','");
    server.sendContent(endpoint);
    server.sendContent("')\">Refresh</button>");
    server.sendContent("<button type='button' onclick=\"toggleAuto('");
    server.sendContent(id);
    server.sendContent("','");
    server.sendContent(endpoint);
    server.sendContent("')\" id='btn_");
    server.sendContent(id);
    server.sendContent("'>Start</button>");
    server.sendContent("<select id='sel_");
    server.sendContent(id);
    server.sendContent("' onchange=\"if(timers['");
    server.sendContent(id);
    server.sendContent("']){toggleAuto('");
    server.sendContent(id);
    server.sendContent("','");
    server.sendContent(endpoint);
    server.sendContent("');toggleAuto('");
    server.sendContent(id);
    server.sendContent("','");
    server.sendContent(endpoint);
    server.sendContent("');}\">"
                       "<option value='250'>250ms</option>"
                       "<option value='500' selected>500ms</option>"
                       "<option value='1000'>1000ms</option>"
                       "<option value='2000'>2000ms</option>"
                       "</select>");
    server.sendContent("</div>");
    server.sendContent("<div class='mono' id='");
    server.sendContent(id);
    server.sendContent("'>click Refresh</div>");
    server.sendContent("</div>");
  };

  pane("rot", "ROT Debug", "/log/rot");
  pane("pc",  "PC Debug",  "/log/pc");
  pane("enc", "ENC Debug", "/log/enc");
  pane("sys", "SYS Debug", "/log/sys");

  server.sendContent("</div></div>");

  server.sendContent("<script>"
    "const timers={};"
    "async function refreshLog(id,ep){"
      "try{"
        "const lines=20;"
        "const r=await fetch(ep+'?lines='+lines,{cache:'no-store'});"
        "const t=await r.text();"
        "document.getElementById(id).textContent=t||'(empty)';"
      "}catch(e){document.getElementById(id).textContent='(error) '+e;}"
    "}"
    "function toggleAuto(id,ep){"
      "const btn=document.getElementById('btn_'+id);"
      "const sel=document.getElementById('sel_'+id);"
      "const ms=parseInt(sel.value||'500',10);"
      "if(timers[id]){clearInterval(timers[id]);timers[id]=null;btn.textContent='Start';btn.classList.remove('secondary');return;}"
      "refreshLog(id,ep);"
      "timers[id]=setInterval(()=>refreshLog(id,ep),ms);"
      "btn.textContent='Stop';btn.classList.add('secondary');"
    "}"
  "</script>");

  sendFooter();
}

static void handleLogPC() {
  size_t lines = server.hasArg("lines") ? (size_t)server.arg("lines").toInt() : LOG_LINES;
  server.send(200, "text/plain; charset=utf-8", logPC.get(lines));
}

static void handleLogROT() {
  size_t lines = server.hasArg("lines") ? (size_t)server.arg("lines").toInt() : LOG_LINES;
  server.send(200, "text/plain; charset=utf-8", logROT.get(lines));
}

static void handleLogENC() {
  size_t lines = server.hasArg("lines") ? (size_t)server.arg("lines").toInt() : LOG_LINES;
  server.send(200, "text/plain; charset=utf-8", logENC.get(lines));
}

static void handleLogSYS() {
  size_t lines = server.hasArg("lines") ? (size_t)server.arg("lines").toInt() : LOG_LINES;
  server.send(200, "text/plain; charset=utf-8", logSYS.get(lines));
}

// ----------------------------
// WiFi
// ----------------------------
static void wifiStart() {
  WiFi.mode(WIFI_STA);
  esp_wifi_set_ps(WIFI_PS_NONE);
  WiFi.setSleep(false);

  logSYS.add("WiFi: connecting STA...");
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  uint32_t start = millis();
  while (millis() - start < WIFI_CONNECT_TIMEOUT_MS) {
    if (WiFi.status() == WL_CONNECTED) {
      wifiSTAConnected = true;
      wifiAPActive = false;
      logSYS.add(String("WiFi STA connected: ") + ipToString(WiFi.localIP()));
      return;
    }
    delay(50);
  }

  wifiSTAConnected = false;
  logSYS.add("WiFi STA failed");

  if (ENABLE_FALLBACK_AP) {
    WiFi.mode(WIFI_AP);
    esp_wifi_set_ps(WIFI_PS_NONE);
    WiFi.softAP(AP_SSID, AP_PASS);
    wifiAPActive = true;
    logSYS.add(String("WiFi AP active: ") + ipToString(WiFi.softAPIP()));
  }
}

// ----------------------------
// Setup / loop
// ----------------------------
static String pcLineBuf;

static void sendButtonAction() {
  String k = btnKey;
  k.trim();
  k.toUpperCase();

  if (k.length() == 1) {
    Keyboard.write((uint8_t)k[0]);
    logENC.add(String("ENC Button (") + k + ")");
    return;
  }

  if (k == "ENTER" || k == "RETURN") {
    Keyboard.write((uint8_t)'\n');
    logENC.add("ENC Button (Enter)");
  } else if (k == "SPACE") {
    Keyboard.write(' ');
    logENC.add("ENC Button (Space)");
  } else if (k == "TAB") {
    Keyboard.write((uint8_t)'\t');
    logENC.add("ENC Button (Tab)");
  } else if (k == "BACKSPACE" || k == "BKSP") {
    Keyboard.write((uint8_t)'\b');
    logENC.add("ENC Button (Backspace)");
  } else if (k == "ESC" || k == "ESCAPE") {
    Keyboard.press(KEY_ESC);
    delay(5);
    Keyboard.releaseAll();
    logENC.add("ENC Button (Esc)");
  } else {
    logENC.add(String("ENC Button (unknown: ") + k + ")");
  }
}

void setup() {
  USB.begin();
  Keyboard.begin();

  Serial.begin(PC_BAUD);
  delay(200);

  prefs.begin("gs232bridge", true);
  rotHost = prefs.getString("rhost", rotHost);
  rotPort = prefs.getUShort("rport", rotPort);
  safetyEnabled = prefs.getBool("safety", safetyEnabled);
  btnKey = prefs.getString("bkey", btnKey);
  prefs.end();

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000); // 400 kHz I2C for snappier LCD updates
  lcd.init();
  lcd.backlight();
  for (uint8_t i = 0; i < LCD_ROWS; i++) lcdLastPrinted[i] = String();
  lcdWriteLine(0, "Booting...");
  lcdWriteLine(1, "WiFi...");
  lcdWriteLine(2, "ROT...");
  lcdWriteLine(3, "Ready");

  pinMode(ENC_A_PIN, INPUT_PULLUP);
  pinMode(ENC_B_PIN, INPUT_PULLUP);
  pinMode(ENC_SW_PIN, ENC_BUTTON_ACTIVE_LOW ? INPUT_PULLUP : INPUT_PULLDOWN);

  btn_stablePressed = encButtonPressedRaw();
  btn_lastEventMs = millis();
  btn_isrPending = false;
  btn_isrLevel = digitalRead(ENC_SW_PIN);
  attachInterrupt(digitalPinToInterrupt(ENC_SW_PIN), encBtnISR, CHANGE);

  encPrevAB = ((uint8_t)digitalRead(ENC_A_PIN) << 1) | (uint8_t)digitalRead(ENC_B_PIN);
  encSteps = 0;
  detentAcc = 0;
  attachInterrupt(digitalPinToInterrupt(ENC_A_PIN), encISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B_PIN), encISR, CHANGE);

  wifiStart();

  server.on("/", HTTP_GET, handleRoot);
  server.on("/set", HTTP_POST, handleSet);
  server.on("/settings", HTTP_GET, handleSettings);
  server.on("/save", HTTP_POST, handleSave);
  server.on("/debug", HTTP_GET, handleDebug);

  server.on("/log/pc", HTTP_GET, handleLogPC);
  server.on("/log/rot", HTTP_GET, handleLogROT);
  server.on("/log/enc", HTTP_GET, handleLogENC);
  server.on("/log/sys", HTTP_GET, handleLogSYS);

  server.onNotFound([]() {
    server.send(404, "text/plain", "Not found");
  });

  server.begin();
  logSYS.add(String("HTTP server on :") + String(HTTP_PORT));
  lastEncEmitMs = 0;
  swStable = digitalRead(ENC_SW_PIN);
  swLastRead = swStable;
  swLastChangeMs = millis();

  lcdWriteLine(0, lcdLineWifiState());
  lcdWriteLine(1, lcdLineIpOrApScroll());
  lcdWriteLine(2, lcdLineRotatorPos());
  lcdWriteLine(3, lcdLine4Mode());
}

void loop() {
  server.handleClient();

  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r' || c == '\n') {
      if (pcLineBuf.length() > 0) {
        handlePcCommand(pcLineBuf);
        pcLineBuf = "";
      }
    } else {
      if (pcLineBuf.length() < 180) pcLineBuf += c;
    }
  }

  encoderTask();

  uint32_t now = millis();
  serviceLCD(now);

  if (!wifiAPActive) {
    wifiSTAConnected = (WiFi.status() == WL_CONNECTED);
  }

  delay(0);
}
