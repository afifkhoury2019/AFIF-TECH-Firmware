#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WiFiClientSecure.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266HTTPUpdateServer.h>
#include <ESP8266mDNS.h>
#include <time.h>
#include <sys/time.h>
#include <DNSServer.h>
#include <EEPROM.h>
#include <PubSubClient.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <bearssl/bearssl_hash.h>

// Forward declaration. The Arduino builder inserts auto-generated function
// prototypes near the top of the sketch; two of them take a PushMsg, which is
// only defined further down (push-notification section). Declaring the type
// here lets those prototypes compile.
struct PushMsg;

// ── Pin definitions ───────────────────────────────────────────
#define TRIG1          D6
#define ECHO1          D7
#define RELAY1         D5
#define LED_INTERNAL   LED_BUILTIN
#define PAIR_BUTTON    D3
#define BUTTON_PRESSED LOW

// Buzzer on D0 (GPIO16). NOTE: GPIO16 is a special pin on the ESP8266 —
// it has no hardware PWM and tone() does not work reliably on it. This
// code therefore drives the buzzer with plain digitalWrite() on/off
// pulses, which is correct for an ACTIVE buzzer (one with its own
// built-in oscillator — it sounds whenever it gets voltage). A PASSIVE
// buzzer needs a driven frequency and will only click on this pin; if
// yours is passive, move it to D8 (GPIO15) and use tone() instead.
#define BUZZER         D0
#define BUZZER_ON      HIGH
#define BUZZER_OFF     LOW

// ── Voltage sensing (A0) ──────────────────────────────────────
// A0 is the ESP8266's only analog input. On the D1 Mini an on-board
// divider gives it a 0-3.2 V range over 1024 ADC steps.
//
// You must wire a sensor to A0 for the voltage rule to do anything.
// Two sensor styles are supported, selected by voltMode from the app:
//   VOLT_MODE_DC  - a plain resistor divider reading a DC supply.
//                   One ADC sample, scaled.
//   VOLT_MODE_AC  - an AC sensor module (e.g. ZMPT101B) whose output
//                   swings around a DC mid-point. Sampled over a few
//                   mains cycles and converted to true RMS.
// voltScale converts ADC counts to volts and MUST be calibrated against
// a known reference with a multimeter - the default is only a placeholder.
#define VOLT_PIN       A0
#define VOLT_MODE_OFF  0
#define VOLT_MODE_DC   1
#define VOLT_MODE_AC   2
#define VOLT_AC_SAMPLES 400       // ~4 mains cycles at 50 Hz
#define VOLT_ADC_MAX   1023.0

// ── OLED ──────────────────────────────────────────────────────
U8G2_SSD1306_128X64_NONAME_F_HW_I2C display(U8G2_R0, U8X8_PIN_NONE);

// ── MQTT defaults — used only until the user saves their own values
//    in pairing mode; after that the EEPROM copy always wins ──────
#define MQTT_BROKER_DEFAULT   "broker.emqx.io"
#define MQTT_PORT_DEFAULT     1883
#define MQTT_TOPIC_DEFAULT    "SensorData"
// Client-ID prefix. The full ID gets this chip's unique ID appended (see
// connectMQTT). A FIXED ID was a real bug: on a public broker every device
// (and anyone else) using "D1Mini_WaterTank" kicks the previous one off,
// which makes the tank flap between online and offline.
#define MQTT_CLIENT           "AFIFTECH-"

// ── Tank / sensor calibration ─────────────────────────────────
// Sensor in use: JSN-SR04T waterproof ultrasonic module.
// Datasheet range: 20 cm – 600 cm (NOT the same as a bare HC-SR04,
// whose datasheet minimum is ~2 cm — the JSN-SR04T's waterproof
// transducer has a much larger physical dead zone).
//
// DEFAULT_TANK_DEPTH_CM = distance (cm) the sensor reads when the
// tank is EMPTY (sensor face → tank floor). Configurable in pairing.
//
// SENSOR_FULL_DISTANCE_CM = distance (cm) the sensor reads when the
// tank is 100% FULL — set to the sensor's real datasheet minimum, so
// anything reported closer than this is physically impossible for
// this module and is treated as a sensor error, not "extra full".
#define DEFAULT_TANK_DEPTH_CM   100
#define SENSOR_FULL_DISTANCE_CM 20

// MIN_TANK_DEPTH_CM guards against configuring a tank depth so
// shallow it leaves no usable range above the sensor's dead zone —
// enforced both in the web form and in the saved-settings validation.
#define MIN_TANK_DEPTH_CM       30

// pulseIn() timeout for the echo pulse. Must cover the full round
// trip for the sensor's 600 cm max range (600*2/0.034 ≈ 35.3 ms) with
// some margin, or a genuine long-range reading would incorrectly time
// out and be reported as a sensor error.
#define ECHO_TIMEOUT_US         40000UL

// ── Auto-fill defaults — configured in pairing mode (min/max %) ──
// Enable/disable happens LIVE from the phone app over MQTT, not here.
#define DEFAULT_AUTOFILL_MIN   20   // % — pump turns ON at or below this
#define DEFAULT_AUTOFILL_MAX   90   // % — pump turns OFF at or above this

// ── Alert thresholds ──────────────────────────────────────────
// The device raises its own notifications (see the Telegram alert engine
// further down) — it does not need the phone app to be running.
#define CRITICAL_LOW_PCT    15   // % — at or below this, raise the low alert

// ── EEPROM layout ─────────────────────────────────────────────
// 0   – 63  : SSID          (64 bytes)
// 64  – 127 : Password      (64 bytes)
// 128 – 129 : Tank depth in cm, little-endian uint16 (2 bytes)
// 130 – 193 : MQTT broker   (64 bytes)
// 194 – 195 : MQTT port, little-endian uint16 (2 bytes)
// 196 – 227 : MQTT topic    (32 bytes)
// 228        : Auto-fill min % (1 byte)
// 229        : Auto-fill max % (1 byte)
// 230        : Auto-fill enabled flag (1 byte, 0/1) — updated live via MQTT
// 231        : Validity flag
// EEPROM_SIZE raised from 320 to 512 to make room for the extra rule
// slots added below. This is SAFE for already-paired devices: the ESP8266
// EEPROM library maps one 4096-byte flash sector, so bytes 0..319 keep
// their existing contents and only the newly addressed bytes are touched.
#define EEPROM_SIZE               512
#define EEPROM_SSID_ADDR          0
#define EEPROM_PASS_ADDR          64
#define EEPROM_DEPTH_ADDR         128   // 2 bytes: low byte, high byte
#define EEPROM_MQTT_BROKER_ADDR   130   // 64 bytes
#define EEPROM_MQTT_PORT_ADDR     194   // 2 bytes
#define EEPROM_MQTT_TOPIC_ADDR    196   // 32 bytes
#define EEPROM_AUTOFILL_MIN_ADDR  228
#define EEPROM_AUTOFILL_MAX_ADDR  229
#define EEPROM_AUTOFILL_EN_ADDR   230
#define EEPROM_FLAG_ADDR          231
#define EEPROM_VALID_FLAG         0xAB

// ── Rules block (added later, kept AFTER the original layout) ──
// Deliberately placed past byte 231 with its OWN validity flag, so an
// OTA update onto an already-paired device keeps its WiFi/MQTT/tank
// settings intact - only the new rules start out empty.
#define EEPROM_RULES_FLAG_ADDR    232
#define EEPROM_RULES_VALID_FLAG   0xCD
#define EEPROM_RULES_ADDR         233   // 4 rules x 10 bytes = 40 (233..272)
#define EEPROM_VOLT_EN_ADDR       273   // voltage guard enabled (1)
#define EEPROM_VOLT_MODE_ADDR     274   // 0=off 1=DC 2=AC RMS (1)
#define EEPROM_VOLT_SCALE_ADDR    275   // scale x100, uint16 LE (2)
#define EEPROM_VOLT_MIN_ADDR      277   // min volts x10, uint16 LE (2)
#define EEPROM_VOLT_MAX_ADDR      279   // max volts x10, uint16 LE (2)

// ── Extended rule slots (5..8) ────────────────────────────────
// The original 4 slots live at 233..272 and MUST stay there, sandwiched
// against the voltage-guard block at 273..280, or an OTA update onto an
// already-configured device would read its old rules from the wrong
// addresses. The extra slots are therefore appended AFTER the voltage
// block, with their OWN validity flag: a device that has never stored
// them simply starts those slots empty while keeping slots 1..4 intact.
#define EEPROM_RULES_EXT_FLAG_ADDR  281
#define EEPROM_RULES_EXT_VALID_FLAG 0xCE
#define EEPROM_RULES_EXT_ADDR       282   // 4 rules x 10 bytes = 40 (282..321)

// MAX_RULES raised 4 -> 8. RULES_BLOCK1_COUNT is how many of them live at
// the original address; everything past that comes from the extended
// block. Raising MAX_RULES further means growing the extended block and
// EEPROM_SIZE to match - see ruleEepromBase() below.
// ── Telegram alert settings (own flag, so flashing this onto a configured
//    device keeps WiFi/MQTT/tank/rules and only starts alerts off).
//    These bytes held the old Pushy settings (flag 0xDE); the NEW flag value
//    makes the device ignore them instead of misreading a Pushy key as names.
#define EEPROM_TG_FLAG_ADDR       322
#define EEPROM_TG_VALID_FLAG      0xDF
#define EEPROM_TG_EN_ADDR         323   // enabled 0/1
#define EEPROM_TG_EV_ADDR         324   // which events may notify (bitmask)
#define EEPROM_TG_USERS_ADDR      325   // "@name1|@name2|@name3" (96) -> ends at 420
#define TG_USERS_LEN              96

#define MAX_RULES                 8
#define RULES_BLOCK1_COUNT        4
#define RULE_BYTES                10

// ── AP (pairing mode) ─────────────────────────────────────────
#define AP_SSID   "AFIF-TECH-Setup"
#define AP_PASS   ""

// ── Button hold time ──────────────────────────────────────────
#define PAIR_HOLD_MS   1000

// A press released before PAIR_HOLD_MS is a short click. Presses shorter
// than SHORT_CLICK_MIN_MS are ignored as switch bounce/noise.
//   single click → toggle relay / force-start a fill cycle
//   double click → mute the low-level alarm (buzzer + app)
//   long  press  → pairing mode (unchanged)
#define SHORT_CLICK_MIN_MS 50

// After a short click is released, wait this long for a second click
// before committing to "single click". Longer = easier to double-click
// but adds this much delay before a single click acts.
#define DOUBLE_CLICK_MS    400

// ── Low-level alarm (buzzer) ──────────────────────────────────
// Matches the app's critical threshold so both alarm together.
#define ALARM_LEVEL_PCT    15
#define BEEP_ON_MS         250    // buzzer on  time within one beep cycle
#define BEEP_OFF_MS        250    // buzzer off time within one beep cycle

// ── OTA (Over-The-Air) firmware update ─────────────────────────
// Reachable at http://watertank.local/update while the device is on
// your home WiFi (normal operation, not pairing mode). Compile the
// sketch as usual in Arduino IDE (Sketch > Export Compiled Binary),
// then upload that .bin file through this page instead of USB.
// CHANGE THIS PASSWORD before deploying to more than one device —
// anyone on the same WiFi network can reflash the device otherwise.
#define OTA_HOSTNAME   "watertank"
#define OTA_USER       "admin"
#define OTA_PASSWORD   "afiftech-ota-2026"

// Firmware version, reported on <topic>/version so the app can show what is
// installed and confirm that an update worked. Raise it for every release.
#define FW_VERSION     "2.2.10"

// ─────────────────────────────────────────────────────────────
WiFiClient              espClient;
PubSubClient            mqtt(espClient);
ESP8266WebServer        server(80);
ESP8266HTTPUpdateServer httpUpdater;
DNSServer               dnsServer;                // catches all DNS → 192.168.4.1

bool          pairingMode    = false;
bool          ledState       = false;
unsigned long lastLedToggle  = 0;
unsigned long lastSensorRead = 0;
int           tankDepthCm    = DEFAULT_TANK_DEPTH_CM;

// Runtime MQTT settings — loaded from EEPROM at boot, or defaulted
// to MQTT_BROKER_DEFAULT / MQTT_PORT_DEFAULT / MQTT_TOPIC_DEFAULT
// if nothing has ever been saved.
String        mqttBroker     = MQTT_BROKER_DEFAULT;
int           mqttPort       = MQTT_PORT_DEFAULT;
String        mqttTopic      = MQTT_TOPIC_DEFAULT;

// Auto-fill runtime state. Min/max are configured in pairing mode;
// "enabled" is toggled live from the phone app over MQTT and is
// persisted immediately so it survives a restart.
int   autoFillMin     = DEFAULT_AUTOFILL_MIN;
int   autoFillMax     = DEFAULT_AUTOFILL_MAX;
bool  autoFillEnabled = true;
bool  fillActive      = false;   // hysteresis latch while auto-fill is on
bool  relayOn         = false;   // current physical relay state (true = energised)
bool  lowNotifySent   = false;   // true once the critical-low push has fired for this episode

// ── Schedule rules ────────────────────────────────────────────
// One rule = "while the clock is inside this window, force the relay to
// <action>". Window-based rather than edge-triggered, so the rule keeps
// asserting its state for the whole window - a manual toggle inside an
// active window is overridden again on the next loop, which is the
// predictable behaviour for a timer.
//
// mode 0 = weekly   : fires on the days set in daysMask
// mode 1 = on-date  : fires only on the given day/month, any year
//
// daysMask bit0=Sunday .. bit6=Saturday. 0x7F = every day.
struct Rule {
  uint8_t enabled;
  uint8_t mode;
  uint8_t daysMask;
  uint8_t startHour, startMin;
  uint8_t endHour,   endMin;
  uint8_t action;      // 1 = relay ON while rule is active, 0 = relay OFF
  uint8_t day;         // 1..31, used when mode == 1
  uint8_t month;       // 1..12, used when mode == 1
  // NOTE on mode 2 (water level): the struct is exactly RULE_BYTES (10) and
  // full, so rather than grow it (which would collide with the voltage-guard
  // EEPROM addresses at 273) mode 2 REUSES two fields that it otherwise
  // ignores:
  //     daysMask  -> the level threshold, 0..100 %
  //     startHour -> the comparison direction
  //                    1 = fires while level is BELOW the threshold
  //                    2 = fires while level is ABOVE the threshold
  //                    0 = legacy rule saved before the direction existed;
  //                        fall back to the old action-implied meaning
  //                        (action 1 = below, action 0 = above)
  // Direction and action are now independent, so "pump OFF while level is
  // BELOW 20%" (dry-run protection) is expressible, which it was not when
  // the action alone decided the direction. All other time fields are
  // ignored for mode 2.
};
Rule rules[MAX_RULES];

// Latest fill % from the sensor, or -1 if unknown/sensor error. Needed by
// activeRuleIndex() so level rules (mode 2) can test the current level.
float lastPct = -1;

// ── Voltage guard ─────────────────────────────────────────────
bool  voltGuardEnabled = false;
int   voltMode         = VOLT_MODE_OFF;
float voltScale        = 1.0;    // ADC counts -> volts (CALIBRATE THIS)
float voltMin          = 200.0;
float voltMax          = 250.0;
float lastVoltage      = 0.0;
bool  voltFault        = false;  // true while the reading is out of range

bool  timeReady        = false;  // we have a real wall-clock time
int   lastRuleIdx      = -1;     // which rule owned the relay last cycle
bool  onlineSetupDone  = false;  // post-WiFi init has completed

// ── Alarm state ───────────────────────────────────────────────
// alarmActive: the tank is at/below ALARM_LEVEL_PCT right now.
// alarmMuted : the user silenced it (double-click on the device, or Mute
//              in the app). Cleared automatically once the level recovers,
//              so the next low episode alarms again from scratch.
bool  alarmActive     = false;
bool  alarmMuted      = false;
bool  buzzerState     = false;   // current physical buzzer output
unsigned long lastBeepToggle = 0;

// Sub-topics derived from mqttTopic — recomputed whenever mqttTopic
// changes (i.e. right after loading settings, or after a fresh save).
String relayStatusTopic;
String relayCmdTopic;
String autofillStatusTopic;
String autofillCmdTopic;
String alarmStatusTopic;   // device → app: "1" alarming, "0" clear/muted
String alarmCmdTopic;      // app → device: "1" = mute now
String voltageTopic;       // device → app: last measured voltage
String statusTopic;        // device → app: "online"/"offline" (retained, last will)
String rulesStatusTopic;   // device → app: full rules JSON (retained)
String rulesCmdTopic;      // app → device: one rules command (JSON)
String otaCmdTopic;        // app → device: install firmware from a link (JSON, signed)
String otaStatusTopic;     // device → app: progress of that update (JSON)
String versionTopic;       // device → app: FW_VERSION (retained)

#define SENSOR_INTERVAL 2000
#define DNS_PORT        53

// Forward declarations
bool pollPairingButton();
void toggleRelayButton();
void handleRulesGet();
void handleRulesPost();
void handleRuleSetGet();
void handleRuleDel();
void handleRulesClearAll();
void handleRulesOptions();
void registerRuleRoutes();
void setBuzzer(bool on);
void sendCORS();
void queuePush(const char* event, uint8_t bit, const char* title, const char* body);
void servicePush();
void loadPushCfg();
void savePushCfg();
void handleTgGet();
void handleLevelTxt();
void handleTgSet();
void handleTgTest();
void queueCall(const char* text);
void handleOtaCommand(const String& msg);
int compareVersions(const String& a, const String& b);
void serviceRemoteOta();
void saveOtaToRtc();
void loadOtaFromRtc();
String heapInfo();
void raiseLevelEvents(float pct);
void raiseVoltageEvent();
void raiseSensorEvent(bool badReading);
void applyRelay(bool state, const char* reason);
String rulesJson();
void publishRules();
void setClockFromEpoch(long epoch);
void handleRulesMqtt(const String& msg);
void registerSettingsRoute();
void handleSettings();
void handleSetTime();
void finishOnlineSetup();
void defaultRule(Rule& r);
int  ruleEepromBase(int i);
void muteAlarm(const char* source);
void mqttCallback(char* topic, byte* payload, unsigned int length);

void updateDerivedTopics() {
  relayStatusTopic    = mqttTopic + "/relay";
  relayCmdTopic       = mqttTopic + "/relay/set";
  autofillStatusTopic = mqttTopic + "/autofill";
  autofillCmdTopic    = mqttTopic + "/autofill/set";
  alarmStatusTopic    = mqttTopic + "/alarm";
  alarmCmdTopic       = mqttTopic + "/alarm/set";
  voltageTopic        = mqttTopic + "/voltage";
  statusTopic         = mqttTopic + "/status";
  rulesStatusTopic    = mqttTopic + "/rules";
  rulesCmdTopic       = mqttTopic + "/rules/set";
  otaCmdTopic         = mqttTopic + "/ota/set";
  otaStatusTopic      = mqttTopic + "/ota";
  versionTopic        = mqttTopic + "/version";
}

// ── EEPROM helpers ────────────────────────────────────────────
void saveAll(const String& ssid, const String& pass, int depthCm,
             const String& mBroker, int mPort, const String& mTopic,
             int afMin, int afMax) {
  EEPROM.begin(EEPROM_SIZE);
  // Clear SSID + pass fields
  for (int i = 0; i < 64; i++) EEPROM.write(EEPROM_SSID_ADDR + i, 0);
  for (int i = 0; i < 64; i++) EEPROM.write(EEPROM_PASS_ADDR + i, 0);
  // Write SSID
  for (unsigned int i = 0; i < ssid.length() && i < 63; i++)
    EEPROM.write(EEPROM_SSID_ADDR + i, ssid[i]);
  // Write password
  for (unsigned int i = 0; i < pass.length() && i < 63; i++)
    EEPROM.write(EEPROM_PASS_ADDR + i, pass[i]);
  // Write tank depth as uint16 little-endian
  uint16_t d = (uint16_t)constrain(depthCm, MIN_TANK_DEPTH_CM, 1000);
  EEPROM.write(EEPROM_DEPTH_ADDR,     d & 0xFF);
  EEPROM.write(EEPROM_DEPTH_ADDR + 1, (d >> 8) & 0xFF);

  // Clear + write MQTT broker
  for (int i = 0; i < 64; i++) EEPROM.write(EEPROM_MQTT_BROKER_ADDR + i, 0);
  for (unsigned int i = 0; i < mBroker.length() && i < 63; i++)
    EEPROM.write(EEPROM_MQTT_BROKER_ADDR + i, mBroker[i]);

  // Write MQTT port as uint16 little-endian
  uint16_t p = (uint16_t)constrain(mPort, 1, 65535);
  EEPROM.write(EEPROM_MQTT_PORT_ADDR,     p & 0xFF);
  EEPROM.write(EEPROM_MQTT_PORT_ADDR + 1, (p >> 8) & 0xFF);

  // Clear + write MQTT topic
  for (int i = 0; i < 32; i++) EEPROM.write(EEPROM_MQTT_TOPIC_ADDR + i, 0);
  for (unsigned int i = 0; i < mTopic.length() && i < 31; i++)
    EEPROM.write(EEPROM_MQTT_TOPIC_ADDR + i, mTopic[i]);

  // Auto-fill min/max
  EEPROM.write(EEPROM_AUTOFILL_MIN_ADDR, (uint8_t)constrain(afMin, 0, 100));
  EEPROM.write(EEPROM_AUTOFILL_MAX_ADDR, (uint8_t)constrain(afMax, 0, 100));

  // Validity flag
  EEPROM.write(EEPROM_FLAG_ADDR, EEPROM_VALID_FLAG);
  EEPROM.commit();
  EEPROM.end();
}

// Save only the auto-fill enabled flag — called live from the MQTT
// command handler, independent of the WiFi/tank/MQTT settings above.
void saveAutoFillEnabled(bool enabled) {
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.write(EEPROM_AUTOFILL_EN_ADDR, enabled ? 1 : 0);
  EEPROM.commit();
  EEPROM.end();
}

bool loadAll(String& ssid, String& pass, int& depthCm,
             String& mBroker, int& mPort, String& mTopic,
             int& afMin, int& afMax, bool& afEnabled) {
  EEPROM.begin(EEPROM_SIZE);

  // Check validity flag FIRST — if not set, return clean defaults for
  // everything so a fresh/cleared device never loads garbage EEPROM data.
  if (EEPROM.read(EEPROM_FLAG_ADDR) != EEPROM_VALID_FLAG) {
    EEPROM.end();
    depthCm   = DEFAULT_TANK_DEPTH_CM;
    mBroker   = MQTT_BROKER_DEFAULT;
    mPort     = MQTT_PORT_DEFAULT;
    mTopic    = MQTT_TOPIC_DEFAULT;
    afMin     = DEFAULT_AUTOFILL_MIN;
    afMax     = DEFAULT_AUTOFILL_MAX;
    afEnabled = true;
    return false;
  }

  // SSID + password
  ssid = ""; pass = "";
  for (int i = 0; i < 63; i++) {
    char c = EEPROM.read(EEPROM_SSID_ADDR + i);
    if (c == 0) break; ssid += c;
  }
  for (int i = 0; i < 63; i++) {
    char c = EEPROM.read(EEPROM_PASS_ADDR + i);
    if (c == 0) break; pass += c;
  }

  // Tank depth
  uint16_t d = EEPROM.read(EEPROM_DEPTH_ADDR)
             | ((uint16_t)EEPROM.read(EEPROM_DEPTH_ADDR + 1) << 8);
  depthCm = (d >= MIN_TANK_DEPTH_CM && d <= 1000) ? (int)d : DEFAULT_TANK_DEPTH_CM;

  // MQTT broker
  mBroker = "";
  for (int i = 0; i < 63; i++) {
    char c = EEPROM.read(EEPROM_MQTT_BROKER_ADDR + i);
    if (c == 0) break; mBroker += c;
  }
  if (!mBroker.length()) mBroker = MQTT_BROKER_DEFAULT;

  // MQTT port — valid saved range is 1–65534; 0 and 65535 (0xFFFF) both
  // indicate an unwritten or corrupted slot and fall back to the default.
  uint16_t p = EEPROM.read(EEPROM_MQTT_PORT_ADDR)
             | ((uint16_t)EEPROM.read(EEPROM_MQTT_PORT_ADDR + 1) << 8);
  mPort = (p >= 1 && p <= 65534) ? (int)p : MQTT_PORT_DEFAULT;

  // MQTT topic
  mTopic = "";
  for (int i = 0; i < 31; i++) {
    char c = EEPROM.read(EEPROM_MQTT_TOPIC_ADDR + i);
    if (c == 0) break; mTopic += c;
  }
  if (!mTopic.length()) mTopic = MQTT_TOPIC_DEFAULT;

  // Auto-fill thresholds
  uint8_t rawMin = EEPROM.read(EEPROM_AUTOFILL_MIN_ADDR);
  uint8_t rawMax = EEPROM.read(EEPROM_AUTOFILL_MAX_ADDR);
  afMin = (rawMin <= 100) ? (int)rawMin : DEFAULT_AUTOFILL_MIN;
  afMax = (rawMax <= 100) ? (int)rawMax : DEFAULT_AUTOFILL_MAX;
  if (afMin >= afMax) { afMin = DEFAULT_AUTOFILL_MIN; afMax = DEFAULT_AUTOFILL_MAX; }

  // Auto-fill enabled flag (0xFF = unwritten flash → default to enabled)
  uint8_t rawEn = EEPROM.read(EEPROM_AUTOFILL_EN_ADDR);
  afEnabled = (rawEn == 0) ? false : true;

  EEPROM.end();
  return ssid.length() > 0;
}

void clearAll() {
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.write(EEPROM_FLAG_ADDR, 0x00);
  EEPROM.commit();
  EEPROM.end();
}

// ── Rules + voltage guard persistence ─────────────────────────
// Separate from saveAll()/loadAll() and guarded by its own flag, so
// flashing this firmware onto an already-configured device does not
// disturb its WiFi, MQTT or tank settings.
// Resets one slot to the canonical EMPTY state. Every "delete" path goes
// through here so a cleared slot is always byte-identical, which is what
// lets the app reliably tell an unused slot from a configured one.
// NOTE: action defaults to 0 (not 1 as in earlier firmware) purely so the
// empty pattern is unambiguous - with enabled = 0 it changes no behaviour.
void defaultRule(Rule& r) {
  r.enabled   = 0;
  r.mode      = 0;
  r.daysMask  = 0x7F;
  r.startHour = 0; r.startMin = 0;
  r.endHour   = 0; r.endMin   = 0;
  r.action    = 0;
  r.day       = 1; r.month    = 1;
}

// Maps a rule index to its EEPROM address. Slots 0..3 sit at the original
// location so existing devices keep their rules across this update; slots
// 4+ live in the extended block past the voltage-guard bytes.
int ruleEepromBase(int i) {
  if (i < RULES_BLOCK1_COUNT) return EEPROM_RULES_ADDR + (i * RULE_BYTES);
  return EEPROM_RULES_EXT_ADDR + ((i - RULES_BLOCK1_COUNT) * RULE_BYTES);
}

void saveRules() {
  EEPROM.begin(EEPROM_SIZE);
  for (int i = 0; i < MAX_RULES; i++) {
    int base = ruleEepromBase(i);
    EEPROM.write(base + 0, rules[i].enabled   ? 1 : 0);
    EEPROM.write(base + 1, rules[i].mode);
    EEPROM.write(base + 2, rules[i].daysMask);
    EEPROM.write(base + 3, rules[i].startHour);
    EEPROM.write(base + 4, rules[i].startMin);
    EEPROM.write(base + 5, rules[i].endHour);
    EEPROM.write(base + 6, rules[i].endMin);
    EEPROM.write(base + 7, rules[i].action ? 1 : 0);
    EEPROM.write(base + 8, rules[i].day);
    EEPROM.write(base + 9, rules[i].month);
  }
  EEPROM.write(EEPROM_VOLT_EN_ADDR,   voltGuardEnabled ? 1 : 0);
  EEPROM.write(EEPROM_VOLT_MODE_ADDR, (uint8_t)voltMode);

  uint16_t sc = (uint16_t)constrain((long)(voltScale * 100.0), 1L, 65535L);
  EEPROM.write(EEPROM_VOLT_SCALE_ADDR,     sc & 0xFF);
  EEPROM.write(EEPROM_VOLT_SCALE_ADDR + 1, (sc >> 8) & 0xFF);

  uint16_t vmin = (uint16_t)constrain((long)(voltMin * 10.0), 0L, 65535L);
  EEPROM.write(EEPROM_VOLT_MIN_ADDR,     vmin & 0xFF);
  EEPROM.write(EEPROM_VOLT_MIN_ADDR + 1, (vmin >> 8) & 0xFF);

  uint16_t vmax = (uint16_t)constrain((long)(voltMax * 10.0), 0L, 65535L);
  EEPROM.write(EEPROM_VOLT_MAX_ADDR,     vmax & 0xFF);
  EEPROM.write(EEPROM_VOLT_MAX_ADDR + 1, (vmax >> 8) & 0xFF);

  EEPROM.write(EEPROM_RULES_FLAG_ADDR,     EEPROM_RULES_VALID_FLAG);
  EEPROM.write(EEPROM_RULES_EXT_FLAG_ADDR, EEPROM_RULES_EXT_VALID_FLAG);
  EEPROM.commit();
  EEPROM.end();

  // Every path that changes rules ends here — HTTP from the Android app,
  // MQTT from the web app — so publishing here keeps every app in sync no
  // matter which one made the change. No-op while MQTT is down.
  publishRules();
}

void loadRules() {
  EEPROM.begin(EEPROM_SIZE);

  // Two independent validity flags. A device updated from the 4-slot
  // firmware has the first one set and the second one not - it keeps its
  // four saved rules and gets four empty new slots, rather than losing
  // everything or reading garbage out of never-written flash.
  bool haveBlock1 = (EEPROM.read(EEPROM_RULES_FLAG_ADDR)     == EEPROM_RULES_VALID_FLAG);
  bool haveExt    = (EEPROM.read(EEPROM_RULES_EXT_FLAG_ADDR) == EEPROM_RULES_EXT_VALID_FLAG);

  // Start from a clean empty set, then overlay whatever is actually stored.
  for (int i = 0; i < MAX_RULES; i++) defaultRule(rules[i]);

  if (!haveBlock1) {
    // Never saved on this device - leave everything disabled so a fresh
    // unit behaves exactly as it did before rules existed.
    EEPROM.end();
    voltGuardEnabled = false;
    voltMode = VOLT_MODE_OFF;
    voltScale = 1.0; voltMin = 200.0; voltMax = 250.0;
    return;
  }

  for (int i = 0; i < MAX_RULES; i++) {
    if (i >= RULES_BLOCK1_COUNT && !haveExt) break;   // extended slots never written
    int base = ruleEepromBase(i);
    rules[i].enabled   = EEPROM.read(base + 0) ? 1 : 0;
    rules[i].mode      = EEPROM.read(base + 1) > 2 ? 0 : EEPROM.read(base + 1);
    rules[i].daysMask  = EEPROM.read(base + 2);
    rules[i].startHour = EEPROM.read(base + 3) % 24;
    rules[i].startMin  = EEPROM.read(base + 4) % 60;
    rules[i].endHour   = EEPROM.read(base + 5) % 24;
    rules[i].endMin    = EEPROM.read(base + 6) % 60;
    rules[i].action    = EEPROM.read(base + 7) ? 1 : 0;
    uint8_t d = EEPROM.read(base + 8);
    uint8_t m = EEPROM.read(base + 9);
    rules[i].day   = (d >= 1 && d <= 31) ? d : 1;
    rules[i].month = (m >= 1 && m <= 12) ? m : 1;
  }

  voltGuardEnabled = EEPROM.read(EEPROM_VOLT_EN_ADDR) ? true : false;
  uint8_t vm = EEPROM.read(EEPROM_VOLT_MODE_ADDR);
  voltMode = (vm <= VOLT_MODE_AC) ? (int)vm : VOLT_MODE_OFF;

  uint16_t sc = EEPROM.read(EEPROM_VOLT_SCALE_ADDR)
              | ((uint16_t)EEPROM.read(EEPROM_VOLT_SCALE_ADDR + 1) << 8);
  voltScale = (sc > 0) ? (sc / 100.0) : 1.0;

  uint16_t vmin = EEPROM.read(EEPROM_VOLT_MIN_ADDR)
                | ((uint16_t)EEPROM.read(EEPROM_VOLT_MIN_ADDR + 1) << 8);
  uint16_t vmax = EEPROM.read(EEPROM_VOLT_MAX_ADDR)
                | ((uint16_t)EEPROM.read(EEPROM_VOLT_MAX_ADDR + 1) << 8);
  voltMin = vmin / 10.0;
  voltMax = vmax / 10.0;
  if (voltMin >= voltMax) { voltMin = 200.0; voltMax = 250.0; }

  EEPROM.end();
}

// ── Voltage measurement ───────────────────────────────────────
// Returns volts, or -1 when sensing is disabled. Nothing here blocks
// for long: the AC path samples for roughly 80 ms, which is short
// enough to sit inside the normal 2 s sensor cycle.
float readVoltage() {
  if (voltMode == VOLT_MODE_OFF) return -1.0;

  if (voltMode == VOLT_MODE_DC) {
    return analogRead(VOLT_PIN) * voltScale;
  }

  // AC: find the signal's DC mid-point, then RMS around it.
  long sum = 0;
  int samples[VOLT_AC_SAMPLES];
  for (int i = 0; i < VOLT_AC_SAMPLES; i++) {
    samples[i] = analogRead(VOLT_PIN);
    sum += samples[i];
    delayMicroseconds(200);
  }
  float mid = (float)sum / VOLT_AC_SAMPLES;

  double sqSum = 0;
  for (int i = 0; i < VOLT_AC_SAMPLES; i++) {
    double d = samples[i] - mid;
    sqSum += d * d;
  }
  float rmsCounts = sqrt(sqSum / VOLT_AC_SAMPLES);
  return rmsCounts * voltScale;
}

// ── Schedule evaluation ───────────────────────────────────────
// Returns the index of the first enabled rule that applies right now, or
// -1 if none.
//
// FIXED: this used to bail out on the very first line whenever the clock
// was not synced, which also killed LEVEL rules (mode 2) even though they
// never look at the clock. Now only the time-based modes are skipped when
// there is no valid clock; level rules keep working regardless.
int activeRuleIndex() {
  time_t now = time(nullptr);
  struct tm* lt = localtime(&now);
  bool clockOk  = timeReady && lt && lt->tm_year >= 100;
  int  nowMins  = clockOk ? (lt->tm_hour * 60 + lt->tm_min) : 0;

  for (int i = 0; i < MAX_RULES; i++) {
    if (!rules[i].enabled) continue;

    // mode 2 = water-level rule: no clock involvement at all, just the
    // current fill % against the threshold stored in daysMask.
    if (rules[i].mode == 2) {
      if (lastPct < 0) continue;              // no valid reading yet
      int thr = rules[i].daysMask;            // 0..100 %

      // Direction comes from startHour (see the struct note above), so it is
      // independent of what the pump is told to do. 0 means the rule predates
      // the setting, in which case the old action-implied meaning still holds
      // and the rule keeps behaving exactly as it did before this update.
      bool above;
      if      (rules[i].startHour == 1) above = false;   // below
      else if (rules[i].startHour == 2) above = true;    // above
      else                              above = !rules[i].action;  // legacy

      bool cond = above ? (lastPct > (float)thr) : (lastPct < (float)thr);
      if (cond) return i;
      continue;
    }

    // Everything below needs a trustworthy wall clock.
    if (!clockOk) continue;

    if (rules[i].mode == 0) {
      if (!(rules[i].daysMask & (1 << lt->tm_wday))) continue;
    } else {
      if (lt->tm_mday != rules[i].day)      continue;
      if ((lt->tm_mon + 1) != rules[i].month) continue;
    }

    int startMins = rules[i].startHour * 60 + rules[i].startMin;
    int endMins   = rules[i].endHour   * 60 + rules[i].endMin;

    bool inWindow;
    if (startMins <= endMins) {
      inWindow = (nowMins >= startMins && nowMins < endMins);
    } else {
      // Window wraps past midnight, e.g. 22:00 -> 06:00
      inWindow = (nowMins >= startMins || nowMins < endMins);
    }
    if (inWindow) return i;
  }
  return -1;
}

// ── OLED ──────────────────────────────────────────────────────
void showMessage(const char* l1, const char* l2 = nullptr,
                 const char* l3 = nullptr) {
  display.clearBuffer();
  display.setFont(u8g2_font_ncenB08_tr);
  display.setCursor(0, 14); display.print(l1);
  if (l2) { display.setCursor(0, 30); display.print(l2); }
  if (l3) { display.setCursor(0, 46); display.print(l3); }
  display.sendBuffer();
}

// Shows upload progress on the OLED during an OTA firmware update.
// Registered with Update.onProgress() — called repeatedly as the .bin
// file is received, with the byte counts so far and the total size.
void showOtaProgress(size_t progress, size_t total) {
  int pct = (total > 0) ? (int)((progress * 100UL) / total) : 0;
  if (pct > 100) pct = 100;

  display.clearBuffer();
  display.setFont(u8g2_font_ncenB08_tr);
  display.setCursor(0, 12); display.print("Upload Firmware");
  display.drawFrame(0, 24, 128, 16);
  int barWidth = (pct * 126) / 100;
  if (barWidth > 0) display.drawBox(1, 25, barWidth, 14);

  char pctStr[8];
  snprintf(pctStr, sizeof(pctStr), "%d%%", pct);
  display.setCursor(52, 56);
  display.print(pctStr);
  display.sendBuffer();
}

void updateDisplay(long dist, float pct, bool relayOnNow, bool mqttOk) {
  display.clearBuffer();
  display.setFont(u8g2_font_ncenB08_tr);
  display.setCursor(0, 10); display.print("Water Tank Monitor");
  display.drawHLine(0, 13, 128);

  if (pct < 0) {
    display.setCursor(0, 32); display.print("Sensor Error");
    display.setCursor(0, 46); display.print("Check HC-SR04");
  } else {
    display.setFont(u8g2_font_ncenB14_tr);
    display.setCursor(0, 36); display.print("Fill:");
    char buf[8];
    snprintf(buf, sizeof(buf), "%d%%", (int)pct);
    display.setCursor(70, 36); display.print(buf);

    display.setFont(u8g2_font_ncenB08_tr);
    display.setCursor(0, 50);
    display.print("Dist:"); display.print(dist);
    display.print("cm D:"); display.print(tankDepthCm); display.print("cm");
    display.setCursor(0, 62);
    display.print("Pump:"); display.print(relayOnNow ? "ON " : "OFF");
    display.print(autoFillEnabled ? " AUTO" : " MAN ");
    display.print(mqttOk ? " OK" : " --");
  }
  display.sendBuffer();
}

void showPairingOLED() {
  IPAddress ip = WiFi.softAPIP();
  display.clearBuffer();
  display.setFont(u8g2_font_ncenB08_tr);
  display.setCursor(0, 10); display.print("-- PAIRING MODE --");
  display.drawHLine(0, 13, 128);
  display.setCursor(0, 26); display.print("Join WiFi:");
  display.setCursor(0, 38); display.print(AP_SSID);
  display.setCursor(0, 50); display.print("Open app, tap WiFi");
  display.setCursor(0, 62); display.print(ip.toString());
  display.sendBuffer();
}

// ── Sensor ────────────────────────────────────────────────────
// Returns -1 only on a genuine sensor failure (no echo received —
// wiring problem, or the target is beyond the sensor's 400 cm range).
long readDistance() {
  digitalWrite(TRIG1, LOW);  delayMicroseconds(2);
  digitalWrite(TRIG1, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG1, LOW);
  long d = pulseIn(ECHO1, HIGH, ECHO_TIMEOUT_US);
  if (d == 0) return -1;
  return d * 0.034 / 2;
}

// Converts a raw distance reading into a 0–100% fill level.
//
// Calibration:
//   - SENSOR_FULL_DISTANCE_CM  → 100% (tank full)
//   - tankDepthCm              → 0%   (tank empty)
//
// A reading exactly at SENSOR_FULL_DISTANCE_CM is 100% full. Anything
// CLOSER than that (distCm < SENSOR_FULL_DISTANCE_CM) is now treated
// as a sensor error rather than clamped — it's outside the sensor's
// valid measuring range and shouldn't be reported as "full". Readings
// at or beyond tankDepthCm are still clamped to 0%. Only genuine
// failures (no echo, or too close) are reported as errors.
float distanceToPercent(long distCm) {
  if (distCm <= 0) return -1;                      // no echo — genuine sensor failure
  if (distCm < SENSOR_FULL_DISTANCE_CM) return -1; // too close — out of valid range

  long range = tankDepthCm - SENSOR_FULL_DISTANCE_CM;
  if (range <= 0) range = 1;  // guards against a misconfigured tank depth

  long clamped = constrain(distCm, (long)SENSOR_FULL_DISTANCE_CM, (long)tankDepthCm);
  float pct = ((float)(tankDepthCm - clamped) / (float)range) * 100.0;
  return constrain(pct, 0.0, 100.0);
}

// ── Telegram alerts (CallMeBot) ───────────────────────────────
// The DEVICE sends every alert itself as a Telegram message, through the
// free CallMeBot service - no Pushy, no API key, no server of our own:
//
//   GET https://api.callmebot.com/text.php?user=@a|@b&text=<urlencoded>&html=yes
//
// and, if enabled, a Telegram VOICE CALL for the critical-low alarm:
//
//   GET https://api.callmebot.com/start.php?user=@a&text=<urlencoded>&lang=..&rpt=2
//
// Telegram delivers through the phone's own push service, so alerts arrive
// even when the AFIF-TECH app is closed or frozen by the phone maker.
//
// Registration (once per person): open https://t.me/CallMeBot_txtbot and
// tap START. Then enter the Telegram @username in the app. That is all.
//
// NOTHING here blocks a callback. Events only ENQUEUE a message (a couple
// of memcpys), and servicePush() does the slow TLS request later, from
// loop(). That matters because applyRelay() is called from inside the MQTT
// callback.
#define TG_HOST          "api.callmebot.com"
#define TG_TEXT_URL      "https://api.callmebot.com/text.php"
#define TG_CALL_URL      "https://api.callmebot.com/start.php"
#define TG_CALL_LANG     "en-US-Standard-B"
#define TG_MAX_USERS     3
#define TG_REG_LINK      "https://t.me/CallMeBot_txtbot"
#define PUSH_QUEUE_LEN   8
#define PUSH_TITLE_MAX   48
#define PUSH_BODY_MAX    112
#define PUSH_MIN_GAP_MS  5000UL  // CallMeBot is free and rate-limited: be gentle
#define SENSOR_FAIL_CYCLES 15    // ~30 s of bad readings before crying wolf

// Which events may notify. Stored as one byte in EEPROM.
#define EV_LOW   0x01            // level at/below CRITICAL_LOW_PCT
#define EV_FULL  0x02            // tank full / fill finished
#define EV_PUMP  0x04            // pump switched ON or OFF
#define EV_FAULT 0x08            // sensor error, voltage guard
#define EV_CALL  0x10            // ALSO ring by Telegram voice call on critical low
#define EV_ALL   0x0F            // all text alerts (the voice call is opt-in)
#define EV_MASK  0x1F

bool     pushEnabled = false;
uint8_t  pushEvents  = EV_ALL;
String   tgUsers     = "";        // "@name1|@name2" - CallMeBot's multi-user format
String   tgLast      = "nothing sent yet";   // result of the last send, for the app
unsigned long tgLastAt = 0;

struct PushMsg {
  char title[PUSH_TITLE_MAX];
  char body[PUSH_BODY_MAX];
  char event[12];
  char callUser[34];             // non-empty = voice call to this one user
};
PushMsg       pushQ[PUSH_QUEUE_LEN];
int           pushHead = 0, pushCount = 0, pushDropped = 0;
unsigned long lastPushAt = 0;
bool          pushMflnProbed = false, pushMfln = false;

// One-shot latches, so a condition that persists notifies once, not every 2 s.
bool fullNotifySent   = false;
bool voltNotifySent   = false;
bool sensorNotifySent = false;
int  sensorFailCycles = 0;

// Adds a text alert to the queue. Safe to call from anywhere, including the
// MQTT callback - it never touches the network.
void queuePush(const char* event, uint8_t bit, const char* title, const char* body) {
  if (!pushEnabled || !(pushEvents & bit)) return;
  if (!tgUsers.length()) return;
  if (pushCount >= PUSH_QUEUE_LEN) { pushDropped++; return; }
  int slot = (pushHead + pushCount) % PUSH_QUEUE_LEN;
  strlcpy(pushQ[slot].event, event, sizeof(pushQ[slot].event));
  strlcpy(pushQ[slot].title, title, sizeof(pushQ[slot].title));
  strlcpy(pushQ[slot].body,  body,  sizeof(pushQ[slot].body));
  pushQ[slot].callUser[0] = 0;
  pushCount++;
}

// Queues one Telegram voice call per registered user (the call API takes a
// single user). Only used for the critical-low alarm, and only if EV_CALL.
void queueCall(const char* text) {
  if (!pushEnabled || !(pushEvents & EV_CALL) || !tgUsers.length()) return;
  int start = 0;
  while (start < (int)tgUsers.length()) {
    int bar = tgUsers.indexOf('|', start);
    if (bar < 0) bar = tgUsers.length();
    String u = tgUsers.substring(start, bar);
    start = bar + 1;
    if (!u.length()) continue;
    if (pushCount >= PUSH_QUEUE_LEN) { pushDropped++; return; }
    int slot = (pushHead + pushCount) % PUSH_QUEUE_LEN;
    strlcpy(pushQ[slot].event, "call", sizeof(pushQ[slot].event));
    strlcpy(pushQ[slot].title, "", sizeof(pushQ[slot].title));
    strlcpy(pushQ[slot].body,  text, sizeof(pushQ[slot].body));
    strlcpy(pushQ[slot].callUser, u.c_str(), sizeof(pushQ[slot].callUser));
    pushCount++;
  }
}

// Minimal JSON string escaping (used by the HTTP endpoints).
String jsonEscape(const char* s) {
  String out;
  for (const char* p = s; *p; p++) {
    char c = *p;
    if (c == '"' || c == '\\') { out += '\\'; out += c; }
    else if (c == '\n')        { out += "\\n"; }
    else if ((uint8_t)c >= 0x20) out += c;      // drop other control chars
  }
  return out;
}

// Percent-encoding for URL query values. UTF-8 bytes (emoji) pass through
// as %XX, which is exactly what CallMeBot expects.
String urlEncode(const char* s) {
  String out;
  char hex[4];
  for (const uint8_t* p = (const uint8_t*)s; *p; p++) {
    uint8_t c = *p;
    if ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') ||
        c == '-' || c == '_' || c == '.' || c == '~') out += (char)c;
    else { snprintf(hex, sizeof(hex), "%%%02X", c); out += hex; }
  }
  return out;
}

// The message is sent with html=yes (for the bold title), so the few
// characters HTML treats specially must be escaped.
String htmlEscape(const char* s) {
  String out;
  for (const char* p = s; *p; p++) {
    if (*p == '<') out += "&lt;";
    else if (*p == '>') out += "&gt;";
    else if (*p == '&') out += "&amp;";
    else out += *p;
  }
  return out;
}

const char* eventIcon(const char* ev) {
  if (!strcmp(ev, "low"))     return "\xE2\x9A\xA0\xEF\xB8\x8F ";   // warning sign
  if (!strcmp(ev, "full"))    return "\xE2\x9C\x85 ";               // check mark
  if (!strcmp(ev, "pump"))    return "\xF0\x9F\x92\xA7 ";           // droplet
  if (!strcmp(ev, "voltage")) return "\xE2\x9A\xA1 ";               // high voltage
  if (!strcmp(ev, "sensor"))  return "\xF0\x9F\x94\xA7 ";           // wrench
  if (!strcmp(ev, "test"))    return "\xF0\x9F\x94\x94 ";           // bell
  return "";
}

bool sendPushNow(const PushMsg& m) {
  if (WiFi.status() != WL_CONNECTED) return false;

  WiFiClientSecure secure;
  secure.setInsecure();          // no cert pinning: the payload is an alert
  // A full-size TLS record buffer costs ~16 kB of heap, which this sketch
  // cannot always spare next to MQTT, the web server and the display. Ask
  // the server ONCE whether it supports a smaller record; if it does, the
  // handshake fits in about 2 kB instead.
  if (!pushMflnProbed) {
    pushMfln = WiFiClientSecure::probeMaxFragmentLength(TG_HOST, 443, 1024);
    pushMflnProbed = true;
    Serial.print("telegram: MFLN supported = "); Serial.println(pushMfln ? "yes" : "no");
  }
  if (pushMfln) secure.setBufferSizes(1024, 512);

  String url;
  bool isCall = m.callUser[0] != 0;
  if (isCall) {
    url = String(TG_CALL_URL) + "?user=" + urlEncode(m.callUser) +
          "&text=" + urlEncode(m.body) + "&lang=" TG_CALL_LANG "&rpt=2";
  } else {
    String text = "<b>" + String(eventIcon(m.event)) + htmlEscape(m.title) + "</b>\n" +
                  htmlEscape(m.body) + "\n\xF0\x9F\x8F\xA0 " + htmlEscape(mqttTopic.c_str());
    url = String(TG_TEXT_URL) + "?user=" + urlEncode(tgUsers.c_str()) +
          "&text=" + urlEncode(text.c_str()) + "&html=yes";
  }

  HTTPClient https;
  if (!https.begin(secure, url)) {
    tgLast = "error: could not start the request"; tgLastAt = millis();
    return false;
  }
  // A voice call keeps the request open while the phone rings. Do not hold
  // the whole device for that: give up waiting after 8 s (the call goes on).
  https.setTimeout(isCall ? 8000 : 10000);
  int code = https.GET();

  // Read at most ~600 bytes of the reply: enough to spot CallMeBot's error
  // text, without buffering a whole web page in the ESP8266's small RAM.
  String reply;
  if (code > 0) {
    WiFiClient* st = https.getStreamPtr();
    unsigned long t0 = millis();
    while (st && reply.length() < 600 && millis() - t0 < 2500) {
      if (st->available()) reply += (char)st->read();
      else if (!st->connected()) break;
      else delay(5);
    }
  }
  https.end();

  String low = reply; low.toLowerCase();
  bool notAuthorized = low.indexOf("not authoriz") >= 0 || low.indexOf("not found") >= 0 ||
                       low.indexOf("authorize callmebot") >= 0;
  tgLastAt = millis();
  if (code <= 0) {
    if (isCall) tgLast = "voice call requested (no reply within 8 s - normal while it rings)";
    else        tgLast = "error: " + HTTPClient::errorToString(code);
  } else if (notAuthorized) {
    tgLast = "NOT AUTHORIZED - open " TG_REG_LINK " in Telegram and tap START";
  } else if (code >= 200 && code < 300) {
    tgLast = isCall ? "voice call started" : ("sent OK (" + String(m.event) + ")");
  } else {
    tgLast = "error: HTTP " + String(code);
  }
  Serial.print("telegram ["); Serial.print(m.event); Serial.print("] HTTP ");
  Serial.print(code); Serial.print(" -> "); Serial.print(tgLast);
  Serial.print(" heap "); Serial.println(ESP.getFreeHeap());
  return code > 0 && code < 300 && !notAuthorized;
}

// Called from loop(). Sends at most one message per PUSH_MIN_GAP_MS, and
// only from here - never from a callback.
void servicePush() {
  if (!pushCount || pairingMode) return;
  if (millis() - lastPushAt < PUSH_MIN_GAP_MS) return;
  if (WiFi.status() != WL_CONNECTED) return;

  lastPushAt = millis();
  PushMsg m = pushQ[pushHead];
  pushHead = (pushHead + 1) % PUSH_QUEUE_LEN;
  pushCount--;

  // The TLS handshake blocks for a second or two. Park the buzzer rather
  // than leaving it stuck mid-beep, then put it back exactly as it was.
  bool wasBuzzing = buzzerState;
  if (wasBuzzing) setBuzzer(false);
  sendPushNow(m);
  if (alarmActive && !alarmMuted && wasBuzzing) setBuzzer(true);
  lastPushAt = millis();         // gap counts from the END of a slow call

  if (pushDropped) {
    Serial.print("telegram: "); Serial.print(pushDropped); Serial.println(" message(s) dropped, queue was full");
    pushDropped = 0;
  }
}

// ── Event detection ───────────────────────────────────────────
// Each of these is a one-shot latch: the condition notifies ONCE and only
// re-arms after it has clearly gone away. Kept as small named functions so
// they can be exercised on their own rather than only in the field.

// Critical-low and tank-full, from one level reading.
void raiseLevelEvents(float pct) {
  if (pct < 0) return;                       // no reading: say nothing

  if (pct <= CRITICAL_LOW_PCT) {
    if (!lowNotifySent) {
      char b[PUSH_BODY_MAX];
      snprintf(b, sizeof(b), "Only %.0f%% left in the tank - pump is %s",
               pct, relayOn ? "running" : "off");
      queuePush("low", EV_LOW, "Water tank critically low", b);
      char c[PUSH_BODY_MAX];
      snprintf(c, sizeof(c), "Warning. Your water tank is critically low, at %.0f percent.", pct);
      queueCall(c);
      lowNotifySent = true;
    }
  } else {
    lowNotifySent = false;
  }

  // Re-arms a little way below the auto-fill maximum, so a level hovering
  // exactly on the threshold cannot notify over and over.
  if (!fullNotifySent && pct >= autoFillMax) {
    char b[PUSH_BODY_MAX];
    snprintf(b, sizeof(b), "Tank is full at %.0f%% - pump %s",
             pct, relayOn ? "still running" : "stopped");
    queuePush("full", EV_FULL, "Tank full", b);
    fullNotifySent = true;
  } else if (fullNotifySent && pct < (autoFillMax - 5)) {
    fullNotifySent = false;
  }
}

// The voltage guard, which reports both the fault and the recovery.
void raiseVoltageEvent() {
  if (voltFault && !voltNotifySent) {
    char b[PUSH_BODY_MAX];
    snprintf(b, sizeof(b), "Supply %.0f V is outside %.0f-%.0f V - pump locked out",
             lastVoltage, voltMin, voltMax);
    queuePush("voltage", EV_FAULT, "Voltage fault", b);
    voltNotifySent = true;
  } else if (!voltFault && voltNotifySent) {
    char b[PUSH_BODY_MAX];
    snprintf(b, sizeof(b), "Supply back to %.0f V - pump released", lastVoltage);
    queuePush("voltage", EV_FAULT, "Voltage normal again", b);
    voltNotifySent = false;
  }
}

// A sensor fault has to persist before it complains — a single bad echo is
// normal and must never wake anybody up.
void raiseSensorEvent(bool badReading) {
  if (!badReading) { sensorFailCycles = 0; sensorNotifySent = false; return; }
  sensorFailCycles++;
  if (sensorFailCycles >= SENSOR_FAIL_CYCLES && !sensorNotifySent) {
    queuePush("sensor", EV_FAULT, "Tank sensor problem",
              "No valid reading for 30 s - check the sensor wiring");
    sensorNotifySent = true;
  }
}

// ── Telegram settings persistence ─────────────────────────────
// Its own validity flag, so flashing this onto a configured device leaves
// WiFi, MQTT, tank and rules untouched and only starts alerts off.
void savePushCfg() {
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.write(EEPROM_TG_EN_ADDR, pushEnabled ? 1 : 0);
  EEPROM.write(EEPROM_TG_EV_ADDR, pushEvents);
  for (int i = 0; i < TG_USERS_LEN; i++)
    EEPROM.write(EEPROM_TG_USERS_ADDR + i, i < (int)tgUsers.length() ? tgUsers[i] : 0);
  EEPROM.write(EEPROM_TG_FLAG_ADDR, EEPROM_TG_VALID_FLAG);
  EEPROM.commit();
  EEPROM.end();
}

void loadPushCfg() {
  EEPROM.begin(EEPROM_SIZE);
  if (EEPROM.read(EEPROM_TG_FLAG_ADDR) != EEPROM_TG_VALID_FLAG) {
    EEPROM.end();
    pushEnabled = false; pushEvents = EV_ALL; tgUsers = "";
    return;
  }
  pushEnabled = EEPROM.read(EEPROM_TG_EN_ADDR) ? true : false;
  pushEvents  = EEPROM.read(EEPROM_TG_EV_ADDR) & EV_MASK;
  tgUsers = "";
  for (int i = 0; i < TG_USERS_LEN - 1; i++) {
    char c = EEPROM.read(EEPROM_TG_USERS_ADDR + i);
    if (!c) break; tgUsers += c;
  }
  EEPROM.end();
}

// Accepts "@ali, @sara  ahmad" or "@ali|@sara" (the @ is added if missing)
// and produces CallMeBot's "@ali|@sara|@ahmad". Telegram usernames are 5-32
// letters, digits or underscores. Returns an error text, or "" when valid.
String normalizeTgUsers(const String& in, String& out) {
  out = "";
  int count = 0;
  String cur;
  for (unsigned int i = 0; i <= in.length(); i++) {
    char c = i < in.length() ? in[i] : ' ';
    bool sep = (c == ',' || c == ';' || c == '|' || c == ' ' || c == '\n' || c == '\t');
    if (!sep) { cur += c; continue; }
    if (!cur.length()) continue;
    if (cur[0] == '@') cur = cur.substring(1);
    if (cur.length() < 5 || cur.length() > 32) return "each username must be 5-32 characters";
    for (unsigned int k = 0; k < cur.length(); k++) {
      char d = cur[k];
      bool ok = (d >= 'a' && d <= 'z') || (d >= 'A' && d <= 'Z') || (d >= '0' && d <= '9') || d == '_';
      if (!ok) return "usernames may only contain letters, digits and _";
    }
    if (++count > TG_MAX_USERS) return "at most 3 usernames";
    if (out.length()) out += '|';
    out += '@'; out += cur;
    cur = "";
  }
  if ((int)out.length() >= TG_USERS_LEN) return "usernames too long";
  return "";
}

// ── Telegram HTTP endpoints ───────────────────────────────────
// GET /telegram                      -> current settings + result of last send
// GET /telegramset?en=&ev=&users=    -> save (users: "@a,@b", max 3)
// GET /telegramtest[?call=1]         -> queue a test message (or test call)
void handleTgGet() {
  String j = "{\"en\":" + String(pushEnabled ? 1 : 0);
  j += ",\"ev\":"      + String(pushEvents);
  j += ",\"users\":\"" + jsonEscape(tgUsers.c_str()) + "\"";
  j += ",\"queued\":"  + String(pushCount);
  j += ",\"last\":\""  + jsonEscape(tgLast.c_str()) + "\"";
  j += ",\"lastAgo\":" + String(tgLastAt ? (long)((millis() - tgLastAt) / 1000) : -1);
  j += ",\"register\":\"" TG_REG_LINK "\"";
  j += "}";
  sendCORS();
  server.send(200, "application/json", j);
}

void handleTgSet() {
  if (server.hasArg("users")) {
    String norm;
    String err = normalizeTgUsers(server.arg("users"), norm);
    if (err.length()) {
      sendCORS();
      server.send(400, "application/json", "{\"ok\":false,\"err\":\"" + jsonEscape(err.c_str()) + "\"}");
      return;
    }
    tgUsers = norm;
  }
  if (server.hasArg("en")) pushEnabled = server.arg("en").toInt() ? true : false;
  if (server.hasArg("ev")) pushEvents  = (uint8_t)(server.arg("ev").toInt() & EV_MASK);
  if (pushEnabled && !tgUsers.length()) {
    sendCORS();
    server.send(400, "application/json", "{\"ok\":false,\"err\":\"enter at least one Telegram username\"}");
    return;
  }
  savePushCfg();
  Serial.print("telegram settings saved, enabled="); Serial.print(pushEnabled ? 1 : 0);
  Serial.print(" users="); Serial.println(tgUsers);
  handleTgGet();
}

void handleTgTest() {
  if (!pushEnabled || !tgUsers.length()) {
    sendCORS();
    server.send(400, "application/json", "{\"ok\":false,\"err\":\"Telegram alerts are off or no username saved\"}");
    return;
  }
  // Bypasses the event mask on purpose: a test must always go out.
  uint8_t keep = pushEvents; pushEvents = EV_MASK;
  char b[PUSH_BODY_MAX];
  if (server.hasArg("call") && server.arg("call").toInt()) {
    queueCall("This is a test call from your AFIF-TECH water tank.");
  } else {
    if (lastPct >= 0) snprintf(b, sizeof(b), "Test from your tank - level %.0f%%, pump %s", lastPct, relayOn ? "ON" : "OFF");
    else              snprintf(b, sizeof(b), "Test from your tank - sensor not reading yet");
    queuePush("test", EV_MASK, "AFIF-TECH test", b);
  }
  pushEvents = keep;
  tgLast = "test queued - sending within a few seconds"; tgLastAt = millis();
  handleTgGet();
}

// ── Relay helper ──────────────────────────────────────────────
// Drives the physical relay, updates the shared state, and publishes
// the new status (retained) so the phone app can always sync to the
// device's actual state — whether it changed via auto-fill logic or
// a manual MQTT command.
// Payload convention for the relay channel is REVERSED at the MQTT
// level, independent of the physical GPIO polarity below: "0" = ON,
// "1" = OFF. This applies to both the command topic (<topic>/relay/set)
// and the status echo (<topic>/relay).
void applyRelay(bool state, const char* reason) {
  bool changed = (state != relayOn);
  relayOn = state;
  digitalWrite(RELAY1, relayOn ? HIGH : LOW);   // active-HIGH relay board (NPN low-side driver)
  mqtt.publish(relayStatusTopic.c_str(), relayOn ? "0" : "1", true);

  // Only a real change is worth a notification. queuePush() never touches the
  // network, which is what makes this safe to call from the MQTT callback.
  if (changed) {
    char body[PUSH_BODY_MAX];
    if (lastPct >= 0)
      snprintf(body, sizeof(body), "Pump %s (%s) - level %.0f%%", relayOn ? "ON" : "OFF", reason, lastPct);
    else
      snprintf(body, sizeof(body), "Pump %s (%s)", relayOn ? "ON" : "OFF", reason);
    queuePush("pump", EV_PUMP, relayOn ? "Pump turned ON" : "Pump turned OFF", body);
  }
}

// An overload rather than a default argument: Arduino generates its own
// prototypes and a default here can clash with them.
void applyRelay(bool state) { applyRelay(state, "manual"); }

// ── Buzzer / low-level alarm ──────────────────────────────────
// Drives the buzzer pin directly. Kept as a helper so every path that
// silences the alarm goes through one place and can never leave the
// buzzer stuck on.
void setBuzzer(bool on) {
  buzzerState = on;
  digitalWrite(BUZZER, on ? BUZZER_ON : BUZZER_OFF);
}

// Publishes the alarm state the app should show. "1" only while the
// alarm is genuinely sounding — a muted alarm reports "0" so the app's
// banner clears in step with the buzzer going quiet.
void publishAlarmState() {
  mqtt.publish(alarmStatusTopic.c_str(),
               (alarmActive && !alarmMuted) ? "1" : "0", true);
}

// Silences the current alarm episode from either source (device
// double-click or the app's Mute button). Does NOT clear alarmActive:
// the tank is still low, we're just not making noise about it. The mute
// lifts by itself in evaluateAlarm() once the level recovers.
void muteAlarm(const char* source) {
  if (!alarmActive) return;   // nothing to mute
  alarmMuted = true;
  setBuzzer(false);
  publishAlarmState();
  Serial.print("Alarm muted by "); Serial.println(source);
}

// Called once per sensor reading with the current fill %.
// Starts the alarm when the level drops to the threshold, and clears it
// (including any mute) once the level recovers above the threshold.
void evaluateAlarm(float pct) {
  if (pct < 0) return;   // sensor error — leave the alarm state untouched

  if (pct <= ALARM_LEVEL_PCT) {
    if (!alarmActive) {
      alarmActive = true;
      alarmMuted  = false;     // fresh episode always alarms
      lastBeepToggle = millis();
      setBuzzer(true);
      publishAlarmState();
      Serial.println("ALARM: water level critical");
    }
  } else {
    if (alarmActive) {
      alarmActive = false;
      alarmMuted  = false;     // reset so the next low episode alarms again
      setBuzzer(false);
      publishAlarmState();
      Serial.println("ALARM: cleared, level recovered");
    }
  }
}

// Non-blocking beep pattern — called every loop() iteration. Toggles the
// buzzer on/off on the BEEP_ON_MS / BEEP_OFF_MS schedule without ever
// using delay(), so the sensor, MQTT, web server and button all keep
// running normally while the alarm is sounding.
void serviceBuzzer() {
  if (!alarmActive || alarmMuted) {
    if (buzzerState) setBuzzer(false);
    return;
  }
  unsigned long interval = buzzerState ? BEEP_ON_MS : BEEP_OFF_MS;
  if (millis() - lastBeepToggle >= interval) {
    lastBeepToggle = millis();
    setBuzzer(!buzzerState);
  }
}

// Called from a short press of the pairing button (released before
// PAIR_HOLD_MS). Mirrors exactly what an MQTT manual-toggle command does —
// applyRelay() already publishes the new state, so the app's toggle switch
// updates the same way whether the change came from the button or the app.
//   - Auto-fill OFF: plain manual toggle, same as tapping the app's switch.
//   - Auto-fill ON:  force-starts a fill cycle right now (ignores the Min%
//     threshold); the normal hysteresis in loop() still turns it back off
//     automatically once the tank reaches Max%, exactly like a scheduled
//     auto-fill cycle would.
void toggleRelayButton() {
  if (voltFault) {
    showMessage("Voltage fault", "Pump locked out");
    delay(900);
    return;
  }
  if (autoFillEnabled) {
    fillActive = true;
    applyRelay(true, "button");
    showMessage("Auto-Fill", "Fill cycle started", "(button)");
  } else {
    applyRelay(!relayOn, "button");
    showMessage("Manual Pump", relayOn ? "Turned ON" : "Turned OFF", "(button)");
  }
  delay(800);   // let the confirmation message be readable before returning to loop()
}

// ── MQTT command handling ─────────────────────────────────────
// Two live commands, both retained-echoed back so the app can confirm:
//   <topic>/relay/set     "0"/"1" — manual relay override, reversed
//                          convention ("0"=ON, "1"=OFF, see applyRelay()
//                          above) — honoured only while auto-fill is
//                          DISABLED
//   <topic>/autofill/set  "1"/"0" — enable/disable auto-fill; persisted
//                          to EEPROM immediately (normal convention:
//                          "1"=enabled, "0"=disabled)
//   <topic>/alarm/set     "1"     — mute the low-level alarm. Sent when the
//                          user taps Mute in the app, so the buzzer and the
//                          app's alarm silence together.
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String t(topic);
  String msg;
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];
  msg.trim();

  if (t == relayCmdTopic) {
    if (voltFault) {
      // Supply out of range - refuse to energise and correct the app's UI.
      mqtt.publish(relayStatusTopic.c_str(), relayOn ? "0" : "1", true);
      Serial.println("Relay command ignored: voltage fault");
    } else if (!autoFillEnabled) {
      applyRelay(msg == "0", "app");   // reversed: "0" = ON, "1" = OFF
    } else {
      // Auto-fill owns the relay right now — re-publish the real
      // state so the app's optimistic UI snaps back to reality.
      mqtt.publish(relayStatusTopic.c_str(), relayOn ? "0" : "1", true);
    }
  } else if (t == alarmCmdTopic) {
    if (msg == "1") muteAlarm("app");
  } else if (t == rulesCmdTopic) {
    handleRulesMqtt(msg);
  } else if (t == otaCmdTopic) {
    handleOtaCommand(msg);          // only checks + queues; download runs in loop()
  } else if (t == autofillCmdTopic) {
    autoFillEnabled = (msg == "1");
    saveAutoFillEnabled(autoFillEnabled);
    mqtt.publish(autofillStatusTopic.c_str(), autoFillEnabled ? "1" : "0", true);
    Serial.print("Auto-fill "); Serial.println(autoFillEnabled ? "ENABLED" : "DISABLED");
  }
}

// ── WiFi connect ──────────────────────────────────────────────
// Returns false either on a real WiFi timeout, OR because the button
// was held and pairing mode is now already active (check pairingMode
// flag to tell which one happened).
bool connectWiFi(const String& ssid, const String& pass) {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid.c_str(), pass.c_str());
  showMessage("Connecting WiFi...", ssid.c_str());
  Serial.print("WiFi");
  int t = 0;
  while (WiFi.status() != WL_CONNECTED) {
    if (pollPairingButton()) return false;  // button wins — no waiting on WiFi
    delay(500); Serial.print(".");
    if (++t > 30) {
      Serial.println(" timeout");
      showMessage("WiFi timeout!", "Hold D3 1s for", "pairing mode");
      return false;
    }
  }
  Serial.println(" OK");
  showMessage("WiFi Connected!", WiFi.localIP().toString().c_str());
  delay(1500);
  return true;
}

// ── MQTT connect ──────────────────────────────────────────────
// Checks the pairing button constantly — including during the retry
// delay — so a held button interrupts this immediately instead of
// waiting for MQTT to succeed, fail, or give up and restart.
void connectMQTT() {
  int attempts = 0;
  while (!mqtt.connected()) {
    if (pollPairingButton()) return;  // button wins — abandon MQTT entirely

    showMessage("MQTT connecting...", mqttBroker.c_str());
    Serial.print("MQTT...");
    // Last will: if this device drops off the network, the BROKER publishes
    // "offline" on <topic>/status for every app to see. The device itself
    // cannot notify you about its own death — something always-on must.
    String clientId = String(MQTT_CLIENT) + String(ESP.getChipId(), HEX);
    if (mqtt.connect(clientId.c_str(), statusTopic.c_str(), 0, true, "offline")) {
      Serial.println("OK");
      showMessage("MQTT Connected", "");
      delay(600);
      mqtt.subscribe(relayCmdTopic.c_str());
      mqtt.subscribe(autofillCmdTopic.c_str());
      mqtt.subscribe(alarmCmdTopic.c_str());
      mqtt.subscribe(rulesCmdTopic.c_str());
      mqtt.subscribe(otaCmdTopic.c_str());
      // Publish current state immediately so the app syncs on open
      mqtt.publish(relayStatusTopic.c_str(), relayOn ? "0" : "1", true);        // reversed: "0"=ON
      mqtt.publish(autofillStatusTopic.c_str(), autoFillEnabled ? "1" : "0", true);
      publishAlarmState();
      publishRules();
      mqtt.publish(versionTopic.c_str(), FW_VERSION, true);
      mqtt.publish(statusTopic.c_str(), "online", true);
    } else {
      Serial.print("fail rc="); Serial.println(mqtt.state());
      if (++attempts > 5) {
        showMessage("MQTT failed", "Restarting...");
        delay(2000); ESP.restart();
      }
      // Same 5s backoff as before, but split into 100ms slices so the
      // button is still checked continuously instead of one long block.
      for (int i = 0; i < 50; i++) {
        if (pollPairingButton()) return;
        delay(100);
      }
    }
  }
}

// ── Portal HTML ───────────────────────────────────────────────
static const char PORTAL_HTML[] PROGMEM = R"RAWHTML(
<!DOCTYPE html><html lang="en"><head>
<meta charset="UTF-8"/>
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no"/>
<title>AFIF-TECH Setup</title>
<style>
*,*::before,*::after{box-sizing:border-box;margin:0;padding:0}
body{min-height:100vh;background:#0f172a;display:flex;flex-direction:column;align-items:center;font-family:Arial,sans-serif;color:#e2e8f0;padding:24px 16px 40px}
.brand{font-size:.7rem;font-weight:700;letter-spacing:3px;color:#38bdf8;text-transform:uppercase;margin-bottom:4px}
h1{font-size:1.2rem;font-weight:800;color:#f1f5f9;margin-bottom:2px}
.sub{font-size:.65rem;color:#64748b;letter-spacing:1px;margin-bottom:22px}
.card{width:100%;max-width:360px;background:#1e293b;border:1px solid #334155;border-radius:16px;padding:18px;margin-bottom:14px}
.card-title{font-size:.6rem;font-weight:700;letter-spacing:2px;color:#38bdf8;text-transform:uppercase;margin-bottom:12px}
.net-list{display:flex;flex-direction:column;gap:7px;max-height:200px;overflow-y:auto}
.net-item{display:flex;align-items:center;justify-content:space-between;padding:9px 12px;background:#0f172a;border:1px solid #334155;border-radius:10px;cursor:pointer;transition:border-color .2s}
.net-item:hover,.net-item.sel{border-color:#38bdf8;background:#0c1a2e}
.net-name{font-size:.82rem;font-weight:600;color:#f1f5f9}
.net-right{display:flex;align-items:center;gap:5px;font-size:.6rem;color:#64748b}
label{display:block;font-size:.6rem;font-weight:700;letter-spacing:1px;color:#94a3b8;margin:12px 0 5px}
input{width:100%;background:#0f172a;border:1px solid #334155;border-radius:8px;padding:9px 12px;font-size:.88rem;color:#f1f5f9;outline:none;transition:border-color .2s}
input:focus{border-color:#38bdf8}
.input-row{display:flex;align-items:center;gap:8px}
.input-row input{flex:1}
.unit{font-size:.75rem;color:#64748b;white-space:nowrap}
.hint{font-size:.6rem;color:#475569;margin-top:4px;line-height:1.5}
.btn{width:100%;margin-top:14px;padding:11px;border:none;border-radius:10px;font-size:.82rem;font-weight:700;cursor:pointer;letter-spacing:1px;transition:background .2s}
.btn.primary{background:#0369a1;color:#fff}.btn.primary:hover{background:#0284c7}
.btn.primary:disabled{background:#334155;color:#64748b;cursor:not-allowed}
.btn.ghost{background:transparent;border:1px solid #334155;color:#94a3b8}.btn.ghost:hover{border-color:#64748b;color:#e2e8f0}
.btn.danger{background:transparent;border:1px solid #7f1d1d;color:#f87171}.btn.danger:hover{background:rgba(127,29,19,.15)}
.status{margin-top:10px;padding:9px 12px;border-radius:8px;font-size:.72rem;font-weight:600;display:none;text-align:center}
.ok  {background:rgba(22,163,74,.15);border:1px solid rgba(22,163,74,.4);color:#4ade80;display:block}
.err {background:rgba(220,38,38,.15);border:1px solid rgba(220,38,38,.4);color:#f87171;display:block}
.info{background:rgba(56,189,248,.1);border:1px solid rgba(56,189,248,.3);color:#38bdf8;display:block}
.spinner{display:none;width:18px;height:18px;border:2px solid #334155;border-top-color:#38bdf8;border-radius:50%;animation:spin .7s linear infinite;margin:10px auto}
@keyframes spin{to{transform:rotate(360deg)}}
.saved-row{display:flex;align-items:center;justify-content:space-between;padding:6px 0;border-bottom:1px solid #1e293b}
.saved-lbl{font-size:.65rem;color:#64748b}.saved-val{font-size:.78rem;font-weight:600;color:#f1f5f9}
.divider{height:1px;background:#334155;margin:14px 0}
</style></head><body>
<div class="brand">⬡ AFIF-TECH</div>
<h1>Device Setup</h1>
<div class="sub">WATER TANK MONITOR — PAIRING MODE</div>

<!-- Current saved settings -->
<div class="card" id="savedCard" style="display:none">
  <div class="card-title">Current Saved Settings</div>
  <div class="saved-row"><span class="saved-lbl">WiFi SSID</span><span class="saved-val" id="sv-ssid">—</span></div>
  <div class="saved-row"><span class="saved-lbl">Tank Depth</span><span class="saved-val" id="sv-depth">—</span></div>
  <div class="saved-row"><span class="saved-lbl">MQTT Broker</span><span class="saved-val" id="sv-broker">—</span></div>
  <div class="saved-row"><span class="saved-lbl">MQTT Topic</span><span class="saved-val" id="sv-topic">—</span></div>
  <div class="saved-row"><span class="saved-lbl">Auto-Fill Min/Max</span><span class="saved-val" id="sv-autofill">—</span></div>
</div>

<!-- WiFi scan -->
<div class="card">
  <div class="card-title">Available Networks</div>
  <div class="net-list" id="netList">
    <div style="color:#64748b;font-size:.8rem;text-align:center;padding:12px">Tap Scan to search...</div>
  </div>
  <button class="btn ghost" style="margin-top:10px" onclick="doScan()">↻ Scan Networks</button>
</div>

<!-- WiFi credentials + Tank depth + MQTT + Auto-Fill — all in one form -->
<div class="card">
  <div class="card-title">WiFi, Tank, MQTT &amp; Auto-Fill Settings</div>

  <label>Network Name (SSID)</label>
  <input type="text" id="ssid" placeholder="Select above or type SSID"/>

  <label>WiFi Password</label>
  <input type="password" id="pass" placeholder="Leave blank for open networks"/>

  <div class="divider"></div>

  <label>Tank Height (depth from sensor to floor)</label>
  <div class="input-row">
    <input type="number" id="depth" min="30" max="1000" placeholder="100"/>
    <span class="unit">cm</span>
  </div>
  <p class="hint">
    Measure the distance from the sensor face down to the bottom of the tank
    when it is completely empty. Range: 30 – 1000 cm (the sensor needs at
    least 20 cm of clearance to the water surface at "full").
  </p>

  <div class="divider"></div>

  <label>MQTT Broker</label>
  <input type="text" id="mqttBroker" placeholder="broker.emqx.io"/>

  <label>MQTT Port</label>
  <input type="number" id="mqttPort" placeholder="1883"/>

  <label>MQTT Topic</label>
  <input type="text" id="mqttTopic" placeholder="SensorData"/>

  <div class="divider"></div>

  <label>Auto-Fill Thresholds</label>
  <div class="input-row">
    <input type="number" id="autoFillMin" min="0" max="100" placeholder="20"/>
    <span class="unit">% Min (pump ON)</span>
  </div>
  <div class="input-row" style="margin-top:8px">
    <input type="number" id="autoFillMax" min="0" max="100" placeholder="90"/>
    <span class="unit">% Max (pump OFF)</span>
  </div>
  <p class="hint">
    With Auto Fill enabled from the app, the pump turns ON at Min% and OFF at Max%.
  </p>

  <button class="btn primary" id="connBtn" onclick="doSaveAll()">
    Save &amp; Connect
  </button>
  <div class="spinner" id="spinner"></div>
  <div class="status" id="status"></div>
</div>

<!-- Danger zone -->
<div class="card">
  <div class="card-title">Device Actions</div>
  <button class="btn danger" onclick="doClear()">⚠ Clear All Settings &amp; Restart</button>
</div>

<script>
function rssiBar(r){return r>=-55?'▂▄▆█':r>=-70?'▂▄▆':r>=-80?'▂▄':'▂'}

function doScan(){
  document.getElementById('netList').innerHTML=
    '<div style="color:#64748b;font-size:.8rem;text-align:center;padding:12px">Starting scan...</div>';
  fetch('/scan').then(()=>{
    setTimeout(()=>pollScan(0), 500);
  }).catch(()=>{
    document.getElementById('netList').innerHTML=
      '<div style="color:#f87171;font-size:.8rem;text-align:center;padding:12px">Scan failed — try again</div>';
  });
}

function pollScan(attempt){
  const list = document.getElementById('netList');
  if (attempt > 20) {
    list.innerHTML =
      '<div style="color:#f87171;font-size:.8rem;text-align:center;padding:12px">Scan timed out — try again</div>';
    return;
  }
  fetch('/scanresult').then(r=>r.json()).then(data=>{
    if (data.status === 'scanning') {
      list.innerHTML =
        '<div style="color:#64748b;font-size:.8rem;text-align:center;padding:12px">Scanning'+'.'.repeat((attempt%3)+1)+'</div>';
      setTimeout(()=>pollScan(attempt+1), 700);
      return;
    }
    if (data.status !== 'done' || !data.networks) {
      list.innerHTML =
        '<div style="color:#f87171;font-size:.8rem;text-align:center;padding:12px">Scan failed — try again</div>';
      return;
    }
    const nets = data.networks;
    if(!nets.length){
      list.innerHTML=
        '<div style="color:#64748b;font-size:.8rem;text-align:center;padding:12px">No networks found</div>';
      return;
    }
    nets.sort((a,b)=>b.rssi-a.rssi);
    list.innerHTML=nets.map(n=>`
      <div class="net-item" onclick="pick('${n.ssid.replace(/'/g,"\\'")}')">
        <span class="net-name">${n.ssid}</span>
        <span class="net-right">${rssiBar(n.rssi)} ${n.secure?'🔒':''}</span>
      </div>`).join('');
  }).catch(()=>{
    list.innerHTML=
      '<div style="color:#f87171;font-size:.8rem;text-align:center;padding:12px">Scan failed — try again</div>';
  });
}

function pick(ssid){
  document.getElementById('ssid').value=ssid;
  document.querySelectorAll('.net-item').forEach(el=>
    el.classList.toggle('sel',el.querySelector('.net-name').textContent===ssid));
  document.getElementById('pass').focus();
}

function showSt(id,msg,type){
  const el=document.getElementById(id);
  el.textContent=msg; el.className='status '+type;
}

function doSaveAll(){
  const ssid=document.getElementById('ssid').value.trim();
  const pass=document.getElementById('pass').value;
  const depth=parseInt(document.getElementById('depth').value)||0;
  const mqttBroker=document.getElementById('mqttBroker').value.trim()||'broker.emqx.io';
  const mqttPort=parseInt(document.getElementById('mqttPort').value)||1883;
  const mqttTopic=document.getElementById('mqttTopic').value.trim()||'SensorData';
  const autoFillMin=parseInt(document.getElementById('autoFillMin').value);
  const autoFillMax=parseInt(document.getElementById('autoFillMax').value);
  if(!ssid){showSt('status','Enter a network name','err');return;}
  if(depth<30||depth>1000){showSt('status','Tank height must be 30–1000 cm','err');return;}
  if(!isNaN(autoFillMin) && !isNaN(autoFillMax) && autoFillMin>=autoFillMax){
    showSt('status','Auto-Fill Min must be less than Max','err');return;
  }
  document.getElementById('connBtn').disabled=true;
  document.getElementById('spinner').style.display='block';
  showSt('status','Connecting and saving — please wait up to 15s...','info');
  fetch('/connect',{method:'POST',
    headers:{'Content-Type':'application/x-www-form-urlencoded'},
    body:'ssid='+encodeURIComponent(ssid)+'&pass='+encodeURIComponent(pass)+'&depth='+depth+
         '&mqttBroker='+encodeURIComponent(mqttBroker)+'&mqttPort='+mqttPort+
         '&mqttTopic='+encodeURIComponent(mqttTopic)+
         '&autoFillMin='+(isNaN(autoFillMin)?'':autoFillMin)+
         '&autoFillMax='+(isNaN(autoFillMax)?'':autoFillMax)
  }).then(r=>r.json()).then(res=>{
    document.getElementById('spinner').style.display='none';
    document.getElementById('connBtn').disabled=false;
    if(res.ok){showSt('status','Saved! Device is restarting...','ok');}
    else{showSt('status','WiFi connection failed. Check password and try again.','err');}
  }).catch(()=>{
    document.getElementById('spinner').style.display='none';
    document.getElementById('connBtn').disabled=false;
    showSt('status','No response — device may be restarting (normal).','info');
  });
}

function doClear(){
  if(!confirm('Clear all saved settings and restart?'))return;
  fetch('/clear').then(()=>showSt('status','Cleared. Restarting...','ok')).catch(()=>{});
}

// Load saved settings on page open
fetch('/settings').then(r=>r.json()).then(d=>{
  if(d.ssid||d.depth){
    document.getElementById('savedCard').style.display='block';
    document.getElementById('sv-ssid').textContent=d.ssid||'(none)';
    document.getElementById('sv-depth').textContent=d.depth?d.depth+' cm':'(default)';
    if(d.ssid) document.getElementById('ssid').value=d.ssid;
    if(d.depth) document.getElementById('depth').value=d.depth;
  }
  if(d.mqttBroker){
    document.getElementById('savedCard').style.display='block';
    document.getElementById('sv-broker').textContent=d.mqttBroker;
    document.getElementById('sv-topic').textContent=d.mqttTopic||'—';
    document.getElementById('mqttBroker').value=d.mqttBroker;
  }
  if(d.mqttPort)  document.getElementById('mqttPort').value=d.mqttPort;
  if(d.mqttTopic) document.getElementById('mqttTopic').value=d.mqttTopic;
  if(d.autoFillMin !== undefined && d.autoFillMax !== undefined){
    document.getElementById('savedCard').style.display='block';
    document.getElementById('sv-autofill').textContent=d.autoFillMin+'% / '+d.autoFillMax+'%';
    document.getElementById('autoFillMin').value=d.autoFillMin;
    document.getElementById('autoFillMax').value=d.autoFillMax;
  }
}).catch(()=>{});

doScan();
</script>
</body></html>
)RAWHTML";

// ── CORS helper ───────────────────────────────────────────────
void sendCORS() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
  server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
}

// ── Web server handlers ───────────────────────────────────────
void handleRoot() {
  server.send_P(200, "text/html", PORTAL_HTML);
}

void handleScan() {
  if (WiFi.getMode() != WIFI_AP_STA) {
    WiFi.mode(WIFI_AP_STA);
    delay(100);
  }
  if (WiFi.scanComplete() != WIFI_SCAN_RUNNING) {
    WiFi.scanNetworksAsync([](int){}, true);
  }
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", "{\"status\":\"scanning\"}");
}

void handleScanResult() {
  int n = WiFi.scanComplete();
  server.sendHeader("Access-Control-Allow-Origin", "*");

  if (n == WIFI_SCAN_RUNNING) {
    server.send(200, "application/json", "{\"status\":\"scanning\"}");
    return;
  }
  if (n == WIFI_SCAN_FAILED) {
    server.send(200, "application/json", "{\"status\":\"error\"}");
    return;
  }

  String json = "{\"status\":\"done\",\"networks\":[";
  for (int i = 0; i < n; i++) {
    if (i) json += ",";
    json += "{\"ssid\":\"" + WiFi.SSID(i) + "\","
            "\"rssi\":"   + String(WiFi.RSSI(i)) + ","
            "\"secure\":"  + (WiFi.encryptionType(i) != ENC_TYPE_NONE ? "true" : "false") + "}";
  }
  json += "]}";
  WiFi.scanDelete();
  server.send(200, "application/json", json);
}

// ── /rules endpoints ──────────────────────────────────────────
// GET  /rules              -> every slot + voltage guard config + live readings
// POST /rules              -> write or delete ONE slot, JSON body
//                             {"id":N, ...fields}  or  {"id":N,"delete":1}
//                             (also still accepts the legacy bulk form body)
// GET  /ruleset?id=N&...   -> same single-slot write, query string only
// GET  /ruledel?id=N       -> clear ONE slot
// GET  /rulesclear         -> clear ALL slots
//
// Every one of these replies with the SAME payload as GET /rules, so the
// app always redraws from what the device actually stored rather than
// from what it hoped it stored.
// The full rules payload. Served over HTTP by GET /rules and published over
// MQTT on <topic>/rules — deliberately the SAME JSON, so both apps parse it
// identically.
String rulesJson() {
  String json = "{\"timeReady\":";
  json += timeReady ? "true" : "false";

  if (timeReady) {
    time_t now = time(nullptr);
    struct tm* lt = localtime(&now);
    char buf[32];
    snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d",
             lt->tm_year + 1900, lt->tm_mon + 1, lt->tm_mday,
             lt->tm_hour, lt->tm_min);
    json += ",\"now\":\"" + String(buf) + "\"";
  } else {
    json += ",\"now\":\"\"";
  }

  json += ",\"activeRule\":" + String(activeRuleIndex());
  json += ",\"max\":"        + String(MAX_RULES);
  json += ",\"voltGuard\":" + String(voltGuardEnabled ? 1 : 0);
  json += ",\"voltMode\":"  + String(voltMode);
  json += ",\"voltScale\":" + String(voltScale, 3);
  json += ",\"voltMin\":"   + String(voltMin, 1);
  json += ",\"voltMax\":"   + String(voltMax, 1);
  json += ",\"voltage\":"   + String(lastVoltage, 1);
  json += ",\"voltFault\":" + String(voltFault ? 1 : 0);
  json += ",\"rules\":[";
  for (int i = 0; i < MAX_RULES; i++) {
    if (i) json += ",";
    json += "{\"en\":"     + String(rules[i].enabled);
    json += ",\"mode\":"   + String(rules[i].mode);
    json += ",\"days\":"   + String(rules[i].daysMask);
    json += ",\"sh\":"     + String(rules[i].startHour);
    json += ",\"sm\":"     + String(rules[i].startMin);
    json += ",\"eh\":"     + String(rules[i].endHour);
    json += ",\"em\":"     + String(rules[i].endMin);
    json += ",\"act\":"    + String(rules[i].action);
    json += ",\"day\":"    + String(rules[i].day);
    json += ",\"month\":"  + String(rules[i].month) + "}";
  }
  json += "]}";
  return json;
}

void handleRulesGet() {
  sendCORS();
  server.send(200, "application/json", rulesJson());
}

// Publishes the rules payload, RETAINED, so an app that opens later gets
// the current state immediately instead of having to ask for it. The
// payload is ~900 bytes, which is why finishOnlineSetup() raises the MQTT
// buffer — PubSubClient's default of 256 bytes would silently drop it.
void publishRules() {
  if (!mqtt.connected()) return;
  String json = rulesJson();
  if (!mqtt.publish(rulesStatusTopic.c_str(), json.c_str(), true)) {
    Serial.print("Rules publish FAILED (");
    Serial.print(json.length());
    Serial.println(" bytes) - is the MQTT buffer big enough?");
  }
}

// Shared by GET /settime and the MQTT {"epoch":...} command.
void setClockFromEpoch(long epoch) {
  struct timeval tv;
  tv.tv_sec  = (time_t)epoch;
  tv.tv_usec = 0;
  settimeofday(&tv, nullptr);
  timeReady = true;
  Serial.print("Clock set from app, epoch "); Serial.println(epoch);
}

// Pulls one unsigned integer field out of a flat JSON object.
// Returns `dflt` when the field is missing, so a PARTIAL body only
// changes the fields it actually carries instead of silently zeroing
// the rest of the rule.
static int jsonInt(const String& body, const char* field, int dflt) {
  String key = String("\"") + field + "\":";
  int pos = body.indexOf(key);
  if (pos == -1) return dflt;
  pos += key.length();
  while (pos < (int)body.length() && (body[pos] == ' ' || body[pos] == '"')) pos++;
  int end = pos;
  while (end < (int)body.length() && body[end] >= '0' && body[end] <= '9') end++;
  if (end == pos) return dflt;                 // present but not a number
  return body.substring(pos, end).toInt();
}

// Writes one slot from a flat JSON object. Every field is optional and
// every value is clamped, so a truncated or malformed body can leave a
// rule unchanged but can never corrupt one.
void applyRuleJson(int id, const String& body) {
  Rule& r = rules[id];
  r.enabled   = jsonInt(body, "enabled", r.enabled) ? 1 : 0;
  r.mode      = (uint8_t)constrain(jsonInt(body, "mode", r.mode), 0, 2);
  int dm      = jsonInt(body, "daysMask", r.daysMask);
  r.daysMask  = (dm >= 0 && dm <= 127) ? (uint8_t)dm : 0x7F;
  r.startHour = (uint8_t)constrain(jsonInt(body, "startHour", r.startHour), 0, 23);
  r.startMin  = (uint8_t)constrain(jsonInt(body, "startMin",  r.startMin),  0, 59);
  r.endHour   = (uint8_t)constrain(jsonInt(body, "endHour",   r.endHour),   0, 23);
  r.endMin    = (uint8_t)constrain(jsonInt(body, "endMin",    r.endMin),    0, 59);
  r.action    = jsonInt(body, "action", r.action) ? 1 : 0;
  r.day       = (uint8_t)constrain(jsonInt(body, "day",   r.day),   1, 31);
  r.month     = (uint8_t)constrain(jsonInt(body, "month", r.month), 1, 12);
}

void handleRulesPost() {
  // ── Locate the JSON body ────────────────────────────────────
  // Normally it arrives as the "plain" argument: ESP8266WebServer parks
  // any non-form POST body there. But some Android WebViews send a
  // text/plain body while still labelling it x-www-form-urlencoded, and
  // then the whole JSON string turns up as an ARGUMENT NAME with an
  // empty value. Check for that too - otherwise the request falls
  // through to the legacy bulk branch below and wipes every rule.
  String body;
  if (server.hasArg("plain")) body = server.arg("plain");
  body.trim();

  if (!body.startsWith("{")) {
    for (int i = 0; i < server.args(); i++) {
      String n = server.argName(i); n.trim();
      String v = server.arg(i);     v.trim();
      if (n.startsWith("{")) { body = n; break; }
      if (v.startsWith("{")) { body = v; break; }
    }
  }

  // ── Single-slot JSON write / delete ─────────────────────────
  if (body.startsWith("{")) {
    int id = jsonInt(body, "id", -1);
    if (id < 0 || id >= MAX_RULES) {
      sendCORS();
      server.send(400, "application/json", "{\"ok\":false,\"err\":\"bad id\"}");
      return;
    }

    if (jsonInt(body, "delete", 0)) {
      defaultRule(rules[id]);
      Serial.print("Rule "); Serial.print(id); Serial.println(" DELETED via JSON");
    } else {
      applyRuleJson(id, body);
      Serial.print("Rule "); Serial.print(id); Serial.println(" updated via JSON");
    }

    saveRules();
    handleRulesGet();     // reply with the full, freshly stored set
    return;
  }

  // ── Legacy form-encoded bulk POST (all rules + voltage guard) ──
  for (int i = 0; i < MAX_RULES; i++) {
    String pre = "r" + String(i);
    if (!server.hasArg(pre + "en")) continue;   // slot not present in this body
    rules[i].enabled   = server.arg(pre + "en").toInt()   ? 1 : 0;
    rules[i].mode      = (uint8_t)constrain(server.arg(pre + "mode").toInt(), 0, 2);
    int dm             = server.arg(pre + "days").toInt();
    rules[i].daysMask  = (dm >= 0 && dm <= 127) ? (uint8_t)dm : 0x7F;
    rules[i].startHour = (uint8_t)constrain(server.arg(pre + "sh").toInt(), 0, 23);
    rules[i].startMin  = (uint8_t)constrain(server.arg(pre + "sm").toInt(), 0, 59);
    rules[i].endHour   = (uint8_t)constrain(server.arg(pre + "eh").toInt(), 0, 23);
    rules[i].endMin    = (uint8_t)constrain(server.arg(pre + "em").toInt(), 0, 59);
    rules[i].action    = server.arg(pre + "act").toInt() ? 1 : 0;
    rules[i].day       = (uint8_t)constrain(server.arg(pre + "day").toInt(), 1, 31);
    rules[i].month     = (uint8_t)constrain(server.arg(pre + "month").toInt(), 1, 12);
  }

  if (server.hasArg("voltGuard")) {
    voltGuardEnabled = server.arg("voltGuard").toInt() ? true : false;
    int vm = server.arg("voltMode").toInt();
    voltMode = (vm >= VOLT_MODE_OFF && vm <= VOLT_MODE_AC) ? vm : VOLT_MODE_OFF;

    float sc = server.arg("voltScale").toFloat();
    if (sc > 0) voltScale = sc;
    float vmn = server.arg("voltMin").toFloat();
    float vmx = server.arg("voltMax").toFloat();
    if (vmx > vmn) { voltMin = vmn; voltMax = vmx; }
  }

  saveRules();
  Serial.println("Rules saved (form bulk)");
  handleRulesGet();
}

// ── GET-based rule write / delete ─────────────────────────────
// These exist because a plain GET with a query string carries no request
// body and no content type, so it can never trigger a CORS preflight and
// never hits the flaky fetch()-with-a-body path in App Inventor's
// embedded WebViewer. The app POSTs JSON first and falls back to these
// the moment the POST does not come back - which is what made saving and
// deleting rules fail from the phone while loading them worked fine.
void handleRuleSetGet() {
  int id = server.hasArg("id") ? server.arg("id").toInt() : -1;
  if (id < 0 || id >= MAX_RULES) {
    sendCORS();
    server.send(400, "application/json", "{\"ok\":false,\"err\":\"bad id\"}");
    return;
  }

  Rule& r = rules[id];
  // Each field is optional, exactly like the JSON path.
  if (server.hasArg("en"))    r.enabled   = server.arg("en").toInt() ? 1 : 0;
  if (server.hasArg("mode"))  r.mode      = (uint8_t)constrain(server.arg("mode").toInt(), 0, 2);
  if (server.hasArg("days")) {
    int dm = server.arg("days").toInt();
    r.daysMask = (dm >= 0 && dm <= 127) ? (uint8_t)dm : 0x7F;
  }
  if (server.hasArg("sh"))    r.startHour = (uint8_t)constrain(server.arg("sh").toInt(), 0, 23);
  if (server.hasArg("sm"))    r.startMin  = (uint8_t)constrain(server.arg("sm").toInt(), 0, 59);
  if (server.hasArg("eh"))    r.endHour   = (uint8_t)constrain(server.arg("eh").toInt(), 0, 23);
  if (server.hasArg("em"))    r.endMin    = (uint8_t)constrain(server.arg("em").toInt(), 0, 59);
  if (server.hasArg("act"))   r.action    = server.arg("act").toInt() ? 1 : 0;
  if (server.hasArg("day"))   r.day       = (uint8_t)constrain(server.arg("day").toInt(), 1, 31);
  if (server.hasArg("month")) r.month     = (uint8_t)constrain(server.arg("month").toInt(), 1, 12);

  saveRules();
  Serial.print("Rule "); Serial.print(id); Serial.println(" updated via GET");
  handleRulesGet();
}

// Clears ONE slot back to empty and persists it.
void handleRuleDel() {
  int id = server.hasArg("id") ? server.arg("id").toInt() : -1;
  if (id < 0 || id >= MAX_RULES) {
    sendCORS();
    server.send(400, "application/json", "{\"ok\":false,\"err\":\"bad id\"}");
    return;
  }
  defaultRule(rules[id]);
  saveRules();
  Serial.print("Rule "); Serial.print(id); Serial.println(" DELETED");
  handleRulesGet();
}

// Clears every slot in one shot. Voltage-guard settings are untouched.
void handleRulesClearAll() {
  for (int i = 0; i < MAX_RULES; i++) defaultRule(rules[i]);
  saveRules();
  Serial.println("ALL rules cleared");
  handleRulesGet();
}

// Answers a CORS preflight instead of 404-ing it. The app is careful to
// send only "simple" requests that never preflight, but a future WebView
// or a desktop browser used for debugging may still send one, and a 404
// there silently cancels the real request.
void handleRulesOptions() {
  sendCORS();
  server.sendHeader("Access-Control-Max-Age", "600");
  server.send(204, "text/plain", "");
}

// Sets the device clock from a UTC epoch supplied by the phone. This is
// the safety net for a device whose NTP never comes through (no internet
// on that WiFi, UDP/123 blocked, a captive router). Without a clock the
// time rules simply never fire, and nothing on the device says why.
// SNTP keeps running afterwards and will correct this if it ever syncs.
void handleSetTime() {
  if (!server.hasArg("epoch")) {
    sendCORS();
    server.send(400, "application/json", "{\"ok\":false,\"err\":\"epoch\"}");
    return;
  }
  long epoch = server.arg("epoch").toInt();
  if (epoch < 1700000000L) {          // sanity: anything before ~Nov 2023 is junk
    sendCORS();
    server.send(400, "application/json", "{\"ok\":false,\"err\":\"range\"}");
    return;
  }
  setClockFromEpoch(epoch);
  publishRules();
  handleRulesGet();
}

// /settings is registered from BOTH setup() and enterPairingMode(), and a
// long-press from normal operation runs the second one on a server that
// already has it. Guarded for the same reason as the rule routes.
bool settingsRouteRegistered = false;
void registerSettingsRoute() {
  if (settingsRouteRegistered) return;
  settingsRouteRegistered = true;
  server.on("/settings", HTTP_GET, handleSettings);
}

// ── Rules over MQTT ────────────────────────────────────────────
// Lets an app that cannot reach the device over the LAN — the https-hosted
// iPhone web app, or any phone away from home — manage rules through the
// broker. One JSON command per message on <topic>/rules/set:
//
//   {"get":1}                         re-publish <topic>/rules now
//   {"id":N, ...fields}               write slot N (same fields as POST /rules)
//   {"id":N,"delete":1}               clear slot N
//   {"clear":1}                       clear every slot
//   {"epoch":<UTC seconds>}           set the device clock
//
// Every command answers by re-publishing <topic>/rules (retained), so the
// app always redraws from what the device actually stored.
//
// SECURITY: on a public broker, anyone who knows the base topic can do this
// — exactly as they already can toggle the pump. Use a private broker with
// credentials before handing devices to other people.
void handleRulesMqtt(const String& msg) {
  if (!msg.startsWith("{")) return;

  if (jsonInt(msg, "get", 0)) { publishRules(); return; }

  if (jsonInt(msg, "clear", 0)) {
    for (int i = 0; i < MAX_RULES; i++) defaultRule(rules[i]);
    Serial.println("ALL rules cleared via MQTT");
    saveRules();                       // saveRules() publishes
    return;
  }

  long epoch = jsonInt(msg, "epoch", 0);
  if (epoch) {
    if (epoch >= 1700000000L) setClockFromEpoch(epoch);
    publishRules();
    return;
  }

  int id = jsonInt(msg, "id", -1);
  if (id < 0 || id >= MAX_RULES) {
    Serial.println("Rules MQTT: bad or missing id - ignored");
    return;
  }
  if (jsonInt(msg, "delete", 0)) {
    defaultRule(rules[id]);
    Serial.print("Rule "); Serial.print(id); Serial.println(" DELETED via MQTT");
  } else {
    applyRuleJson(id, msg);
    Serial.print("Rule "); Serial.print(id); Serial.println(" updated via MQTT");
  }
  saveRules();                         // saveRules() publishes
}

// Registered exactly once per boot. setup() calls this during normal
// operation and enterPairingMode() calls it for a device that was never
// configured. The guard matters because a long-press from NORMAL
// operation enters pairing mode on a server that ALREADY has these
// routes - re-adding them would append duplicate handler entries
// (gotcha #4). The guard makes that case a no-op instead.
bool ruleRoutesRegistered = false;
void registerRuleRoutes() {
  if (ruleRoutesRegistered) return;
  ruleRoutesRegistered = true;
  server.on("/rules",      HTTP_GET,     handleRulesGet);
  server.on("/rules",      HTTP_POST,    handleRulesPost);
  server.on("/rules",      HTTP_OPTIONS, handleRulesOptions);
  server.on("/ruleset",    HTTP_GET,     handleRuleSetGet);
  server.on("/ruledel",    HTTP_GET,     handleRuleDel);
  server.on("/rulesclear", HTTP_GET,     handleRulesClearAll);
  server.on("/settime",    HTTP_GET,     handleSetTime);
  server.on("/telegram",     HTTP_GET,   handleTgGet);
  server.on("/telegramset",  HTTP_GET,   handleTgSet);
  server.on("/telegramtest", HTTP_GET,   handleTgTest);
  server.on("/level",      HTTP_GET,     handleLevelTxt);
}

// Plain-text fill level for the Android app's background watcher, which polls
// this over the local WiFi once a minute. The device decides whether the level
// is critical and says so in words, so the App Inventor blocks only ever have
// to compare text - no number parsing, nothing that can throw on a stray reply.
//   "LOW 12.4"   at or below CRITICAL_LOW_PCT
//   "OK 45.0"    healthy
//   "NA"         no valid reading yet
void handleLevelTxt() {
  char body[24];
  if (lastPct < 0)                          strcpy(body, "NA");
  else if (lastPct <= CRITICAL_LOW_PCT)     snprintf(body, sizeof(body), "LOW %.1f", lastPct);
  else                                      snprintf(body, sizeof(body), "OK %.1f", lastPct);
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "text/plain", body);
}

void handleSettings() {
  String ssid, pass;
  int depth;
  String mBroker, mTopic;
  int mPort;
  int afMin, afMax;
  bool afEnabled;
  bool valid = loadAll(ssid, pass, depth, mBroker, mPort, mTopic, afMin, afMax, afEnabled);
  String json = "{\"ssid\":\"" + (valid ? ssid : String("")) + "\","
                "\"depth\":" + String(depth) + ","
                "\"mqttBroker\":\"" + mBroker + "\","
                "\"mqttPort\":" + String(mPort) + ","
                "\"mqttTopic\":\"" + mTopic + "\","
                "\"autoFillMin\":" + String(afMin) + ","
                "\"autoFillMax\":" + String(afMax) + "}";
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", json);
}

void handleConnect() {
  String ssid   = server.arg("ssid");
  String pass   = server.arg("pass");
  int    depth  = server.arg("depth").toInt();
  if (depth < MIN_TANK_DEPTH_CM || depth > 1000) depth = DEFAULT_TANK_DEPTH_CM;

  String mBroker = server.arg("mqttBroker");
  if (!mBroker.length()) mBroker = MQTT_BROKER_DEFAULT;
  int mPort = server.arg("mqttPort").toInt();
  if (mPort <= 0 || mPort > 65535) mPort = MQTT_PORT_DEFAULT;
  String mTopic = server.arg("mqttTopic");
  if (!mTopic.length()) mTopic = MQTT_TOPIC_DEFAULT;

  int afMin = server.arg("autoFillMin").toInt();
  int afMax = server.arg("autoFillMax").toInt();
  if (!server.hasArg("autoFillMin") || afMin < 0 || afMin > 100) afMin = DEFAULT_AUTOFILL_MIN;
  if (!server.hasArg("autoFillMax") || afMax < 0 || afMax > 100) afMax = DEFAULT_AUTOFILL_MAX;
  if (afMin >= afMax) { afMin = DEFAULT_AUTOFILL_MIN; afMax = DEFAULT_AUTOFILL_MAX; }

  if (!ssid.length()) {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "application/json", "{\"ok\":false}");
    return;
  }

  showMessage("Testing WiFi...", ssid.c_str());
  WiFi.mode(WIFI_AP_STA);
  WiFi.begin(ssid.c_str(), pass.c_str());

  int t = 0;
  while (WiFi.status() != WL_CONNECTED && t < 20) { delay(500); t++; }

  if (WiFi.status() == WL_CONNECTED) {
    saveAll(ssid, pass, depth, mBroker, mPort, mTopic, afMin, afMax);
    tankDepthCm = depth;
    mqttBroker  = mBroker;
    mqttPort    = mPort;
    mqttTopic   = mTopic;
    autoFillMin = afMin;
    autoFillMax = afMax;
    updateDerivedTopics();
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "application/json", "{\"ok\":true}");
    char buf[48];
    snprintf(buf, sizeof(buf), "Depth: %d cm", depth);
    showMessage("Saved & Connected!", buf, "Restarting...");
    delay(2000);
    ESP.restart();
  } else {
    WiFi.disconnect();
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP(AP_SSID, AP_PASS);
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "application/json", "{\"ok\":false}");
    showMessage("WiFi failed!", "Check password", "Try again");
  }
}

void handleClear() {
  clearAll();
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "text/plain", "ok");
  showMessage("All settings", "cleared", "Restarting...");
  delay(2000);
  ESP.restart();
}

// ── Enter pairing mode ────────────────────────────────────────
void enterPairingMode() {
  pairingMode = true;
  Serial.println("Entering pairing mode");

  mqtt.disconnect();
  WiFi.disconnect();
  delay(200);

  // WIFI_AP_STA (not plain WIFI_AP) from the start: scanNetworksAsync()
  // needs the station radio active to scan. It's documented to enable
  // STA internally on demand, but that on-the-fly switch silently fails
  // on many ESP8266 cores once the AP is already actively broadcasting —
  // the scan never actually starts and scanComplete() stays stuck at
  // "not triggered" forever. Starting in AP_STA up front avoids that
  // failure mode entirely — this is the same approach WiFiManager uses.
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(AP_SSID, AP_PASS);
  delay(500); // let AP settle before starting the web server

  // No DNS catch-all and no OS captive-portal auto-detection routes.
  // The phone joins the AP silently — the app drives everything by
  // tapping "WiFi" and calling these endpoints directly.
  server.on("/",          HTTP_GET,  handleRoot);
  server.on("/scan",       HTTP_GET,  handleScan);
  server.on("/scanresult", HTTP_GET,  handleScanResult);
  registerSettingsRoute();
  registerRuleRoutes();
  server.on("/connect",   HTTP_POST, handleConnect);
  server.on("/clear",     HTTP_GET,  handleClear);
  // No onNotFound redirect — unmatched requests get a normal 404.

  server.begin();

  showPairingOLED();
  Serial.print("AP IP: "); Serial.println(WiFi.softAPIP());
  Serial.println("Web server active — waiting for app to connect");
}

// ── Button check ──────────────────────────────────────────────
// Call this ANYWHERE the code might otherwise block (WiFi connect,
// MQTT connect, normal loop). Returns true the instant a full 1s
// hold is detected — at which point pairing mode is ALREADY active
// (AP + web server running) and the caller must stop what it was
// doing and return immediately, without waiting on WiFi/MQTT at all.
bool pollPairingButton() {
  if (digitalRead(PAIR_BUTTON) != BUTTON_PRESSED) return false;

  unsigned long pressStart = millis();

  while (digitalRead(PAIR_BUTTON) == BUTTON_PRESSED) {
    unsigned long held = millis() - pressStart;

    // LED flash speed increases as user holds
    int flashRate = (held > 500) ? 80 : 300;
    if (millis() - lastLedToggle > (unsigned long)flashRate) {
      ledState = !ledState;
      digitalWrite(LED_INTERNAL, ledState ? LOW : HIGH);
      lastLedToggle = millis();
    }

    // Progress bar on OLED
    int progress = (int)(held * 128 / PAIR_HOLD_MS);
    if (progress > 128) progress = 128;
    display.clearBuffer();
    display.setFont(u8g2_font_ncenB08_tr);
    display.setCursor(0, 10); display.print("Hold for pairing...");
    display.drawFrame(0, 22, 128, 12);
    display.drawBox(0, 22, progress, 12);
    display.setCursor(0, 48); display.print("Release: toggle pump");
    char pct[8];
    snprintf(pct, sizeof(pct), "%d%%", (int)(held * 100 / PAIR_HOLD_MS));
    display.setCursor(0, 62); display.print(pct);
    display.sendBuffer();

    if (held >= PAIR_HOLD_MS) {
      digitalWrite(LED_INTERNAL, LOW);
      // Abandon whatever WiFi/MQTT connection attempt was in progress —
      // no waiting for MQTT, no waiting for WiFi timeout. Reset now.
      mqtt.disconnect();
      WiFi.disconnect(true);
      applyRelay(false);   // safety: never leave the pump running while in pairing mode
      clearAll();
      Serial.println("Settings cleared by button — skipping WiFi/MQTT, entering pairing now");
      showMessage("All settings", "cleared!", "Entering pairing...");
      delay(600);
      enterPairingMode();
      return true;
    }
    delay(30);
  }

  // Released before the pairing threshold — a short click, or noise.
  digitalWrite(LED_INTERNAL, HIGH);
  unsigned long heldMs = millis() - pressStart;
  if (heldMs < SHORT_CLICK_MIN_MS) {
    Serial.println("Button: press too short, ignored (debounce)");
    return false;
  }

  // One short click has happened. Wait up to DOUBLE_CLICK_MS for a second
  // press before deciding what it was. The buzzer is serviced inside this
  // wait so the alarm keeps beeping normally while the user is clicking.
  unsigned long waitStart = millis();
  bool secondClick = false;
  while (millis() - waitStart < DOUBLE_CLICK_MS) {
    serviceBuzzer();
    if (digitalRead(PAIR_BUTTON) == BUTTON_PRESSED) {
      // Second press detected — debounce it, then wait for its release so
      // the next pollPairingButton() call doesn't see it as a third click.
      delay(30);
      unsigned long secondStart = millis();
      while (digitalRead(PAIR_BUTTON) == BUTTON_PRESSED) {
        serviceBuzzer();
        // A long hold on the second press still means pairing mode.
        if (millis() - secondStart >= PAIR_HOLD_MS) {
          digitalWrite(LED_INTERNAL, LOW);
          mqtt.disconnect();
          WiFi.disconnect(true);
          applyRelay(false);
          setBuzzer(false);
          clearAll();
          Serial.println("Settings cleared by button (2nd press held) — entering pairing");
          showMessage("All settings", "cleared!", "Entering pairing...");
          delay(600);
          enterPairingMode();
          return true;
        }
        delay(10);
      }
      secondClick = true;
      break;
    }
    delay(10);
  }

  if (secondClick) {
    Serial.println("Button: DOUBLE click -> mute alarm");
    if (alarmActive && !alarmMuted) {
      muteAlarm("button");
      showMessage("Alarm muted", "(double click)");
      delay(800);
    } else {
      showMessage("No active alarm", "to mute");
      delay(800);
    }
  } else {
    Serial.print("Button: SINGLE click ("); Serial.print(heldMs); Serial.println("ms)");
    toggleRelayButton();
  }
  return false;
}

// ── Setup ─────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(100);

  pinMode(TRIG1,        OUTPUT);
  pinMode(ECHO1,        INPUT);
  pinMode(RELAY1,       OUTPUT);
  pinMode(LED_INTERNAL, OUTPUT);
  pinMode(PAIR_BUTTON,  INPUT_PULLUP);
  pinMode(BUZZER,       OUTPUT);

  digitalWrite(RELAY1,      LOW);    // active-HIGH: relay OFF at boot (and during pairing)
  digitalWrite(TRIG1,       LOW);
  digitalWrite(LED_INTERNAL,HIGH);
  digitalWrite(BUZZER,      BUZZER_OFF);   // never buzz at boot or in pairing mode

  Wire.begin();
  display.begin();
  showMessage("AFIF-TECH", "Water Tank Monitor", "Booting...");
  delay(1200);

  // Load all settings from EEPROM (MQTT + auto-fill values are always
  // populated, falling back to compiled-in defaults if never saved)
  String ssid, pass;
  if (!loadAll(ssid, pass, tankDepthCm, mqttBroker, mqttPort, mqttTopic,
               autoFillMin, autoFillMax, autoFillEnabled)) {
    Serial.println("No settings saved — entering pairing mode");
    showMessage("No settings saved", "Starting pairing", "mode...");
    delay(800);
    enterPairingMode();
    return;
  }

  updateDerivedTopics();

  Serial.print("Tank depth: ");  Serial.print(tankDepthCm); Serial.println(" cm");
  Serial.print("SSID: ");        Serial.println(ssid);
  Serial.print("MQTT broker: "); Serial.print(mqttBroker);
  Serial.print(":");             Serial.println(mqttPort);
  Serial.print("MQTT topic: ");  Serial.println(mqttTopic);
  Serial.print("Auto-fill: ");   Serial.print(autoFillEnabled ? "ON " : "OFF ");
  Serial.print(autoFillMin);     Serial.print("% - "); Serial.print(autoFillMax); Serial.println("%");

  if (!connectWiFi(ssid, pass)) {
    if (!pairingMode) {
      // Only show this if it was a REAL WiFi timeout — if the button
      // already put us into pairing mode, that screen is already showing.
      showMessage("WiFi failed!", "Hold D3 for 1s", "to enter pairing");
    }
    return;
  }

  finishOnlineSetup();
  connectMQTT();
}

// Everything that can only be done once WiFi is up: the clock, the stored
// rules, mDNS, the web server and OTA.
//
// THIS USED TO LIVE INLINE IN setup(), AFTER AN EARLY `return`.
// If WiFi did not come up within setup()'s 15 s window - the classic case
// being a power cut where the router boots slower than the ESP8266 - setup()
// returned right here and NONE of this ever ran. loop() would then reconnect
// WiFi and MQTT perfectly happily, so the device looked entirely healthy:
// live level readings, working auto-fill, app connected. But loadRules() had
// never been called, so rules[] was still all zeros - every rule disabled -
// and configTime() had never been called either, so there was no clock.
// Rules silently did nothing until the next reboot. loop() now calls this as
// soon as WiFi is available, so a slow router just delays the rule engine
// instead of disabling it.
void finishOnlineSetup() {
  if (onlineSetupDone) return;
  onlineSetupDone = true;

  // Wall-clock time for the schedule rules. Jordan is UTC+3 all year
  // (DST was abolished in 2022), so a fixed offset is correct here.
  configTime(3 * 3600, 0, "pool.ntp.org", "time.nist.gov");
  Serial.print("Waiting for NTP");
  for (int i = 0; i < 20 && time(nullptr) < 100000; i++) {
    delay(250); Serial.print(".");
  }
  timeReady = (time(nullptr) > 100000);
  Serial.println(timeReady ? " OK" : " not yet (loop() keeps trying)");

  loadRules();
  loadPushCfg();
  loadOtaFromRtc();          // an internet update accepted just before a restart

  // OTA firmware updates — reachable at http://watertank.local/update
  // (or http://<device-ip>/update) while connected to home WiFi.
  if (MDNS.begin(OTA_HOSTNAME)) {
    MDNS.addService("http", "tcp", 80);
    Serial.print("OTA ready at http://"); Serial.print(OTA_HOSTNAME); Serial.println(".local/update");
  }
  // Rules are edited from the app while both are on home WiFi, so these
  // must be registered in normal operation too, not just in pairing mode.
  registerRuleRoutes();
  registerSettingsRoute();

  httpUpdater.setup(&server, "/update", OTA_USER, OTA_PASSWORD);
  // Update is the global singleton ESP8266HTTPUpdateServer writes to
  // internally — registering callbacks here still fires during an OTA
  // upload even though httpUpdater.setup() handles the request itself.
  Update.onProgress(showOtaProgress);
  Update.onEnd([]() {
    showMessage("Update complete!", "Restarting...");
  });
  server.begin();

  mqtt.setServer(mqttBroker.c_str(), mqttPort);
  mqtt.setKeepAlive(60);
  mqtt.setCallback(mqttCallback);
  // PubSubClient defaults to a 256-byte buffer for BOTH directions. The
  // retained rules payload is ~900 bytes and would be dropped silently,
  // so raise it. Needs PubSubClient 2.8 or newer.
  mqtt.setBufferSize(1280);
}

// ── Loop ──────────────────────────────────────────────────────
// ── Firmware update over the INTERNET ─────────────────────────
// The local /update page only works on the same WiFi. This lets the app
// update the device from anywhere:
//
//   app -> <topic>/ota/set : {"url":"https://.../fw.bin","md5":"<32 hex>",
//                             "size":<bytes>,"sig":"<64 hex>"}
//   device -> <topic>/ota  : {"state":"accepted|downloading|done|error",
//                             "pct":0-100,"msg":"...","version":"x.y.z"}
//
// Safety (from 2.2.0): the firmware is SIGNED. A public.key file next to
// this sketch makes the ESP8266 core refuse any image that is not signed
// with the matching private key (kept secret in GitHub). This applies to
// every update path: internet, the local /update page, anything. So even
// on a public broker, a stranger can at most make the tank install your
// own official firmware. A "version" older than or equal to FW_VERSION is
// refused unless "force":1, so it cannot be pushed back by accident.
// ("md5" and "sig" are optional; the app still sends them for devices
// running 2.1.0, which checked a password-based signature instead.)
//
// The MQTT callback only verifies and queues; the (long) download happens
// here, from loop().
bool   otaPending = false;
String otaUrl, otaMd5, otaVersion;
int    otaSize = 0;
unsigned long otaRebootAt = 0;     // restart scheduled to download with clean memory
unsigned long otaWaitUntil = 0;    // after that restart: how long to wait for WiFi + MQTT

// An accepted update is kept in RTC memory across a software restart, so the
// download starts right after boot, when the heap is clean. (After hours of
// running, the heap gets fragmented: the long GitHub redirect URL and the
// 16 kB TLS buffer may then not fit, which showed up as "HTTP 302" and
// "connection failed" in the field.) RTC memory survives ESP.restart() but
// not a power loss.
struct OtaRtc {
  uint32_t magic;
  int32_t  size;
  char     version[16];
  char     url[432];
};                                  // 456 bytes, fits the 512-byte RTC user area
#define OTA_RTC_MAGIC 0xAF1F7A2Bu

void saveOtaToRtc() {
  OtaRtc r;
  memset(&r, 0, sizeof(r));
  r.magic = OTA_RTC_MAGIC;
  r.size = otaSize;
  strlcpy(r.version, otaVersion.c_str(), sizeof(r.version));
  strlcpy(r.url, otaUrl.c_str(), sizeof(r.url));
  ESP.rtcUserMemoryWrite(0, (uint32_t*)&r, sizeof(r));
}

// Called once in setup(): picks up an update saved before the restart.
void loadOtaFromRtc() {
  OtaRtc r;
  if (!ESP.rtcUserMemoryRead(0, (uint32_t*)&r, sizeof(r))) return;
  if (r.magic != OTA_RTC_MAGIC) return;
  uint32_t zero = 0;
  ESP.rtcUserMemoryWrite(0, &zero, sizeof(zero));      // use once: never loop on a bad image
  r.url[sizeof(r.url) - 1] = 0; r.version[sizeof(r.version) - 1] = 0;
  otaUrl = String(r.url); otaSize = r.size; otaVersion = String(r.version); otaMd5 = "";
  otaPending = true;
  otaWaitUntil = millis() + 90000UL;
  Serial.print("Internet OTA resumed after restart: "); Serial.println(otaUrl);
}

String heapInfo() {
  return " (heap " + String(ESP.getFreeHeap()) + ", largest block " + String(ESP.getMaxFreeBlockSize()) + ")";
}

// Reads a string field from a small JSON object (handles \/ and \" escapes).
static String jsonStr(const String& body, const char* field) {
  String key = String("\"") + field + "\":";
  int i = body.indexOf(key);
  if (i < 0) return "";
  i += key.length();
  while (i < (int)body.length() && body[i] == ' ') i++;
  if (i >= (int)body.length() || body[i] != '"') return "";
  i++;
  String out;
  while (i < (int)body.length()) {
    char c = body[i++];
    if (c == '\\' && i < (int)body.length()) { out += body[i++]; continue; }
    if (c == '"') break;
    out += c;
  }
  return out;
}

String sha256Hex(const String& in) {
  br_sha256_context ctx;
  br_sha256_init(&ctx);
  br_sha256_update(&ctx, in.c_str(), in.length());
  uint8_t out[32];
  br_sha256_out(&ctx, out);
  char hex[65];
  for (int i = 0; i < 32; i++) snprintf(hex + 2 * i, 3, "%02x", out[i]);
  return String(hex);
}

// Compares "2.10.1" style versions numerically: <0, 0, >0.
int compareVersions(const String& a, const String& b) {
  int ia = 0, ib = 0;
  for (int part = 0; part < 4; part++) {
    long va = 0, vb = 0;
    while (ia < (int)a.length() && !isDigit(a[ia])) ia++;
    while (ia < (int)a.length() && isDigit(a[ia])) va = va * 10 + (a[ia++] - '0');
    while (ib < (int)b.length() && !isDigit(b[ib])) ib++;
    while (ib < (int)b.length() && isDigit(b[ib])) vb = vb * 10 + (b[ib++] - '0');
    if (va != vb) return va < vb ? -1 : 1;
  }
  return 0;
}

void publishOta(const char* state, int pct, const String& msg) {
  if (!mqtt.connected()) return;
  String j = String("{\"state\":\"") + state + "\",\"pct\":" + String(pct) +
             ",\"msg\":\"" + jsonEscape(msg.c_str()) + "\",\"version\":\"" FW_VERSION "\"}";
  mqtt.publish(otaStatusTopic.c_str(), j.c_str(), false);
}

void handleOtaCommand(const String& msg) {
  String url = jsonStr(msg, "url");
  String md5 = jsonStr(msg, "md5");  md5.toLowerCase();
  String ver = jsonStr(msg, "version");
  int size   = jsonInt(msg, "size", 0);
  bool force = jsonInt(msg, "force", 0) == 1;
  if (!url.startsWith("http://") && !url.startsWith("https://")) { publishOta("error", 0, "invalid link"); return; }
  if (size <= 0) { publishOta("error", 0, "missing file size"); return; }
  if (ver.length() && !force && compareVersions(ver, FW_VERSION) <= 0) {
    publishOta("error", 0, "already running " FW_VERSION);
    return;
  }
  if (otaPending) return;
  if ((uint32_t)size > ESP.getFreeSketchSpace()) { publishOta("error", 0, "file too big for this device"); return; }
  if ((int)url.length() >= 432) { publishOta("error", 0, "link too long"); return; }
  otaUrl = url; otaMd5 = md5; otaSize = size; otaVersion = ver;
  // Restart first, then download with clean memory (see OtaRtc above).
  saveOtaToRtc();
  otaRebootAt = millis() + 1500;
  Serial.print("Internet OTA accepted: "); Serial.println(url);
  publishOta("accepted", 0, "restarting to prepare the update");
}

void serviceRemoteOta() {
  if (otaRebootAt && (long)(millis() - otaRebootAt) >= 0) {
    Serial.println("Internet OTA: restarting to download with clean memory");
    mqtt.loop(); delay(100);
    ESP.restart();
  }
  if (!otaPending) return;
  // Right after the restart: wait (up to 90 s) for WiFi and the broker, so the
  // app can see the progress.
  if (WiFi.status() != WL_CONNECTED || !mqtt.connected()) {
    if (otaWaitUntil && (long)(millis() - otaWaitUntil) < 0) return;
    if (WiFi.status() != WL_CONNECTED) { otaPending = false; publishOta("error", 0, "no WiFi"); return; }
  }
  otaPending = false;
  Serial.print("Internet OTA starting"); Serial.println(heapInfo());

  bool wasBuzzing = buzzerState;
  if (wasBuzzing) setBuzzer(false);
  publishOta("downloading", 0, "");
  showMessage("Internet update", "Downloading...");

  WiFiClient plain;
  WiFiClientSecure secure;
  bool https = otaUrl.startsWith("https://");
  if (https) {
    secure.setInsecure();                 // integrity comes from the MD5 + signature
    secure.setBufferSizes(16384, 512);    // big RX record for GitHub etc., small TX
  }
  HTTPClient http;
  http.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);   // GitHub release links redirect
  http.setTimeout(15000);
  http.setUserAgent("AFIF-TECH-WaterTank/" FW_VERSION);

  String err;
  bool begun = https ? http.begin(secure, otaUrl) : http.begin(plain, otaUrl);
  if (!begun) {
    err = "bad link";
  } else {
    int code = http.GET();
    if (code != HTTP_CODE_OK) {
      err = "download failed: HTTP " + String(code);
      if (code < 0) err += " " + HTTPClient::errorToString(code);
    } else {
      int len = http.getSize();
      if (len <= 0)                                   err = "server did not send the file size";
      else if (len != otaSize)                        err = "file size changed (" + String(len) + " bytes)";
      else if ((uint32_t)len > ESP.getFreeSketchSpace()) err = "file too big for this device";
      else if (!Update.begin(len))                    err = "cannot start update: " + Update.getErrorString();
      else {
        // No MD5 needed: the core verifies the RSA signature of the whole
        // image in Update.end() and rejects anything unsigned or altered.
        WiFiClient* st = http.getStreamPtr();
        uint8_t buf[1024];
        int done = 0, lastPct = -1;
        unsigned long lastData = millis();
        while (done < len) {
          size_t avail = st->available();
          if (avail) {
            int n = st->readBytes(buf, avail < sizeof(buf) ? avail : sizeof(buf));
            if (n > 0) {
              if (Update.write(buf, n) != (size_t)n) { err = "flash write failed: " + Update.getErrorString(); break; }
              done += n;
              lastData = millis();
              int pct = (int)(((long long)done * 100) / len);
              if (pct != lastPct) {
                showOtaProgress(done, len);
                if (pct / 10 != lastPct / 10) { publishOta("downloading", pct, ""); mqtt.loop(); }
                lastPct = pct;
              }
            }
          } else if (!st->connected()) {
            err = "connection closed after " + String(done) + " bytes"; break;
          } else if (millis() - lastData > 20000) {
            err = "download stalled"; break;
          } else {
            delay(1);
          }
          yield();
        }
        if (!err.length()) {
          if (!Update.end()) err = "rejected: " + Update.getErrorString() + " (not signed by AFIF-TECH?)";
        } else {
          Update.end();   // discard the partial image; the old firmware stays
        }
      }
    }
    http.end();
  }

  if (err.length()) {
    Serial.print("Internet OTA failed: "); Serial.println(err);
    publishOta("error", 0, err + heapInfo());
    showMessage("Update failed", "Old firmware kept");
    delay(1500);
    if (alarmActive && !alarmMuted && wasBuzzing) setBuzzer(true);
    return;
  }
  Serial.println("Internet OTA OK - restarting");
  publishOta("done", 100, "installed - restarting");
  showMessage("Update OK!", "Restarting...");
  for (int i = 0; i < 20; i++) { mqtt.loop(); delay(50); }   // let the last message go out
  ESP.restart();
}

void loop() {
  if (pairingMode) {
    if (buzzerState) setBuzzer(false);   // never buzz while in pairing mode
    server.handleClient();          // no DNS server — app connects directly
    if (millis() - lastLedToggle > 300) {
      ledState = !ledState;
      digitalWrite(LED_INTERNAL, ledState ? LOW : HIGH);
      lastLedToggle = millis();
    }
    return;
  }

  serviceBuzzer();   // non-blocking beep pattern while the alarm is active

  if (pollPairingButton()) return;  // pairing mode is now active — bail out this cycle

  server.handleClient();   // serves /update (OTA) during normal operation
  MDNS.update();

  if (WiFi.status() != WL_CONNECTED) {
    showMessage("WiFi lost...", "Reconnecting");
    String ssid, pass;
    int depth, afMin, afMax;
    String mBroker, mTopic;
    int mPort;
    bool afEnabled;
    if (loadAll(ssid, pass, depth, mBroker, mPort, mTopic, afMin, afMax, afEnabled)) {
      tankDepthCm = depth;
      connectWiFi(ssid, pass);
    }
    if (pairingMode) return;  // button was held during reconnect — pairing took over
  }

  // If setup() bailed out before finishing (WiFi was not up yet), do the
  // rest of the init now that it is. No-op once it has run.
  if (WiFi.status() == WL_CONNECTED && !onlineSetupDone) finishOnlineSetup();

  // NTP often takes longer than the few seconds finishOnlineSetup() waits,
  // and the SDK's SNTP client keeps polling in the background long after
  // that. This used to be latched ONCE at boot and never looked at again,
  // so a slow first sync disabled every time rule until the next reboot -
  // even though the clock itself became correct moments later. Re-check it
  // each pass instead.
  if (!timeReady && time(nullptr) > 100000) {
    timeReady = true;
    Serial.println("NTP clock acquired — time rules are now active");
  }

  if (!mqtt.connected()) connectMQTT();
  if (pairingMode) return;    // button was held during MQTT connect — pairing took over
  mqtt.loop();                // also delivers any pending mqttCallback() invocations

  servicePush();              // at most one queued notification per pass
  serviceRemoteOta();         // internet firmware update, if one was requested

  if (millis() - lastSensorRead >= SENSOR_INTERVAL) {
    lastSensorRead = millis();

    long  dist   = readDistance();
    float pct    = distanceToPercent(dist);
    bool  mqttOk = false;

    // Publish the reading to the global BEFORE any rule is evaluated, so
    // level rules (mode 2) test the value from this very cycle. -1 on a
    // sensor error makes them stand down rather than act on stale data.
    lastPct = pct;

    // ── Relay decision, highest priority first ────────────────
    // 1. Voltage guard  - safety interlock, forces OFF and blocks
    //                     every other source from turning the pump on.
    // 2. Schedule rule  - while inside an active window, the rule owns
    //                     the relay and re-asserts its state each cycle.
    // 3. Auto-fill      - normal hysteresis (needs a valid level).
    // 4. Manual         - if none of the above apply, whatever the last
    //                     manual command set simply persists.
    //
    // THIS WHOLE BLOCK USED TO SIT INSIDE `else` OF `if (pct < 0)`, so a
    // single sensor error skipped rule evaluation entirely. A full tank
    // reads closer than the JSN-SR04T's 20 cm floor and therefore reports
    // an error, which meant a timed rule could not fire exactly when the
    // tank was full - and never fired at all on a flaky sensor. Time rules
    // have nothing to do with the water level, so they now run either way;
    // only auto-fill and the alarm still require a good reading.
    lastVoltage = readVoltage();
    voltFault = false;

    if (voltGuardEnabled && voltMode != VOLT_MODE_OFF && lastVoltage >= 0) {
      if (lastVoltage < voltMin || lastVoltage > voltMax) voltFault = true;
    }

    raiseVoltageEvent();

    if (lastVoltage >= 0) {
      char vbuf[12];
      snprintf(vbuf, sizeof(vbuf), "%.1f", lastVoltage);
      mqtt.publish(voltageTopic.c_str(), vbuf, true);
    }

    int ruleIdx = activeRuleIndex();

    if (voltFault) {
      // Out-of-range supply: cut the pump and leave it cut.
      if (relayOn) applyRelay(false, "voltage guard");
      fillActive = false;
      Serial.print("VOLTAGE FAULT ("); Serial.print(lastVoltage, 1);
      Serial.println("V) - pump forced OFF");
    } else if (ruleIdx >= 0) {
      bool want = rules[ruleIdx].action ? true : false;
      if (want != relayOn) {
        char why[16];
        snprintf(why, sizeof(why), "rule %d", ruleIdx + 1);
        applyRelay(want, why);
        Serial.print("RULE "); Serial.print(ruleIdx + 1);
        Serial.println(want ? " -> pump ON" : " -> pump OFF");
      }
      fillActive = want;   // keep auto-fill's latch in step with the rule
    } else {
      // No rule owns the relay right now.
      if (lastRuleIdx >= 0 && !autoFillEnabled) {
        // A window just ended and nothing else controls the pump. Release
        // it instead of leaving it latched on for ever - previously a
        // "Pump ON" window with Auto Fill off never switched back off.
        if (relayOn) applyRelay(false, "rule ended");
        fillActive = false;
        Serial.print("RULE "); Serial.print(lastRuleIdx + 1);
        Serial.println(" window ended -> pump released");
      }
      if (autoFillEnabled && pct >= 0) {
        if (pct <= autoFillMin)      fillActive = true;
        else if (pct >= autoFillMax) fillActive = false;
        if (fillActive != relayOn) applyRelay(fillActive, "auto-fill");
      }
    }
    lastRuleIdx = ruleIdx;

    // Re-publish the rules payload when what it reports changes on its own
    // (a rule starting or ending, the clock arriving), so the web app's
    // "Running now" and clock banner stay current without polling.
    {
      static int  pubRuleIdx   = -2;
      static bool pubTimeReady = false;
      static unsigned long pubAt = 0;
      // ...and once a minute regardless, so the "Device time" the app shows
      // (HH:MM) never goes stale.
      if (ruleIdx != pubRuleIdx || timeReady != pubTimeReady ||
          millis() - pubAt >= 60000UL) {
        pubRuleIdx   = ruleIdx;
        pubTimeReady = timeReady;
        pubAt        = millis();
        publishRules();
      }
    }

    // One line per cycle saying exactly what the rule engine sees. If a
    // rule is not firing, this says whether the clock is good and which
    // rule (if any) matched.
    {
      time_t nowT = time(nullptr);
      struct tm* ltNow = localtime(&nowT);
      Serial.print("clock:");
      if (timeReady && ltNow) {
        char tb[24];
        snprintf(tb, sizeof(tb), "%04d-%02d-%02d %02d:%02d wday%d",
                 ltNow->tm_year + 1900, ltNow->tm_mon + 1, ltNow->tm_mday,
                 ltNow->tm_hour, ltNow->tm_min, ltNow->tm_wday);
        Serial.print(tb);
      } else {
        Serial.print("NOT SET");
      }
      Serial.print(" activeRule:");
      if (ruleIdx >= 0) Serial.print(ruleIdx + 1); else Serial.print("none");
      Serial.print(" pump:"); Serial.println(relayOn ? "ON" : "OFF");
    }

    if (pct < 0) {
      raiseSensorEvent(true);
      Serial.println("Sensor error - level control paused, time rules still running");
    } else {
      raiseSensorEvent(false);
      // Buzzer alarm — starts/clears itself around ALARM_LEVEL_PCT.
      // serviceBuzzer() in loop() does the actual beeping.
      evaluateAlarm(pct);

      char payload[10];
      snprintf(payload, sizeof(payload), "%.1f", pct);
      mqttOk = mqtt.publish(mqttTopic.c_str(), payload);

      // Critical-low push notification — fires once per "episode" (not
      // every 2s while it stays low), and re-arms once the level rises
      // back out of the critical band so it can fire again next time.
      raiseLevelEvents(pct);

      Serial.print("Dist:");    Serial.print(dist);
      Serial.print("cm Fill:"); Serial.print(payload);
      Serial.print("% D:");     Serial.print(tankDepthCm);
      Serial.print("cm Pump:"); Serial.print(relayOn ? "ON" : "OFF");
      Serial.print(autoFillEnabled ? " (AUTO)" : " (MANUAL)");
      Serial.print(" MQTT:");   Serial.println(mqttOk ? "OK" : "FAIL");
    }

    updateDisplay(dist, pct, relayOn, mqttOk);
  }
}
