// ============================================================
//  Smart Greenhouse Controller - ESP32
//  Sensors : DHT22 (temp/hum), Soil moisture, LDR (light)
//  Outputs : Fan LED, Pump LED, Grow LED, Status LED
//  Interface: I2C LCD 16x2, 2 buttons, WiFi web dashboard
//  Storage  : NVS (thresholds), SD card (CSV log)
// ============================================================

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WebServer.h>
#include <SD.h>
#include <SPI.h>

// ── Pin Definitions ─────────────────────────────────────────
#define DHTPIN      15    // DHT22 data pin
#define DHTTYPE     DHT22
#define SOIL_PIN    35    // Soil moisture analog input
#define LDR_PIN     34    // Light sensor analog input

#define FAN_LED      2    // Red LED  → fan active
#define GROW_LED     4    // Blue LED → grow light active
#define PUMP_LED     5    // Yellow LED → pump active
#define STATUS_LED  19    // Green LED → all systems normal

#define MODE_BUTTON 18    // Cycles AUTO → MANUAL → EDIT; long-press saves in EDIT
#define EDIT_BUTTON 13    // In EDIT mode: single=increment, double=decrement, hold=fast increment

#define LCD_SDA     21
#define LCD_SCL     22
#define LCD_ADDR    0x27  // Default I2C address for most 16x2 LCD backpacks

#define SD_CS       25    // SD card chip-select
#define SD_SCK      32
#define SD_MISO     23
#define SD_MOSI     33
#define SD_LOG_FILE "/greenhouse_log.csv"

// ── Light / Night-mode Thresholds ───────────────────────────
// LDR raw value: higher = darker (inverse relationship)
const int NIGHT_ON_THRESHOLD  = 2200; // Enter NIGHT mode when raw > this
const int NIGHT_OFF_THRESHOLD = 1500; // Leave NIGHT mode when raw < this
// Hysteresis gap prevents rapid switching at boundary

// ── Timing Constants (milliseconds) ─────────────────────────
const unsigned long PRINT_INTERVAL  = 5000;  // Serial debug print every 5 s
const unsigned long SCREEN_INTERVAL = 3000;  // LCD screen rotation every 3 s
const unsigned long DEBOUNCE_DELAY  = 220;   // Ignore repeated button edges within 220 ms
const unsigned long DHT_INTERVAL    = 2000;  // DHT22 minimum read interval

// ── Control Hysteresis ───────────────────────────────────────
// Prevents rapid on/off cycling at threshold boundaries
const float TEMP_HYSTERESIS = 0.5f; // ±0.5 °C dead-band for fan
const int   SOIL_HYSTERESIS = 2;    // ±2 % dead-band for pump

// ── Data Log (circular buffer in RAM) ───────────────────────
#define LOG_SIZE        1440          // 1440 minutes = 24 hours of 1-min samples
#define LOG_INTERVAL_MS 60000UL      // Log one entry every 60 seconds

// ── Button Timing ────────────────────────────────────────────
const unsigned long DOUBLE_CLICK_TIME = 220; // Max gap between clicks for double-click
const unsigned long HOLD_TIME         = 600; // Hold duration to trigger hold action
const unsigned long HOLD_REPEAT       = 180; // Repeat interval while held
const unsigned long MODE_MSG_TIME     = 2000; // How long mode-change message shows on LCD

// ── Alert Thresholds ─────────────────────────────────────────
const float TEMP_DANGER_HIGH = 38.0f; // °C — critical high temperature alert
const int   SOIL_DANGER_LOW  = 10;    // % — critical low soil moisture alert

// ── Hardware Object Instances ────────────────────────────────
LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2);
DHT dht(DHTPIN, DHTTYPE);
Preferences prefs; // NVS (non-volatile storage) for threshold persistence

// ── WiFi & Web Server ────────────────────────────────────────
const char* WIFI_SSID     = "BIC";
const char* WIFI_PASSWORD = "11223344";

WebServer server(80);
bool wifiConnected = false;
bool sdReady       = false;

// ── Timing Trackers ──────────────────────────────────────────
unsigned long lastPrintTime  = 0;
unsigned long lastLogTime    = 0;
unsigned long lastScreenTime = 0;
unsigned long lastModePress  = 0;
unsigned long lastDHTTime    = 0;

// ── LCD State ────────────────────────────────────────────────
int  screenIndex = 0;    // Which of the 5 LCD screens is currently shown
bool lcdDirty    = true; // Flag: LCD content needs to be redrawn

// ── Circular Log Buffers ─────────────────────────────────────
float tempLog[LOG_SIZE];
float humLog[LOG_SIZE];
int   soilLog[LOG_SIZE];
int   lightLog[LOG_SIZE];
int   modeLog[LOG_SIZE];
int   fanLog[LOG_SIZE];
int   pumpLog[LOG_SIZE];
int   growLog[LOG_SIZE];

int   logIndex      = 0;     // Next write position in the circular buffer
bool  logBufferFull = false; // True once the buffer has wrapped around once

// ── Live Sensor Values ───────────────────────────────────────
float humidity    = 0.0f;
float temperature = 0.0f;
bool  dhtReady    = false; // True after the first successful DHT read
bool  dhtError    = false; // True if the most recent DHT read failed

int soilPct  = 0; // Soil moisture mapped to 0–100 %
int lightRaw = 0; // Raw ADC value from LDR (0–4095)

// ── 24-Hour Min/Max ──────────────────────────────────────────
float minTemp24 = 0.0f;
float maxTemp24 = 0.0f;
float minHum24  = 0.0f;
float maxHum24  = 0.0f;

// ── User-Configurable Thresholds (persisted in NVS) ─────────
float minTempEdit   = 18.0f; // Lower bound for acceptable temperature
float maxTempEdit   = 30.0f; // Upper bound for acceptable temperature
float minHumEdit    = 40.0f; // Lower bound for acceptable humidity
float maxHumEdit    = 80.0f; // Upper bound for acceptable humidity
float fanThreshold  = 30.0f; // Fan turns ON above this temperature
int   pumpThreshold = 30;    // Pump turns ON below this soil moisture %

// ── EDIT Mode State ──────────────────────────────────────────
int editItem = 0; // Index of the threshold currently being edited

// Labels shown on LCD during EDIT mode (matches order of editItem index)
const char* editLabels[] = {
  "MinTemp",
  "MaxTemp",
  "MinHum",
  "MaxHum",
  "FanThr",
  "PumpThr"
};
const int EDIT_ITEM_COUNT = 6;

// ── Actuator States ──────────────────────────────────────────
bool fanState   = false; // Is the fan currently ON?
bool pumpState  = false; // Is the pump currently ON?
bool growState  = false; // Is the grow light currently ON?
bool statusSafe = false; // Are all conditions within safe ranges?

// ── Alert Flags ──────────────────────────────────────────────
bool tempDangerAlert = false; // Temperature at or above TEMP_DANGER_HIGH
bool soilDangerAlert = false; // Soil moisture at or below SOIL_DANGER_LOW
bool humLowAlert     = false; // Humidity below minHumEdit
bool humHighAlert    = false; // Humidity above maxHumEdit

// ── Button State Tracking ────────────────────────────────────
bool lastModeButtonState = HIGH; // Previous physical state of MODE button
bool lastEditButtonState = HIGH; // Previous physical state of EDIT button

// EDIT button gesture state machine variables
bool editPressed          = false;
bool holdActive           = false;
bool singleClickPending   = false;
unsigned long editPressStart    = 0;
unsigned long lastHoldRepeat    = 0;
unsigned long pendingSingleTime = 0;

// MODE button long-press state machine variables
bool modePressed     = false;
bool modeHoldHandled = false;
unsigned long modePressStart = 0;

// ── LCD Mode-Change Message ──────────────────────────────────
char lastModeMsg[17] = "Mode:AUTO";       // Message shown briefly after mode change
unsigned long modeMsgUntil = 0;           // Timestamp when the message should stop showing

// ── Operating Modes ──────────────────────────────────────────
enum Mode {
  AUTO_MODE,   // Normal automatic control based on sensor readings
  MANUAL_MODE, // Fan/pump disabled; grow light still responds to light level
  EDIT_MODE,   // User is editing threshold values
  NIGHT_MODE   // Automatically entered when ambient light is very low
};

Mode userSelectedMode           = AUTO_MODE; // The mode the user has chosen
Mode currentMode                = AUTO_MODE; // The mode currently active (may be overridden by NIGHT)
Mode previousUserModeBeforeEdit = AUTO_MODE; // Saved mode to restore after leaving EDIT

// ── Day/Night Duration Tracking ─────────────────────────────
unsigned long modeStartTime     = 0; // millis() when the current mode started
unsigned long lastDayDuration   = 0; // Duration of the most recently completed day period
unsigned long lastNightDuration = 0; // Duration of the most recently completed night period

// ── Function Prototypes ──────────────────────────────────────
void saveThresholds();
void loadThresholds();
void factoryReset();
void handleRoot();
void drawLCD();
void updateMode(int rawLight, unsigned long currentTime);
void updateStatusLED();
void incrementEditItem();
void decrementEditItem();
void initSD();
void logToSD();
void computeMinMax24();
void recordModeChange(Mode oldMode, Mode newMode, unsigned long durationMs);
void updateGrowState(int rawLight);
const char* modeToText(Mode mode);
void formatDuration(unsigned long ms, char* buffer);
float getEditValue(int item);
void handleModeButton();
void handleEditButton();
void evaluateAlerts();
void applyActuatorLogic();
void writeOutputs();

// ============================================================
//  HELPER UTILITIES
// ============================================================

// Returns a short human-readable string for any Mode value
const char* modeToText(Mode mode) {
  switch (mode) {
    case AUTO_MODE:   return "AUTO";
    case MANUAL_MODE: return "MANUAL";
    case EDIT_MODE:   return "EDIT";
    case NIGHT_MODE:  return "NIGHT";
    default:          return "UNK";
  }
}

// Converts a millisecond duration into "HH:MM" format
// e.g. 3 661 000 ms → "01:01"
void formatDuration(unsigned long ms, char* buffer) {
  unsigned long totalMins = ms / 60000UL;
  sprintf(buffer, "%02lu:%02lu", totalMins / 60UL, totalMins % 60UL);
}

// Returns the current float value of the threshold at editItem index
float getEditValue(int item) {
  switch (item) {
    case 0: return minTempEdit;
    case 1: return maxTempEdit;
    case 2: return minHumEdit;
    case 3: return maxHumEdit;
    case 4: return fanThreshold;
    case 5: return (float)pumpThreshold;
    default: return 0.0f;
  }
}

// ============================================================
//  EDIT MODE — VALUE ADJUSTMENT
// ============================================================

// Decrements the currently selected threshold by one step.
// Wraps around when reaching the minimum, and enforces
// that minTemp < maxTemp and minHum < maxHum at all times.
void decrementEditItem() {
  switch (editItem) {
    case 0: // Min temperature (step: 1 °C, range: 0–39)
      minTempEdit -= 1.0f;
      if (minTempEdit < 0.0f)          minTempEdit = 39.0f;   // wrap-around
      if (minTempEdit >= maxTempEdit)  minTempEdit = maxTempEdit - 1.0f; // keep gap
      break;
    case 1: // Max temperature (step: 1 °C, range: minTemp+1–40)
      maxTempEdit -= 1.0f;
      if (maxTempEdit <= minTempEdit)  maxTempEdit = 40.0f;   // wrap-around
      if (maxTempEdit > 40.0f)         maxTempEdit = 40.0f;
      break;
    case 2: // Min humidity (step: 5 %, range: 0–95)
      minHumEdit -= 5.0f;
      if (minHumEdit < 0.0f)           minHumEdit = 95.0f;
      if (minHumEdit >= maxHumEdit)    minHumEdit = maxHumEdit - 5.0f;
      break;
    case 3: // Max humidity (step: 5 %, range: minHum+5–100)
      maxHumEdit -= 5.0f;
      if (maxHumEdit <= minHumEdit)    maxHumEdit = 100.0f;
      if (maxHumEdit > 100.0f)         maxHumEdit = 100.0f;
      break;
    case 4: // Fan threshold temperature (step: 1 °C, range: 20–40)
      fanThreshold -= 1.0f;
      if (fanThreshold < 20.0f)        fanThreshold = 40.0f;
      break;
    case 5: // Pump threshold soil % (step: 5 %, range: 10–100)
      pumpThreshold -= 5;
      if (pumpThreshold < 10)          pumpThreshold = 100;
      break;
  }
}

// Increments the currently selected threshold by one step.
// Wraps around at maximums, enforces min < max constraint.
void incrementEditItem() {
  switch (editItem) {
    case 0: // Min temperature
      minTempEdit += 1.0f;
      if (minTempEdit >= maxTempEdit)  minTempEdit = maxTempEdit - 1.0f;
      if (minTempEdit < 0.0f)          minTempEdit = 0.0f;
      if (minTempEdit > 39.0f)         minTempEdit = 0.0f; // wrap-around
      break;
    case 1: // Max temperature
      maxTempEdit += 1.0f;
      if (maxTempEdit > 40.0f)         maxTempEdit = minTempEdit + 1.0f; // wrap-around
      if (maxTempEdit <= minTempEdit)  maxTempEdit = minTempEdit + 1.0f;
      break;
    case 2: // Min humidity
      minHumEdit += 5.0f;
      if (minHumEdit >= maxHumEdit)    minHumEdit = maxHumEdit - 5.0f;
      if (minHumEdit < 0.0f)           minHumEdit = 0.0f;
      if (minHumEdit > 95.0f)          minHumEdit = 0.0f;
      break;
    case 3: // Max humidity
      maxHumEdit += 5.0f;
      if (maxHumEdit > 100.0f)         maxHumEdit = minHumEdit + 5.0f;
      if (maxHumEdit <= minHumEdit)    maxHumEdit = minHumEdit + 5.0f;
      break;
    case 4: // Fan threshold
      fanThreshold += 1.0f;
      if (fanThreshold > 40.0f)        fanThreshold = 20.0f;
      break;
    case 5: // Pump threshold
      pumpThreshold += 5;
      if (pumpThreshold > 100)         pumpThreshold = 10;
      break;
  }
}

// ============================================================
//  BUTTON HANDLERS
// ============================================================

// Handles the MODE button:
//   Short press (normal)  → cycle AUTO → MANUAL → EDIT
//   Short press (in EDIT) → advance to next threshold item
//   Long press  (in EDIT) → save thresholds and exit EDIT mode
void handleModeButton() {
  bool currentState = digitalRead(MODE_BUTTON);
  unsigned long now = millis();

  // ── Detect press edge (HIGH → LOW, active-low with INPUT_PULLUP) ──
  if (lastModeButtonState == HIGH && currentState == LOW) {
    if (now - lastModePress > DEBOUNCE_DELAY) { // ignore bounce
      lastModePress    = now;
      modePressed      = true;
      modeHoldHandled  = false;
      modePressStart   = now;
    }
  }

  // ── Long-press detection while button is held ──
  if (modePressed && currentState == LOW && !modeHoldHandled) {
    if ((now - modePressStart >= HOLD_TIME) && currentMode == EDIT_MODE) {
      // Save thresholds to NVS and exit EDIT mode
      saveThresholds();

      Mode oldMode     = currentMode;
      userSelectedMode = previousUserModeBeforeEdit; // restore pre-edit mode
      currentMode      = userSelectedMode;
      recordModeChange(oldMode, currentMode, now - modeStartTime);
      modeStartTime    = now;

      modeHoldHandled = true;
      modePressed     = false;
      lastScreenTime  = now;
      screenIndex     = 0;
      lcdDirty        = true;
    }
  }

  // ── Release edge (LOW → HIGH) — execute short-press action ──
  if (lastModeButtonState == LOW && currentState == HIGH) {
    if (modePressed && !modeHoldHandled) {

      if (currentMode == EDIT_MODE) {
        // In EDIT mode: advance to next threshold item
        editItem = (editItem + 1) % EDIT_ITEM_COUNT;
      } else {
        // Cycle: AUTO → MANUAL → EDIT → AUTO
        if      (userSelectedMode == AUTO_MODE) {
          userSelectedMode = MANUAL_MODE;
        } else if (userSelectedMode == MANUAL_MODE) {
          previousUserModeBeforeEdit = MANUAL_MODE;
          userSelectedMode = EDIT_MODE;
        } else {
          previousUserModeBeforeEdit = AUTO_MODE;
          userSelectedMode = AUTO_MODE;
        }

        // Apply mode change unless NIGHT_MODE is currently overriding
        if (currentMode != NIGHT_MODE) {
          Mode oldMode = currentMode;

          // Remember pre-edit mode so we can restore it on exit
          if (userSelectedMode == EDIT_MODE && oldMode != EDIT_MODE) {
            previousUserModeBeforeEdit = oldMode;
          }

          currentMode = userSelectedMode;
          recordModeChange(oldMode, currentMode, now - modeStartTime);
          modeStartTime = now;
        }

        lastScreenTime = now;
        if (currentMode != EDIT_MODE) {
          screenIndex = 0; // reset LCD rotation when leaving EDIT
        }
      }

      lcdDirty = true;
    }

    modePressed     = false;
    modeHoldHandled = false;
  }

  lastModeButtonState = currentState;
}

// Handles the EDIT button (only active while in EDIT mode):
//   Single click → increment current threshold
//   Double click → decrement current threshold
//   Hold         → fast-increment (repeats every HOLD_REPEAT ms)
void handleEditButton() {
  bool currentState = digitalRead(EDIT_BUTTON);
  unsigned long now = millis();

  // Do nothing if not in EDIT mode; also reset all state
  if (currentMode != EDIT_MODE) {
    lastEditButtonState = currentState;
    editPressed         = false;
    holdActive          = false;
    singleClickPending  = false;
    return;
  }

  // ── Press edge ──
  if (lastEditButtonState == HIGH && currentState == LOW) {
    editPressed    = true;
    holdActive     = false;
    editPressStart = now;
    lastHoldRepeat = now;
  }

  // ── While held: check for hold threshold, then repeat ──
  if (editPressed && currentState == LOW) {
    if (!holdActive && (now - editPressStart >= HOLD_TIME)) {
      // First hold trigger
      holdActive         = true;
      singleClickPending = false;
      incrementEditItem();
      saveThresholds();
      lcdDirty       = true;
      lastHoldRepeat = now;
    } else if (holdActive && (now - lastHoldRepeat >= HOLD_REPEAT)) {
      // Subsequent repeated increments while held
      incrementEditItem();
      saveThresholds();
      lcdDirty       = true;
      lastHoldRepeat = now;
    }
  }

  // ── Release edge ──
  if (lastEditButtonState == LOW && currentState == HIGH) {
    if (editPressed) {
      editPressed = false;

      if (!holdActive) {
        // Was a short tap — decide single vs double click
        if (singleClickPending && (now - pendingSingleTime <= DOUBLE_CLICK_TIME)) {
          // Second tap within window → double click → decrement
          decrementEditItem();
          saveThresholds();
          lcdDirty           = true;
          singleClickPending = false;
        } else {
          // First tap — wait to see if a second tap arrives
          singleClickPending  = true;
          pendingSingleTime   = now;
        }
      } else {
        // Hold just finished — clear state, don't treat as click
        holdActive         = false;
        singleClickPending = false;
      }
    }
  }

  // ── Pending single click timeout: window passed, treat as increment ──
  if (singleClickPending && (now - pendingSingleTime > DOUBLE_CLICK_TIME)) {
    incrementEditItem();
    saveThresholds();
    lcdDirty           = true;
    singleClickPending = false;
  }

  lastEditButtonState = currentState;
}

// ============================================================
//  MODE & LIGHT MANAGEMENT
// ============================================================

// Logs a mode transition to Serial and SD card.
// Also triggers the LCD mode-change banner message.
void recordModeChange(Mode oldMode, Mode newMode, unsigned long durationMs) {
  char durBuf[6];
  formatDuration(durationMs, durBuf);

  Serial.print("MODE CHANGE: ");
  Serial.print(modeToText(oldMode));
  Serial.print(" -> ");
  Serial.print(modeToText(newMode));
  Serial.print(" | Prev duration: ");
  Serial.println(durBuf);

  // Prepare the brief LCD banner message
  snprintf(lastModeMsg, sizeof(lastModeMsg), "Mode:%s", modeToText(newMode));
  modeMsgUntil = millis() + MODE_MSG_TIME;

  // Append mode event line to SD log if available
  if (sdReady) {
    File f = SD.open(SD_LOG_FILE, FILE_APPEND);
    if (f) {
      f.print("MODE_EVENT,");
      f.print(modeToText(oldMode));
      f.print("->");
      f.print(modeToText(newMode));
      f.print(",duration=");
      f.println(durBuf);
      f.close();
    }
  }
}

// Checks ambient light level and switches to/from NIGHT_MODE.
// Uses hysteresis (two different thresholds) to prevent flickering.
// NIGHT_MODE takes priority over user-selected mode automatically.
void updateMode(int rawLight, unsigned long currentTime) {
  Mode newMode = currentMode;

  // Higher raw ADC = darker environment
  if (rawLight > NIGHT_ON_THRESHOLD) {
    newMode = NIGHT_MODE;                // Dark enough → force NIGHT
  } else if (rawLight < NIGHT_OFF_THRESHOLD) {
    newMode = userSelectedMode;          // Bright enough → restore user mode
  }
  // Between the two thresholds → keep current mode (hysteresis zone)

  if (newMode != currentMode) {
    unsigned long duration = currentTime - modeStartTime;

    // Record how long the outgoing mode lasted
    if (currentMode == NIGHT_MODE) lastNightDuration = duration;
    else                           lastDayDuration   = duration;

    recordModeChange(currentMode, newMode, duration);

    currentMode   = newMode;
    modeStartTime = currentTime;
    lcdDirty      = true;

    // Reset LCD rotation when changing modes (except in EDIT)
    if (currentMode != EDIT_MODE) {
      lastScreenTime = currentTime;
      screenIndex    = 0;
    }
  }
}

// Controls the grow light based on ambient light level.
// Grow light ON when it's dark (high raw ADC), OFF when bright.
// Uses the same night thresholds with hysteresis.
void updateGrowState(int rawLight) {
  if (growState) {
    if (rawLight < NIGHT_OFF_THRESHOLD) growState = false; // Bright → turn off grow light
  } else {
    if (rawLight > NIGHT_ON_THRESHOLD)  growState = true;  // Dark → turn on grow light
  }
}

// ============================================================
//  ALERT EVALUATION & ACTUATOR LOGIC
// ============================================================

// Compares live sensor readings against thresholds and
// sets boolean alert flags used by display and web dashboard.
void evaluateAlerts() {
  tempDangerAlert = (temperature >= TEMP_DANGER_HIGH); // Critical high temp
  soilDangerAlert = (soilPct <= SOIL_DANGER_LOW);      // Critical dry soil
  humLowAlert     = (humidity < minHumEdit);           // Humidity below target range
  humHighAlert    = (humidity > maxHumEdit);           // Humidity above target range
}

// Decides fan and pump state based on current mode and sensor values.
// MANUAL mode disables automatic fan/pump control.
// Hysteresis prevents rapid on/off switching at threshold boundaries.
void applyActuatorLogic() {
  if (currentMode == MANUAL_MODE) {
    // Manual mode: operator has disabled automatic actuation
    fanState  = false;
    pumpState = false;
  } else {
    // Auto / Night / Edit modes: apply hysteresis-based control

    // Fan: ON if temp exceeds threshold + hysteresis; OFF if below threshold - hysteresis
    if (temperature > (fanThreshold + TEMP_HYSTERESIS))      fanState = true;
    else if (temperature < (fanThreshold - TEMP_HYSTERESIS)) fanState = false;

    // Pump: ON if soil moisture drops below threshold - hysteresis; OFF once sufficiently wet
    if (soilPct < (pumpThreshold - SOIL_HYSTERESIS))         pumpState = true;
    else if (soilPct > (pumpThreshold + SOIL_HYSTERESIS))    pumpState = false;
  }

  // Grow light always responds to ambient light regardless of mode
  updateGrowState(lightRaw);
}

// Updates the STATUS LED:
// GREEN (HIGH) = all actuators off and no danger alerts → everything is safe
// OFF  (LOW)   = any actuator active or any alert present → attention needed
void updateStatusLED() {
  bool humidityWarning = humLowAlert || humHighAlert;
  bool anyWarning      = fanState || pumpState || growState
                         || tempDangerAlert || soilDangerAlert || humidityWarning;

  statusSafe = !anyWarning;
  digitalWrite(STATUS_LED, statusSafe ? HIGH : LOW);
}

// Writes all actuator states to their output pins,
// then updates the status LED to reflect the new state.
void writeOutputs() {
  digitalWrite(FAN_LED,  fanState  ? HIGH : LOW);
  digitalWrite(PUMP_LED, pumpState ? HIGH : LOW);
  digitalWrite(GROW_LED, growState ? HIGH : LOW);
  updateStatusLED();
}

// ============================================================
//  DATA LOGGING
// ============================================================

// Scans the circular log buffer to find the 24-hour
// min/max values for temperature and humidity.
void computeMinMax24() {
  int count = logBufferFull ? LOG_SIZE : logIndex;

  // Edge case: no data yet — use current readings
  if (count <= 0) {
    minTemp24 = temperature;
    maxTemp24 = temperature;
    minHum24  = humidity;
    maxHum24  = humidity;
    return;
  }

  // Initialise with the first logged value, then scan the rest
  minTemp24 = tempLog[0];
  maxTemp24 = tempLog[0];
  minHum24  = humLog[0];
  maxHum24  = humLog[0];

  for (int i = 1; i < count; i++) {
    if (tempLog[i] < minTemp24) minTemp24 = tempLog[i];
    if (tempLog[i] > maxTemp24) maxTemp24 = tempLog[i];
    if (humLog[i]  < minHum24)  minHum24  = humLog[i];
    if (humLog[i]  > maxHum24)  maxHum24  = humLog[i];
  }
}

// Initialises the SD card and creates the CSV log file
// with a header row if it does not already exist.
void initSD() {
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);

  if (!SD.begin(SD_CS)) {
    sdReady = false; // SD card not present or failed to mount
    return;
  }

  sdReady = true;

  // Create file with header only on first boot
  if (!SD.exists(SD_LOG_FILE)) {
    File f = SD.open(SD_LOG_FILE, FILE_WRITE);
    if (f) {
      f.println("Entry,Temp_C,Humidity_%,Soil_%,Light_raw,Mode,Fan,Pump,Grow");
      f.close();
    }
  }
}

// Appends one CSV data row to the SD log file.
// Called once per LOG_INTERVAL_MS from the main loop.
void logToSD() {
  if (!sdReady) return;

  File f = SD.open(SD_LOG_FILE, FILE_APPEND);
  if (!f) return;

  f.print(logIndex);                 f.print(",");
  f.print(temperature, 1);           f.print(",");
  f.print((int)humidity);            f.print(",");
  f.print(soilPct);                  f.print(",");
  f.print(lightRaw);                 f.print(",");
  f.print(modeToText(currentMode));  f.print(",");
  f.print(fanState  ? "ON" : "OFF"); f.print(",");
  f.print(pumpState ? "ON" : "OFF"); f.print(",");
  f.println(growState ? "ON" : "OFF");
  f.close();
}

// ============================================================
//  THRESHOLD PERSISTENCE (NVS)
// ============================================================

// Saves all user-configurable thresholds to ESP32 NVS flash.
// Called automatically whenever a value is changed in EDIT mode.
void saveThresholds() {
  prefs.begin("gh", false); // open "gh" namespace in read-write mode
  prefs.putFloat("minT", minTempEdit);
  prefs.putFloat("maxT", maxTempEdit);
  prefs.putFloat("minH", minHumEdit);
  prefs.putFloat("maxH", maxHumEdit);
  prefs.putFloat("fanT", fanThreshold);
  prefs.putInt("pmpT", pumpThreshold);
  prefs.end();
}

// Loads thresholds from NVS on boot.
// Default values are used if no data has been saved yet.
void loadThresholds() {
  prefs.begin("gh", true); // open "gh" namespace in read-only mode
  minTempEdit   = prefs.getFloat("minT", 18.0f);
  maxTempEdit   = prefs.getFloat("maxT", 30.0f);
  minHumEdit    = prefs.getFloat("minH", 40.0f);
  maxHumEdit    = prefs.getFloat("maxH", 80.0f);
  fanThreshold  = prefs.getFloat("fanT", 30.0f);
  pumpThreshold = prefs.getInt("pmpT", 30);
  prefs.end();
}

// Clears all saved thresholds from NVS and resets to factory defaults.
// Triggered by sending 'R' or 'r' over the Serial monitor.
void factoryReset() {
  prefs.begin("gh", false);
  prefs.clear(); // wipe entire "gh" NVS namespace
  prefs.end();

  // Restore in-memory values to defaults
  minTempEdit   = 18.0f;
  maxTempEdit   = 30.0f;
  minHumEdit    = 40.0f;
  maxHumEdit    = 80.0f;
  fanThreshold  = 30.0f;
  pumpThreshold = 30;

  lcdDirty = true;
}

// ============================================================
//  LCD DISPLAY
// ============================================================

// Draws the appropriate screen to the 16x2 LCD.
// Called whenever lcdDirty is true (sensor update, mode change, etc.)
//
// Screen rotation (non-EDIT modes, every SCREEN_INTERVAL ms):
//   0 → Temperature + Humidity / Soil + Light
//   1 → 24h Min/Max Temp and Humidity
//   2 → Current mode + Fan/Pump state
//   3 → Last day / night duration
//   4 → Alert status summary
void drawLCD() {
  lcd.clear();

  // ── Temporary mode-change banner ──
  if (modeMsgUntil > millis()) {
    lcd.setCursor(0, 0);
    lcd.print(lastModeMsg);
    lcd.setCursor(0, 1);
    lcd.print("Mode changed");
    return;
  }

  // ── EDIT MODE: show threshold label and current value ──
  if (currentMode == EDIT_MODE) {
    lcd.setCursor(0, 0);
    lcd.print(editLabels[editItem]);   // e.g. "FanThr"
    lcd.setCursor(10, 0);
    lcd.print(editItem + 1);           // e.g. "5"
    lcd.print("/");
    lcd.print(EDIT_ITEM_COUNT);        // e.g. "/6"

    lcd.setCursor(0, 1);
    lcd.print("Val:");
    float val = getEditValue(editItem);

    // Humidity and pump items display as integer percent
    if (editItem == 5 || editItem == 2 || editItem == 3) {
      lcd.print((int)val);
      lcd.print("%");
    } else {
      lcd.print(val, 1);
      lcd.print("C");
    }
  }
  // ── Screen 0: Live sensor readings ──
  else if (screenIndex == 0) {
    lcd.setCursor(0, 0);
    if (dhtError) {
      lcd.print("DHT Read Error");
    } else {
      lcd.print("T:");
      lcd.print(temperature, 1);
      lcd.print(" H:");
      lcd.print((int)humidity);
      lcd.print("%");
    }
    lcd.setCursor(0, 1);
    lcd.print("S:");
    lcd.print(soilPct);
    lcd.print("% L:");
    lcd.print(lightRaw);
  }
  // ── Screen 1: 24-hour min/max ──
  else if (screenIndex == 1) {
    lcd.setCursor(0, 0);
    lcd.print("Tn:");
    lcd.print(minTemp24, 1);
    lcd.print(" Tx:");
    lcd.print(maxTemp24, 1);

    lcd.setCursor(0, 1);
    lcd.print("Hn:");
    lcd.print((int)minHum24);
    lcd.print("% Hx:");
    lcd.print((int)maxHum24);
    lcd.print("%");
  }
  // ── Screen 2: Mode and actuator states ──
  else if (screenIndex == 2) {
    lcd.setCursor(0, 0);
    lcd.print("Mode:");
    lcd.print(modeToText(currentMode));

    lcd.setCursor(0, 1);
    lcd.print("F:");
    lcd.print(fanState ? "ON " : "OFF");
    lcd.print(" P:");
    lcd.print(pumpState ? "ON" : "OFF");
  }
  // ── Screen 3: Day/Night duration history ──
  else if (screenIndex == 3) {
    char dayBuf[6], nightBuf[6];
    formatDuration(lastDayDuration, dayBuf);
    formatDuration(lastNightDuration, nightBuf);

    lcd.setCursor(0, 0);
    lcd.print("Day:  ");
    lcd.print(dayBuf);

    lcd.setCursor(0, 1);
    lcd.print("Night:");
    lcd.print(nightBuf);
  }
  // ── Screen 4: Alert summary ──
  else {
    lcd.setCursor(0, 0);
    if (tempDangerAlert)      lcd.print("ALERT:TEMP HIGH ");
    else if (humHighAlert)    lcd.print("ALERT:HUM HIGH  ");
    else if (humLowAlert)     lcd.print("ALERT:HUM LOW   ");
    else                      lcd.print("Temp/Hum OK     ");

    lcd.setCursor(0, 1);
    if (soilDangerAlert)      lcd.print("ALERT:SOIL LOW  ");
    else if (!statusSafe)     lcd.print("Check env range ");
    else                      lcd.print("Soil OK         ");
  }
}

// ============================================================
//  WEB DASHBOARD (handleRoot)
// ============================================================

// Serves the live HTML dashboard at the root URL "/".
// The page auto-refreshes every 5 seconds.
// All sensor values, actuator states, alerts, thresholds,
// and system info are embedded directly into the HTML response.
void handleRoot() {
  char dayBuf[6], nightBuf[6];
  formatDuration(lastDayDuration, dayBuf);
  formatDuration(lastNightDuration, nightBuf);

  // Pre-compute bar chart fill percentages (0–100) for each sensor
  int tempBarPct  = (int)constrain(map((long)(temperature * 10), 0, 500, 0, 100), 0, 100);
  int humBarPct   = (int)constrain(humidity, 0, 100);
  int soilBarPct  = (int)constrain(soilPct, 0, 100);
  int lightBarPct = (int)constrain(map(lightRaw, 0, 4095, 0, 100), 0, 100);

  // ── HTML + CSS ───────────────────────────────────────────
  String html = F("<!DOCTYPE html><html lang='en'><head>"
    "<meta charset='UTF-8'>"
    "<meta http-equiv='refresh' content='5'>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>Greenhouse OS</title>"
    "<link rel='preconnect' href='https://fonts.googleapis.com'>"
    "<link href='https://fonts.googleapis.com/css2?family=Space+Mono:wght@400;700&family=Playfair+Display:wght@700&display=swap' rel='stylesheet'>"
    "<style>"
    // CSS custom properties (design tokens)
    ":root{"
      "--bg:#0b0f0b;"
      "--surface:#111711;"
      "--surface2:#172017;"
      "--border:#1e2e1e;"
      "--green:#5aff8f;"
      "--green-dim:#2a7a48;"
      "--amber:#ffb830;"
      "--amber-dim:#7a5010;"
      "--red:#ff4d4d;"
      "--blue:#5ab4ff;"
      "--text:#c8dcc8;"
      "--text-dim:#5a7a5a;"
      "--mono:'Space Mono',monospace;"
      "--serif:'Playfair Display',serif;"
    "}"
    "*{box-sizing:border-box;margin:0;padding:0;}"
    "body{background:var(--bg);color:var(--text);font-family:var(--mono);font-size:13px;min-height:100vh;padding:0;}"
    // CRT scanline overlay for industrial aesthetic
    "body::before{content:'';position:fixed;inset:0;background:repeating-linear-gradient(0deg,transparent,transparent 2px,rgba(0,0,0,.07) 2px,rgba(0,0,0,.07) 4px);pointer-events:none;z-index:9999;}"
    // Header bar
    ".hdr{display:flex;align-items:center;justify-content:space-between;padding:18px 28px 14px;"
      "border-bottom:1px solid var(--border);background:var(--surface);}"
    ".hdr-left{display:flex;align-items:baseline;gap:14px;}"
    ".logo{font-family:var(--serif);font-size:22px;color:var(--green);letter-spacing:.5px;}"
    ".sub{font-size:10px;color:var(--text-dim);letter-spacing:2px;text-transform:uppercase;}"
    // Mode indicator pill — color changes per mode
    ".mode-pill{padding:4px 14px;border-radius:2px;font-size:11px;font-weight:700;letter-spacing:2px;text-transform:uppercase;border:1px solid;}"
    ".mode-AUTO{color:var(--green);border-color:var(--green-dim);background:rgba(90,255,143,.07);}"
    ".mode-MANUAL{color:var(--amber);border-color:var(--amber-dim);background:rgba(255,184,48,.07);}"
    ".mode-NIGHT{color:var(--blue);border-color:#1a3a5a;background:rgba(90,180,255,.06);}"
    ".mode-EDIT{color:var(--red);border-color:#5a1010;background:rgba(255,77,77,.06);}"
    // Pulsing dot to indicate live auto-refresh
    ".refresh-dot{width:7px;height:7px;border-radius:50%;background:var(--green);display:inline-block;animation:pulse 2s infinite;margin-right:6px;}"
    "@keyframes pulse{0%,100%{opacity:1}50%{opacity:.2}}"
    // Responsive 3-column grid
    ".main{padding:20px 28px;display:grid;grid-template-columns:1fr 1fr 1fr;gap:16px;max-width:1100px;margin:0 auto;}"
    "@media(max-width:800px){.main{grid-template-columns:1fr 1fr;}}"
    "@media(max-width:500px){.main{grid-template-columns:1fr;padding:12px;}}"
    // Card base styles
    ".card{background:var(--surface);border:1px solid var(--border);border-radius:3px;padding:18px 18px 14px;}"
    ".card-full{grid-column:1/-1;}"
    ".card-title{font-size:9px;letter-spacing:3px;text-transform:uppercase;color:var(--text-dim);"
      "margin-bottom:16px;padding-bottom:8px;border-bottom:1px solid var(--border);"
      "display:flex;align-items:center;gap:8px;}"
    ".card-title::before{content:'';width:3px;height:3px;border-radius:50%;background:var(--green);display:inline-block;}"
    // Sensor stat layout
    ".big-stat{display:flex;flex-direction:column;gap:16px;}"
    ".stat-row{display:flex;flex-direction:column;gap:6px;}"
    ".stat-label{font-size:10px;color:var(--text-dim);letter-spacing:1px;text-transform:uppercase;display:flex;justify-content:space-between;}"
    // Thin progress bars for sensor visualisation
    ".bar-track{height:3px;background:var(--border);border-radius:2px;overflow:hidden;}"
    ".bar-fill{height:100%;border-radius:2px;transition:width .4s ease;}"
    ".bar-green{background:var(--green);}"
    ".bar-amber{background:var(--amber);}"
    ".bar-blue{background:var(--blue);}"
    ".bar-red{background:var(--red);}"
    // Large hero number (temperature / humidity)
    ".hero{text-align:center;padding:10px 0;}"
    ".hero-num{font-family:var(--serif);font-size:56px;line-height:1;letter-spacing:-2px;}"
    ".hero-unit{font-size:14px;color:var(--text-dim);margin-top:4px;}"
    // 2×2 actuator grid
    ".act-grid{display:grid;grid-template-columns:1fr 1fr;gap:10px;}"
    ".act-item{background:var(--surface2);border:1px solid var(--border);border-radius:2px;padding:12px 10px;display:flex;flex-direction:column;gap:6px;}"
    ".act-name{font-size:9px;letter-spacing:2px;text-transform:uppercase;color:var(--text-dim);}"
    ".act-state{font-size:14px;font-weight:700;}"
    ".fan-on{color:var(--red);}"
    ".pump-on{color:var(--amber);}"
    ".grow-on{color:var(--blue);}"
    ".safe-on{color:var(--green);}"
    ".off{color:#2a3a2a;}"
    // Alert list rows with left-border color coding
    ".alert-list{display:flex;flex-direction:column;gap:8px;}"
    ".alert-row{display:flex;justify-content:space-between;align-items:center;"
      "padding:8px 10px;background:var(--surface2);border-radius:2px;border-left:3px solid transparent;}"
    ".alert-row.warn{border-left-color:var(--red);}"
    ".alert-row.ok{border-left-color:var(--green-dim);}"
    ".alert-label{font-size:11px;color:var(--text-dim);}"
    ".alert-badge{font-size:10px;font-weight:700;letter-spacing:1px;padding:2px 8px;border-radius:1px;}"
    ".badge-danger{background:rgba(255,77,77,.15);color:var(--red);}"
    ".badge-ok{background:rgba(90,255,143,.08);color:var(--green);}"
    ".badge-warn{background:rgba(255,184,48,.12);color:var(--amber);}"
    // Threshold display grid
    ".thr-grid{display:grid;grid-template-columns:1fr 1fr;gap:8px;}"
    ".thr-item{background:var(--surface2);padding:10px;border-radius:2px;}"
    ".thr-key{font-size:9px;color:var(--text-dim);letter-spacing:1px;text-transform:uppercase;margin-bottom:4px;}"
    ".thr-val{font-size:13px;font-weight:700;color:var(--amber);}"
    // History table rows
    ".hist-row{display:flex;justify-content:space-between;align-items:center;padding:7px 0;border-bottom:1px solid var(--border);}"
    ".hist-row:last-child{border-bottom:none;}"
    ".hist-label{font-size:11px;color:var(--text-dim);}"
    ".hist-val{font-size:12px;font-weight:700;}"
    // System status dots
    ".sys-row{display:flex;flex-wrap:wrap;gap:16px;padding-top:4px;}"
    ".sys-item{display:flex;align-items:center;gap:6px;font-size:11px;color:var(--text-dim);}"
    ".sys-dot{width:6px;height:6px;border-radius:50%;flex-shrink:0;}"
    ".dot-ok{background:var(--green);}"
    ".dot-err{background:var(--red);}"
    ".dot-warn{background:var(--amber);}"
    ".footer{text-align:center;padding:14px;font-size:10px;color:var(--text-dim);letter-spacing:2px;border-top:1px solid var(--border);}"
    "</style></head><body>");

  // ── HEADER ───────────────────────────────────────────────
  String modeCls = "mode-" + String(modeToText(currentMode));
  html += "<div class='hdr'>";
  html += "<div class='hdr-left'><span class='logo'>&#127807; Greenhouse OS</span><span class='sub'>6504-SEPA / ESP32</span></div>";
  html += "<div style='display:flex;align-items:center;gap:12px;'>";
  html += "<span><span class='refresh-dot'></span><span style='font-size:10px;color:var(--text-dim);letter-spacing:1px;'>LIVE &mdash; 5s</span></span>";
  html += "<span class='mode-pill " + modeCls + "'>" + String(modeToText(currentMode)) + "</span>";
  html += "</div></div>";

  html += "<div class='main'>";

  // ── CARD: Temperature (hero display) ─────────────────────
  // Color shifts green → amber → red based on temperature level
  String tempCol = tempDangerAlert ? "var(--red)" : (temperature > fanThreshold ? "var(--amber)" : "var(--green)");
  html += "<div class='card'><div class='card-title'>Temperature</div>";
  html += "<div class='hero'>";
  html += "<div class='hero-num' style='color:" + tempCol + ";'>" + String(temperature, 1) + "</div>";
  html += "<div class='hero-unit'>&#176;C &nbsp;/&nbsp; " + String(temperature * 9.0f / 5.0f + 32.0f, 1) + " &#176;F</div>";
  html += "<div style='margin-top:12px;font-size:10px;color:var(--text-dim);'>Range&nbsp; " + String(minTempEdit, 1) + " &mdash; " + String(maxTempEdit, 1) + " &deg;C</div>";
  html += "<div style='margin-top:8px;'><div class='bar-track'><div class='bar-fill " + String(tempDangerAlert ? "bar-red" : "bar-green") + "' style='width:" + String(tempBarPct) + "%;'></div></div></div>";
  html += "</div></div>";

  // ── CARD: Humidity ────────────────────────────────────────
  String humCol = (humLowAlert || humHighAlert) ? "var(--amber)" : "var(--green)";
  html += "<div class='card'><div class='card-title'>Humidity</div><div class='big-stat'>";
  html += "<div style='text-align:center;padding:8px 0;'><div class='hero-num' style='font-size:44px;color:" + humCol + ";'>" + String((int)humidity) + "<span style='font-size:20px;'>%</span></div></div>";
  html += "<div class='stat-row'><div class='stat-label'><span>CURRENT</span><span style='color:var(--text);'>" + String((int)humidity) + "%</span></div><div class='bar-track'><div class='bar-fill bar-blue' style='width:" + String(humBarPct) + "%;'></div></div></div>";
  html += "<div class='stat-row'><div class='stat-label'><span>24h MIN</span><span>" + String((int)minHum24) + "%</span></div></div>";
  html += "<div class='stat-row'><div class='stat-label'><span>24h MAX</span><span>" + String((int)maxHum24) + "%</span></div></div>";
  html += "</div></div>";

  // ── CARD: Soil Moisture & Light ───────────────────────────
  html += "<div class='card'><div class='card-title'>Soil &amp; Light</div><div class='big-stat'>";
  html += "<div class='stat-row'><div class='stat-label'><span>SOIL MOISTURE</span><span style='color:" + String(soilDangerAlert ? "var(--red)" : "var(--text)") + ";font-weight:700;'>" + String(soilPct) + "%</span></div><div class='bar-track'><div class='bar-fill " + String(soilDangerAlert ? "bar-red" : "bar-green") + "' style='width:" + String(soilBarPct) + "%;'></div></div></div>";
  html += "<div class='stat-row'><div class='stat-label'><span>LIGHT RAW</span><span>" + String(lightRaw) + "</span></div><div class='bar-track'><div class='bar-fill bar-amber' style='width:" + String(lightBarPct) + "%;'></div></div></div>";
  html += "<div style='margin-top:10px;padding-top:10px;border-top:1px solid var(--border);'>";
  html += "<div class='stat-label' style='margin-bottom:4px;'><span>GROW LIGHT TRIGGER</span></div>";
  html += "<div style='font-size:10px;color:var(--text-dim);'>ON above " + String(NIGHT_ON_THRESHOLD) + " &nbsp;&bull;&nbsp; OFF below " + String(NIGHT_OFF_THRESHOLD) + "</div>";
  html += "</div></div></div>";

  // ── CARD: Actuators ───────────────────────────────────────
  // Each actuator card highlights its border color when active
  html += "<div class='card'><div class='card-title'>Actuators</div><div class='act-grid'>";
  // Fan
  html += "<div class='act-item' style='" + String(fanState ? "border-color:var(--red);" : "") + "'>";
  html += "<span class='act-name'>&#128168; Fan</span>";
  html += "<span class='act-state " + String(fanState ? "fan-on" : "off") + "'>" + String(fanState ? "ON" : "OFF") + "</span>";
  html += "<span style='font-size:9px;color:var(--text-dim);'>thr " + String(fanThreshold, 1) + "&deg;C</span></div>";
  // Pump
  html += "<div class='act-item' style='" + String(pumpState ? "border-color:var(--amber);" : "") + "'>";
  html += "<span class='act-name'>&#128167; Pump</span>";
  html += "<span class='act-state " + String(pumpState ? "pump-on" : "off") + "'>" + String(pumpState ? "ON" : "OFF") + "</span>";
  html += "<span style='font-size:9px;color:var(--text-dim);'>thr " + String(pumpThreshold) + "%</span></div>";
  // Grow light
  html += "<div class='act-item' style='" + String(growState ? "border-color:var(--blue);" : "") + "'>";
  html += "<span class='act-name'>&#127774; Grow</span>";
  html += "<span class='act-state " + String(growState ? "grow-on" : "off") + "'>" + String(growState ? "ON" : "OFF") + "</span>";
  html += "<span style='font-size:9px;color:var(--text-dim);'>light low</span></div>";
  // Status
  html += "<div class='act-item' style='" + String(statusSafe ? "border-color:var(--green);" : "border-color:var(--red);") + "'>";
  html += "<span class='act-name'>&#9679; Status</span>";
  html += "<span class='act-state " + String(statusSafe ? "safe-on" : "fan-on") + "'>" + String(statusSafe ? "SAFE" : "WARN") + "</span>";
  html += "<span style='font-size:9px;color:var(--text-dim);'>system</span></div>";
  html += "</div></div>";

  // ── CARD: Alerts ──────────────────────────────────────────
  // Lambda to generate a single alert row without code repetition
  html += "<div class='card'><div class='card-title'>Alerts</div><div class='alert-list'>";
  auto alertRow = [&](const char* label, bool active, bool warn = false) {
    String cls  = active ? "warn" : "ok";
    String bCls = active ? (warn ? "badge-warn" : "badge-danger") : "badge-ok";
    html += "<div class='alert-row " + cls + "'>";
    html += "<span class='alert-label'>" + String(label) + "</span>";
    html += "<span class='alert-badge " + bCls + "'>" + String(active ? "ALERT" : "OK") + "</span>";
    html += "</div>";
  };
  alertRow("Temperature Danger (&ge;38&deg;C)", tempDangerAlert);
  alertRow("Soil Critical (&le;10%)",           soilDangerAlert);
  alertRow("Humidity Low",                      humLowAlert, true);
  alertRow("Humidity High",                     humHighAlert, true);
  // DHT sensor health row
  if (dhtError) {
    html += "<div class='alert-row warn'><span class='alert-label'>DHT22 Sensor</span><span class='alert-badge badge-danger'>ERROR</span></div>";
  } else {
    html += "<div class='alert-row ok'><span class='alert-label'>DHT22 Sensor</span><span class='alert-badge badge-ok'>OK</span></div>";
  }
  html += "</div></div>";

  // ── CARD: Thresholds ──────────────────────────────────────
  html += "<div class='card'><div class='card-title'>Thresholds</div><div class='thr-grid'>";
  html += "<div class='thr-item'><div class='thr-key'>Temp Min</div><div class='thr-val'>" + String(minTempEdit, 1) + " &deg;C</div></div>";
  html += "<div class='thr-item'><div class='thr-key'>Temp Max</div><div class='thr-val'>" + String(maxTempEdit, 1) + " &deg;C</div></div>";
  html += "<div class='thr-item'><div class='thr-key'>Hum Min</div><div class='thr-val'>"  + String((int)minHumEdit) + " %</div></div>";
  html += "<div class='thr-item'><div class='thr-key'>Hum Max</div><div class='thr-val'>"  + String((int)maxHumEdit) + " %</div></div>";
  html += "<div class='thr-item'><div class='thr-key'>Fan Temp</div><div class='thr-val'>" + String(fanThreshold, 1) + " &deg;C</div></div>";
  html += "<div class='thr-item'><div class='thr-key'>Pump Soil</div><div class='thr-val'>"+ String(pumpThreshold) + " %</div></div>";
  html += "</div></div>";

  // ── CARD: History + System (full-width) ───────────────────
  html += "<div class='card card-full'><div class='card-title'>History &amp; System</div>";
  html += "<div style='display:grid;grid-template-columns:1fr 1fr;gap:20px;'>";

  // Left column: 24h history and day/night durations
  html += "<div>";
  html += "<div class='hist-row'><span class='hist-label'>Min Temp 24h</span><span class='hist-val'>"  + String(minTemp24, 1)   + " &deg;C</span></div>";
  html += "<div class='hist-row'><span class='hist-label'>Max Temp 24h</span><span class='hist-val'>"  + String(maxTemp24, 1)   + " &deg;C</span></div>";
  html += "<div class='hist-row'><span class='hist-label'>Min Hum 24h</span><span class='hist-val'>"   + String((int)minHum24)  + " %</span></div>";
  html += "<div class='hist-row'><span class='hist-label'>Max Hum 24h</span><span class='hist-val'>"   + String((int)maxHum24)  + " %</span></div>";
  html += "<div class='hist-row'><span class='hist-label'>Last Day Duration</span><span class='hist-val'>"   + String(dayBuf)   + "</span></div>";
  html += "<div class='hist-row'><span class='hist-label'>Last Night Duration</span><span class='hist-val'>" + String(nightBuf) + "</span></div>";
  html += "</div>";

  // Right column: system status dots
  html += "<div class='sys-row' style='align-content:start;'>";
  html += "<div class='sys-item'><span class='sys-dot " + String(sdReady       ? "dot-ok"   : "dot-err")  + "'></span>SD Card: "  + String(sdReady       ? "OK"       : "Not found") + "</div>";
  html += "<div class='sys-item'><span class='sys-dot " + String(wifiConnected ? "dot-ok"   : "dot-warn") + "'></span>WiFi: "     + String(wifiConnected ? WiFi.localIP().toString() : "Offline") + "</div>";
  html += "<div class='sys-item'><span class='sys-dot dot-ok'></span>Heap: "  + String(ESP.getFreeHeap()) + " bytes</div>";
  html += "<div class='sys-item'><span class='sys-dot dot-ok'></span>Log: "   + String(logBufferFull ? LOG_SIZE : logIndex) + (logBufferFull ? " (rolling)" : " entries") + "</div>";
  html += "<div class='sys-item'><span class='sys-dot dot-ok'></span>DHT: "   + String(dhtError ? "Error" : "OK") + "</div>";
  html += "</div>";

  html += "</div></div>"; // close inner grid + history card

  html += "</div>"; // close .main grid

  html += "<div class='footer'>GREENHOUSE OS &nbsp;&bull;&nbsp; 6504-SEPA &nbsp;&bull;&nbsp; ESP32 &nbsp;&bull;&nbsp; AUTO-REFRESH 5s</div>";
  html += "</body></html>";

  server.send(200, "text/html", html);
}

// ============================================================
//  SETUP
// ============================================================

void setup() {
  Serial.begin(115200);
  delay(100);
  dht.begin();

  loadThresholds(); // Restore saved thresholds from NVS flash

  // Configure output pins and ensure all LEDs start OFF
  pinMode(FAN_LED,    OUTPUT); digitalWrite(FAN_LED,    LOW);
  pinMode(PUMP_LED,   OUTPUT); digitalWrite(PUMP_LED,   LOW);
  pinMode(GROW_LED,   OUTPUT); digitalWrite(GROW_LED,   LOW);
  pinMode(STATUS_LED, OUTPUT); digitalWrite(STATUS_LED, LOW);

  // Configure button pins with internal pull-up resistors
  // (buttons connect pin to GND → active LOW)
  pinMode(MODE_BUTTON, INPUT_PULLUP);
  pinMode(EDIT_BUTTON, INPUT_PULLUP);
  lastModeButtonState = digitalRead(MODE_BUTTON);
  lastEditButtonState = digitalRead(EDIT_BUTTON);

  // Initialise I2C LCD
  Wire.begin(LCD_SDA, LCD_SCL);
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("Greenhouse");
  lcd.setCursor(0, 1); lcd.print("Starting...");
  delay(1500);
  lcd.clear();

  // Attempt WiFi connection (10-second timeout)
  lcd.setCursor(0, 0); lcd.print("WiFi Connect..");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  unsigned long wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < 10000UL) {
    delay(250);
  }

  if (WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
    server.on("/", handleRoot); // Register web dashboard route
    server.begin();
    Serial.println("WiFi Connected!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("WiFi OK!");
    lcd.setCursor(0, 1); lcd.print(WiFi.localIP().toString());
    delay(2000);
  } else {
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("WiFi Failed");
    lcd.setCursor(0, 1); lcd.print("Running local");
    delay(1500);
  }

  lcd.clear();

  initSD();                // Mount SD card and create log file if needed
  modeStartTime = millis();
  computeMinMax24();       // Initialise min/max with current readings

  Serial.println("Smart Greenhouse System Ready");
}

// ============================================================
//  MAIN LOOP
// ============================================================

void loop() {
  unsigned long currentTime = millis();

  // ── Serial command: 'R' or 'r' triggers factory reset ──
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == 'R' || cmd == 'r') {
      factoryReset();
    }
  }

  // ── Handle incoming HTTP requests ──
  if (wifiConnected) {
    server.handleClient();
  }

  // ── Poll buttons every loop iteration ──
  handleModeButton();
  handleEditButton();

  // ── Read DHT22 (max once per DHT_INTERVAL to avoid sensor errors) ──
  if (currentTime - lastDHTTime >= DHT_INTERVAL) {
    lastDHTTime = currentTime;

    float newHum  = dht.readHumidity();
    float newTemp = dht.readTemperature();

    if (isnan(newHum) || isnan(newTemp)) {
      // Keep last valid values; flag the error for display and alerts
      dhtError = true;
      Serial.println("ERROR: DHT22 read failed - keeping last valid values");
      lcdDirty = true;
    } else {
      humidity    = newHum;
      temperature = newTemp;
      dhtReady    = true;
      dhtError    = false;
      lcdDirty    = true;
    }
  }

  // ── Read analog sensors every loop (fast, no delay needed) ──
  // Soil: map ADC 0-4095 → 100-0 % (inverted: dry = high ADC)
  soilPct  = map(analogRead(SOIL_PIN), 0, 4095, 100, 0);
  soilPct  = constrain(soilPct, 0, 100);
  lightRaw = analogRead(LDR_PIN);

  // ── Wait for first valid DHT reading before running control logic ──
  if (!dhtReady) {
    if (lcdDirty) {
      lcdDirty = false;
      drawLCD();
    }
    return;
  }

  // ── Core control loop ──
  updateMode(lightRaw, currentTime); // Check for NIGHT mode transitions
  applyActuatorLogic();              // Decide fan/pump/grow states
  evaluateAlerts();                  // Set alert flags
  writeOutputs();                    // Write states to output pins + status LED

  // ── Periodic serial debug print ──
  if (currentTime - lastPrintTime >= PRINT_INTERVAL) {
    lastPrintTime = currentTime;

    char dayBuf[6], nightBuf[6];
    formatDuration(lastDayDuration, dayBuf);
    formatDuration(lastNightDuration, nightBuf);

    const char* tempStatus  = (temperature > maxTempEdit)    ? "ABOVE range" :
                              (temperature < minTempEdit)    ? "BELOW range" : "WITHIN range";
    const char* humStatus   = (humidity > maxHumEdit)        ? "ABOVE range" :
                              (humidity < minHumEdit)        ? "BELOW range" : "WITHIN range";
    const char* soilStatus  = (soilPct < pumpThreshold)      ? "BELOW range" :
                              (soilPct > pumpThreshold + 20) ? "ABOVE range" : "WITHIN range";
    const char* lightStatus = growState                      ? "BELOW range" : "WITHIN range";

    Serial.println("----------- SENSOR READINGS -----------");
    Serial.print("Mode: ");       Serial.println(modeToText(currentMode));
    Serial.print("User Mode: ");  Serial.println(modeToText(userSelectedMode));
    Serial.println("--- Thresholds ---");
    Serial.print("Fan ON above:  "); Serial.print(fanThreshold);  Serial.println(" C");
    Serial.print("Pump ON below: "); Serial.print(pumpThreshold); Serial.println(" %");
    Serial.print("Temp range:    "); Serial.print(minTempEdit,1); Serial.print(" - "); Serial.print(maxTempEdit,1); Serial.println(" C");
    Serial.print("Hum range:     "); Serial.print(minHumEdit,1);  Serial.print(" - "); Serial.print(maxHumEdit,1);  Serial.println(" %");
    Serial.println("--- Readings & Status ---");
    Serial.print("Temperature:   "); Serial.print(temperature,1); Serial.print(" C  -> "); Serial.println(tempStatus);
    Serial.print("Humidity:      "); Serial.print(humidity,1);    Serial.print(" %  -> "); Serial.println(humStatus);
    Serial.print("Soil Moisture: "); Serial.print(soilPct);       Serial.print(" %  -> "); Serial.println(soilStatus);
    Serial.print("Light Raw:     "); Serial.print(lightRaw);      Serial.print("     -> "); Serial.println(lightStatus);
    Serial.println("--- Actuators & Status LEDs ---");
    Serial.print("RED    Fan:        "); Serial.println(fanState   ? "ON  (temp HIGH)"  : "OFF");
    Serial.print("YELLOW Pump:       "); Serial.println(pumpState  ? "ON  (soil LOW)"   : "OFF");
    Serial.print("BLUE   Grow Light: "); Serial.println(growState  ? "ON  (light LOW)"  : "OFF");
    Serial.print("GREEN  Status:     "); Serial.println(statusSafe ? "ON  (all OK)"     : "OFF (warning active)");
    Serial.println("--- Alerts ---");
    Serial.print("Temp Danger: ");    Serial.println(tempDangerAlert ? "YES" : "NO");
    Serial.print("Soil Danger: ");    Serial.println(soilDangerAlert ? "YES" : "NO");
    Serial.print("Humidity Low: ");   Serial.println(humLowAlert ? "YES" : "NO");
    Serial.print("Humidity High: ");  Serial.println(humHighAlert ? "YES" : "NO");
    Serial.println("--- History ---");
    Serial.print("Min/Max Temp 24h: "); Serial.print(minTemp24,1);   Serial.print(" / "); Serial.print(maxTemp24,1);   Serial.println(" C");
    Serial.print("Min/Max Hum 24h:  "); Serial.print((int)minHum24); Serial.print(" / "); Serial.print((int)maxHum24); Serial.println(" %");
    Serial.print("Last Day:   "); Serial.println(dayBuf);
    Serial.print("Last Night: "); Serial.println(nightBuf);
    Serial.print("SD Card: ");   Serial.println(sdReady ? "OK" : "Not found");
    Serial.print("WiFi: ");      Serial.println(wifiConnected ? "Connected" : "Not connected");
    if (wifiConnected) {
      Serial.print("IP: ");
      Serial.println(WiFi.localIP());
    }
    Serial.print("Free Heap: "); Serial.print(ESP.getFreeHeap()); Serial.println(" bytes");
    Serial.print("Log: ");       Serial.print(logBufferFull ? LOG_SIZE : logIndex);
    if (logBufferFull) Serial.print(" (FULL - oldest overwritten)");
    Serial.println();
    Serial.println("--------------------------------------\n");
  }

  // ── Periodic RAM log entry (circular buffer + SD card) ──
  if (currentTime - lastLogTime >= LOG_INTERVAL_MS) {
    lastLogTime = currentTime;

    // Write current readings into the circular buffer at logIndex
    tempLog[logIndex]  = temperature;
    humLog[logIndex]   = humidity;
    soilLog[logIndex]  = soilPct;
    lightLog[logIndex] = lightRaw;
    modeLog[logIndex]  = (int)currentMode;
    fanLog[logIndex]   = fanState  ? 1 : 0;
    pumpLog[logIndex]  = pumpState ? 1 : 0;
    growLog[logIndex]  = growState ? 1 : 0;

    logIndex++;
    if (logIndex >= LOG_SIZE) {
      logIndex      = 0;       // Wrap around — oldest entry will be overwritten next
      logBufferFull = true;
      Serial.println("INFO: Log buffer wrapped - oldest overwritten");
    }

    computeMinMax24(); // Recalculate 24h min/max from updated buffer

    Serial.print("LOG #"); Serial.print(logIndex);
    Serial.print(" T:");   Serial.print(temperature,1);
    Serial.print(" H:");   Serial.print((int)humidity);
    Serial.print(" S:");   Serial.print(soilPct);
    Serial.print(" L:");   Serial.println(lightRaw);

    logToSD(); // Persist entry to SD card CSV
  }

  // ── LCD screen rotation (skip in EDIT mode) ──
  if (currentMode != EDIT_MODE) {
    if (currentTime - lastScreenTime >= SCREEN_INTERVAL) {
      lastScreenTime = currentTime;
      screenIndex    = (screenIndex + 1) % 5; // Cycle through 5 screens
      lcdDirty       = true;
    }
  }

  // ── Redraw LCD only when content has changed ──
  if (lcdDirty) {
    lcdDirty = false;
    drawLCD();
  }
}