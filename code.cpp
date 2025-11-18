#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <SPI.h>
#include <arduinoFFT.h>
#include <ESP32Servo.h>

// ===== TFT =====
#define TFT_CS   5
#define TFT_DC   21
#define TFT_RST  4
Adafruit_ILI9341 tft(TFT_CS, TFT_DC, TFT_RST);

// ===== BUTTONS =====
#define BTN_POWER        32
#define BTN_START_STOP   25
#define BTN_STRING_SEL   26
#define BTN_TUNING_MODE  27

// Button debouncing
unsigned long lastDebounceTime[4] = {0, 0, 0, 0};
const unsigned long debounceDelay = 50;
bool lastButtonState[4] = {HIGH, HIGH, HIGH, HIGH};
bool buttonState[4] = {HIGH, HIGH, HIGH, HIGH};

// ===== PIEZO SENSOR =====
const int PIEZO_PIN = 34;  // ADC pin for piezo sensor

// ===== FFT Configuration =====
const uint16_t SAMPLES = 2048;
const double SAMPLING_FREQ = 8192;  // May need adjustment based on piezo response
ArduinoFFT<double> FFT = ArduinoFFT<double>();
double *vReal;
double *vImag;

// ===== SYSTEM STATES =====
enum SystemState {
  STATE_OFF,
  STATE_STANDBY,
  STATE_TUNING,
  STATE_AUTO_TUNE_ALL,
  STATE_STRING_SELECT,
  STATE_MODE_SELECT,
  STATE_STATISTICS,
  STATE_SETTINGS
};

enum TuningMode {
  MODE_STANDARD,
  MODE_HALF_STEP_DOWN,
  MODE_HALF_STEP_UP,
  MODE_FULL_STEP_DOWN
};

SystemState currentState = STATE_STANDBY;
TuningMode tuningMode = MODE_STANDARD;
int selectedString = -1;
bool isAutoMode = true;

// ===== AUTO TUNE ALL =====
int autoTuneCurrentString = 0;
bool autoTuneInProgress = false;
unsigned long autoTuneStringStartTime = 0;
const unsigned long AUTO_TUNE_TIMEOUT = 30000;  // 30s per string max

// ===== TUNING DEFINITIONS =====
struct TuningDef {
  const char* name;
  float freqs[6];
};

TuningDef tuningModes[] = {
  {"STANDARD", {82.41, 110.0, 146.83, 196.0, 246.94, 329.63}},
  {"1/2 STEP DOWN", {77.78, 103.83, 138.59, 185.0, 233.08, 311.13}},
  {"1/2 STEP UP", {87.31, 116.54, 155.56, 207.65, 261.63, 349.23}},
  {"FULL STEP DOWN", {73.42, 98.0, 130.81, 174.61, 220.0, 293.66}}
};

const char* STRING_NAMES[] = {"E2", "A2", "D3", "G3", "B3", "E4"};

// Analysis settings - adjusted for piezo
const float F_MIN = 70.0f;
const float F_MAX = 1000.0f;
const float NOISE_THRESHOLD = 15.0;  // Adjusted for piezo - may need tuning
int TUNE_TOLERANCE = 5;  // Now adjustable in settings

// Smoothing
const int SMOOTH_WINDOW = 3;
float freqHistory[SMOOTH_WINDOW] = {0};
int histIdx = 0;

const char* NOTE_NAMES[] = {"C","C#","D","D#","E","F","F#","G","G#","A","A#","B"};

// Signal meter
float signalLevel = 0;

// ===== HISTORY GRAPH =====
const int HISTORY_SIZE = 40;
float freqHistoryGraph[HISTORY_SIZE] = {0};
int historyGraphIdx = 0;

// Add this to the global variables section (around line 100)
unsigned long lastReadTime = 0;
const unsigned long READ_INTERVAL = 3000;  // 3 seconds

// ===== STATISTICS =====
struct Statistics {
  int totalStringsTuned;
  int sessionStringsTuned;
  unsigned long totalTuningTime;  // milliseconds
  unsigned long sessionStartTime;
  int successfulTunes;
  int failedTunes;
  float avgTuningTime;  // seconds
} stats = {0, 0, 0, 0, 0, 0, 0.0};

unsigned long currentTuneStartTime = 0;

// ===== SERVO =====
Servo tunerServo;
const int SERVO_PIN = 33;
int servoPos = 90;
int targetServoPos = 90;
unsigned long lastServoMove = 0;
uint32_t SERVO_MOVE_PERIOD = 150;  // Adjustable in settings
bool servoAttached = false;

// Tuning history for stability
int TUNE_STABLE_COUNT = 5;  // Adjustable in settings
int stableCount = 0;
int lastCents = 0;

// Servo movement tracking
bool servoIsMoving = false;
unsigned long servoMoveStartTime = 0;
const uint32_t SERVO_MOVE_DURATION = 120;

// ===== ANIMATION =====
int animationFrame = 0;
unsigned long lastAnimationTime = 0;
bool showSuccessAnimation = false;
int successAnimationFrame = 0;
unsigned long successAnimationStartTime = 0;  // NEW
const unsigned long SUCCESS_DISPLAY_TIME = 3000;  // NEW (3 seconds)

// ===== BATTERY SIMULATION =====
int batteryLevel = 100;  // Percentage (simulated for now)

// ===== COLOR SCHEME =====
#define COLOR_BG        0x0841
#define COLOR_CARD      0x1082
#define COLOR_PRIMARY   0x07FF
#define COLOR_SUCCESS   0x07E0
#define COLOR_WARNING   0xFD20
#define COLOR_DANGER    0xF800
#define COLOR_TEXT      0xFFFF
#define COLOR_TEXT_DIM  0x7BEF
#define COLOR_ACCENT    0x07FF
#define COLOR_PURPLE    0x781F
#define COLOR_GOLD      0xFEA0

// ===== PIEZO CALIBRATION =====
// These values may need adjustment based on your piezo/amplifier setup
float PIEZO_GAIN_ADJUST = 1.0;  // Adjust if signal too weak/strong
bool USE_DC_BLOCK = true;  // Remove DC offset from piezo

// ===== UI HELPER FUNCTIONS =====

void drawCard(int x, int y, int w, int h, uint16_t color = COLOR_CARD) {
  tft.fillRoundRect(x, y, w, h, 8, color);
}

void drawProgressBar(int x, int y, int w, int h, int value, int maxValue, uint16_t color) {
  tft.drawRoundRect(x, y, w, h, 4, COLOR_TEXT_DIM);
  int fillWidth = map(constrain(value, 0, maxValue), 0, maxValue, 0, w - 4);
  if (fillWidth > 0) {
    tft.fillRoundRect(x + 2, y + 2, fillWidth, h - 4, 2, color);
  }
}

void drawCentsMeter(int x, int y, int w, int h, int cents) {
  tft.fillRoundRect(x, y, w, h, 6, COLOR_CARD);
  
  int centerX = x + w/2;
  tft.drawFastVLine(centerX, y + 5, h - 10, COLOR_TEXT_DIM);
  
  int tolerancePixels = map(TUNE_TOLERANCE, 0, 50, 0, w/2);
  tft.drawFastVLine(centerX - tolerancePixels, y + 5, h - 10, COLOR_SUCCESS);
  tft.drawFastVLine(centerX + tolerancePixels, y + 5, h - 10, COLOR_SUCCESS);
  
  int c = constrain(cents, -50, 50);
  int indicatorX;
  if (c < 0) {
    indicatorX = centerX + map(c, -50, 0, -w/2 + 8, 0);
  } else {
    indicatorX = centerX + map(c, 0, 50, 0, w/2 - 8);
  }
  
  uint16_t color = COLOR_SUCCESS;
  if (abs(cents) > TUNE_TOLERANCE && abs(cents) <= 15) color = COLOR_WARNING;
  else if (abs(cents) > 15) color = COLOR_DANGER;
  
  tft.fillCircle(indicatorX, y + h/2, 8, color);
  tft.drawCircle(indicatorX, y + h/2, 9, COLOR_TEXT);
}

void drawBatteryIcon(int x, int y) {
  // Battery outline
  tft.drawRect(x, y + 2, 20, 10, COLOR_TEXT_DIM);
  tft.fillRect(x + 20, y + 4, 2, 6, COLOR_TEXT_DIM);
  
  // Battery fill
  uint16_t fillColor = COLOR_SUCCESS;
  if (batteryLevel < 20) fillColor = COLOR_DANGER;
  else if (batteryLevel < 50) fillColor = COLOR_WARNING;
  
  int fillWidth = map(batteryLevel, 0, 100, 0, 16);
  if (fillWidth > 0) {
    tft.fillRect(x + 2, y + 4, fillWidth, 6, fillColor);
  }
  
  // Percentage text
  tft.setTextSize(1);
  tft.setTextColor(COLOR_TEXT_DIM);
  tft.setCursor(x + 24, y + 3);
  tft.print(batteryLevel);
  tft.print("%");
}

void drawFrequencyGraph(int x, int y, int w, int h, float targetFreq) {
  // Background
  tft.fillRect(x, y, w, h, COLOR_BG);
  tft.drawRect(x, y, w, h, COLOR_TEXT_DIM);
  
  // Target line
  int targetY = y + h/2;
  tft.drawFastHLine(x, targetY, w, COLOR_SUCCESS);
  
  // Draw history
  for (int i = 1; i < HISTORY_SIZE; i++) {
    int idx1 = (historyGraphIdx + i - 1) % HISTORY_SIZE;
    int idx2 = (historyGraphIdx + i) % HISTORY_SIZE;
    
    if (freqHistoryGraph[idx1] > 0 && freqHistoryGraph[idx2] > 0) {
      float cents1 = 1200.0f * log2f(freqHistoryGraph[idx1] / targetFreq);
      float cents2 = 1200.0f * log2f(freqHistoryGraph[idx2] / targetFreq);
      
      int y1 = targetY - map(constrain(cents1, -50, 50), -50, 50, -h/2 + 2, h/2 - 2);
      int y2 = targetY - map(constrain(cents2, -50, 50), -50, 50, -h/2 + 2, h/2 - 2);
      
      int x1 = x + map(i - 1, 0, HISTORY_SIZE - 1, 0, w);
      int x2 = x + map(i, 0, HISTORY_SIZE - 1, 0, w);
      
      tft.drawLine(x1, y1, x2, y2, COLOR_ACCENT);
    }
  }
}

void addToFrequencyHistory(float freq) {
  freqHistoryGraph[historyGraphIdx] = freq;
  historyGraphIdx = (historyGraphIdx + 1) % HISTORY_SIZE;
}

void drawSuccessAnimation() {
  if (!showSuccessAnimation) return;
  
  // Pulsing checkmark
  int centerX = 160;
  int centerY = 120;
  int size = 30 + (successAnimationFrame % 10);
  
  // Green circle
  tft.fillCircle(centerX, centerY, size, COLOR_SUCCESS);
  
  // Checkmark
  tft.drawLine(centerX - 10, centerY, centerX - 3, centerY + 10, COLOR_TEXT);
  tft.drawLine(centerX - 3, centerY + 10, centerX + 12, centerY - 10, COLOR_TEXT);
  tft.drawLine(centerX - 9, centerY, centerX - 3, centerY + 9, COLOR_TEXT);
  tft.drawLine(centerX - 3, centerY + 9, centerX + 11, centerY - 10, COLOR_TEXT);
  
  // Stars around
  for (int i = 0; i < 4; i++) {
    int angle = (successAnimationFrame * 10 + i * 90) % 360;
    int sx = centerX + cos(angle * PI / 180) * (40 + successAnimationFrame);
    int sy = centerY + sin(angle * PI / 180) * (40 + successAnimationFrame);
    tft.fillCircle(sx, sy, 3, COLOR_GOLD);
  }
  
  successAnimationFrame++;
  if (successAnimationFrame > 60) {  // Loop the animation
    successAnimationFrame = 0;
  }
}

// ===== UI SCREENS =====

void drawStandbyScreen() {
  tft.fillScreen(COLOR_BG);
  
  // Battery indicator
  drawBatteryIcon(250, 5);
  
  // Header card
  drawCard(10, 10, 300, 60);
  tft.setTextSize(3);
  tft.setTextColor(COLOR_PRIMARY);
  tft.setCursor(50, 25);
  tft.print("GUITAR TUNER");
  
  // Status card
  drawCard(10, 80, 300, 50);
  tft.setTextSize(2);
  tft.setTextColor(COLOR_TEXT_DIM);
  tft.setCursor(20, 88);
  tft.print("Status:");
  tft.setTextSize(2);
  tft.setTextColor(COLOR_WARNING);
  tft.setCursor(110, 88);
  tft.print("READY");
  
  // Session stats
  tft.setTextSize(1);
  tft.setTextColor(COLOR_ACCENT);
  tft.setCursor(20, 108);
  tft.print("Session: ");
  tft.print(stats.sessionStringsTuned);
  tft.print(" strings");
  
  // Tuning mode
  drawCard(10, 140, 145, 38);
  tft.setTextSize(1);
  tft.setTextColor(COLOR_TEXT_DIM);
  tft.setCursor(20, 148);
  tft.print("MODE");
  tft.setTextSize(1);
  tft.setTextColor(COLOR_ACCENT);
  tft.setCursor(20, 160);
  tft.print(tuningModes[tuningMode].name);
  
  // String mode
  drawCard(165, 140, 145, 38);
  tft.setTextSize(1);
  tft.setTextColor(COLOR_TEXT_DIM);
  tft.setCursor(175, 148);
  tft.print("STRING");
  tft.setTextSize(1);
  tft.setTextColor(COLOR_ACCENT);
  tft.setCursor(175, 160);
  if (isAutoMode) tft.print("AUTO");
  else tft.print(STRING_NAMES[selectedString]);
  
  // Action buttons
  drawCard(10, 188, 145, 40);
  tft.setTextSize(2);
  tft.setTextColor(COLOR_SUCCESS);
  tft.setCursor(23, 200);
  tft.print("START");
  
  drawCard(165, 188, 145, 40);
  tft.setTextSize(2);
  tft.setTextColor(COLOR_PURPLE);
  tft.setCursor(175, 200);
  tft.print("AUTO ALL");
}

void drawTuningScreen() {
  tft.fillScreen(COLOR_BG);
  
  // Battery
  drawBatteryIcon(250, 5);
  
  // Compact header
  drawCard(10, 5, 230, 30);
  tft.setTextSize(2);
  tft.setTextColor(COLOR_SUCCESS);
  tft.setCursor(20, 12);
  tft.print("TUNING");
  
  tft.setTextSize(1);
  tft.setTextColor(COLOR_TEXT_DIM);
  tft.setCursor(150, 12);
  tft.print(tuningModes[tuningMode].name);
  
  // Signal level
  drawCard(10, 40, 300, 28);
  tft.setTextSize(1);
  tft.setTextColor(COLOR_TEXT_DIM);
  tft.setCursor(20, 47);
  tft.print("SIGNAL");
  
  // Frequency and note
  drawCard(10, 73, 145, 40);
  tft.setTextSize(1);
  tft.setTextColor(COLOR_TEXT_DIM);
  tft.setCursor(20, 80);
  tft.print("FREQUENCY");
  
  drawCard(165, 73, 145, 40);
  tft.setTextSize(1);
  tft.setTextColor(COLOR_TEXT_DIM);
  tft.setCursor(175, 80);
  tft.print("NOTE");
  
  // Frequency history graph
  tft.setTextSize(1);
  tft.setTextColor(COLOR_TEXT_DIM);
  tft.setCursor(20, 118);
  tft.print("TUNING PROGRESS");
  // Graph area: 10, 128, 300, 40
  
  // Cents meter
  tft.setTextSize(1);
  tft.setTextColor(COLOR_TEXT_DIM);
  tft.setCursor(20, 173);
  tft.print("ACCURACY");
  // Meter will be drawn in update
}

void drawAutoTuneAllScreen() {
  tft.fillScreen(COLOR_BG);
  
  drawBatteryIcon(250, 5);
  
  // Header
  drawCard(10, 10, 300, 45);
  tft.setTextSize(3);
  tft.setTextColor(COLOR_PURPLE);
  tft.setCursor(30, 20);
  tft.print("AUTO TUNE");
  
  // Progress
  drawCard(10, 65, 300, 60);
  tft.setTextSize(2);
  tft.setTextColor(COLOR_TEXT);
  tft.setCursor(20, 75);
  tft.print("Progress:");
  
  // String indicators
  int yStart = 95;
  for (int i = 0; i < 6; i++) {
    int x = 20 + (i % 3) * 95;
    int y = yStart + (i / 3) * 25;
    
    uint16_t color = COLOR_TEXT_DIM;
    if (i < autoTuneCurrentString) color = COLOR_SUCCESS;  // Done
    else if (i == autoTuneCurrentString) color = COLOR_WARNING;  // Current
    
    tft.setTextSize(2);
    tft.setTextColor(color);
    tft.setCursor(x, y);
    tft.print(STRING_NAMES[i]);
    
    if (i < autoTuneCurrentString) {
      tft.print(" ");
      tft.setTextColor(COLOR_SUCCESS);
      tft.print((char)251);  // Checkmark
    } else if (i == autoTuneCurrentString) {
      tft.print(" ...");
    }
  }
  
  // Overall progress bar
  tft.setTextSize(1);
  tft.setTextColor(COLOR_TEXT_DIM);
  tft.setCursor(20, 150);
  tft.print("Overall Progress:");
  drawProgressBar(20, 163, 280, 20, autoTuneCurrentString, 6, COLOR_PURPLE);
  
  // Current string details
  if (autoTuneCurrentString < 6) {
    drawCard(10, 190, 300, 45);
    tft.setTextSize(1);
    tft.setTextColor(COLOR_TEXT_DIM);
    tft.setCursor(20, 198);
    tft.print("Current: ");
    tft.setTextSize(2);
    tft.setTextColor(COLOR_WARNING);
    tft.print(STRING_NAMES[autoTuneCurrentString]);
    tft.print(" (");
    tft.print((int)tuningModes[tuningMode].freqs[autoTuneCurrentString]);
    tft.print(" Hz)");
  }
}

void updateAutoTuneAllScreen(float freq, int cents) {
  // Update current string status
  tft.fillRect(20, 213, 280, 20, COLOR_CARD);
  tft.setTextSize(1);
  tft.setTextColor(COLOR_ACCENT);
  tft.setCursor(20, 215);
  
  if (freq > 0) {
    tft.print("Detected: ");
    tft.print((int)freq);
    tft.print(" Hz (");
    if (cents > 0) tft.print("+");
    tft.print(cents);
    tft.print(" cents)");
  } else {
    tft.print("Waiting for signal...");
  }
}

void drawStringSelectScreen() {
  tft.fillScreen(COLOR_BG);
  
  drawBatteryIcon(250, 5);
  
  drawCard(10, 5, 300, 35);
  tft.setTextSize(2);
  tft.setTextColor(COLOR_PRIMARY);
  tft.setCursor(30, 12);
  tft.print("SELECT STRING");
  
  bool autoSelected = isAutoMode;
  drawCard(10, 45, 145, 38);
  tft.setTextSize(2);
  tft.setTextColor(autoSelected ? COLOR_SUCCESS : COLOR_TEXT);
  tft.setCursor(40, 55);
  tft.print("AUTO");
  if (autoSelected) {
    tft.drawRoundRect(10, 45, 145, 38, 8, COLOR_SUCCESS);
    tft.drawRoundRect(11, 46, 143, 36, 7, COLOR_SUCCESS);
  }
  
  tft.setTextSize(1);
  tft.setTextColor(COLOR_ACCENT);
  tft.setCursor(165, 50);
  tft.print("Selected:");
  tft.setTextSize(2);
  tft.setCursor(165, 62);
  if (isAutoMode) {
    tft.print("AUTO");
  } else {
    tft.print(STRING_NAMES[selectedString]);
  }
  
  for (int i = 0; i < 6; i++) {
    int col = i % 2;
    int row = i / 2;
    int x = 10 + col * 155;
    int y = 90 + row * 48;
    
    bool selected = (!isAutoMode && selectedString == i);
    drawCard(x, y, 145, 42);
    
    tft.setTextSize(2);
    tft.setTextColor(selected ? COLOR_SUCCESS : COLOR_TEXT);
    tft.setCursor(x + 10, y + 8);
    tft.print(STRING_NAMES[i]);
    
    tft.setTextSize(1);
    tft.setTextColor(COLOR_TEXT_DIM);
    tft.setCursor(x + 10, y + 28);
    tft.print((int)tuningModes[tuningMode].freqs[i]);
    tft.print("Hz");
    
    if (selected) {
      tft.drawRoundRect(x, y, 145, 42, 8, COLOR_SUCCESS);
      tft.drawRoundRect(x + 1, y + 1, 143, 40, 7, COLOR_SUCCESS);
    }
  }
  
  tft.setTextSize(1);
  tft.setTextColor(COLOR_TEXT_DIM);
  tft.setCursor(10, 228);
  tft.print("Press STRING SEL to cycle");
}

void drawModeSelectScreen() {
  tft.fillScreen(COLOR_BG);
  
  drawBatteryIcon(250, 5);
  
  drawCard(10, 10, 300, 50);
  tft.setTextSize(2);
  tft.setTextColor(COLOR_PRIMARY);
  tft.setCursor(20, 22);
  tft.print("TUNING MODE");
  
  for (int i = 0; i < 4; i++) {
    int y = 70 + i * 42;
    bool selected = (tuningMode == i);
    
    drawCard(10, y, 300, 38);
    
    tft.setTextSize(2);
    tft.setTextColor(selected ? COLOR_SUCCESS : COLOR_TEXT);
    tft.setCursor(20, y + 10);
    tft.print(tuningModes[i].name);
    
    if (selected) {
      tft.drawRoundRect(10, y, 300, 38, 8, COLOR_SUCCESS);
      tft.drawRoundRect(11, y + 1, 298, 36, 7, COLOR_SUCCESS);
    }
  }
}

void drawStatisticsScreen() {
  tft.fillScreen(COLOR_BG);
  
  drawBatteryIcon(250, 5);
  
  drawCard(10, 10, 300, 50);
  tft.setTextSize(3);
  tft.setTextColor(COLOR_GOLD);
  tft.setCursor(40, 22);
  tft.print("STATISTICS");
  
  // Session stats
  drawCard(10, 70, 145, 75);
  tft.setTextSize(1);
  tft.setTextColor(COLOR_TEXT_DIM);
  tft.setCursor(20, 78);
  tft.print("SESSION");
  tft.setTextSize(3);
  tft.setTextColor(COLOR_SUCCESS);
  tft.setCursor(35, 100);
  tft.print(stats.sessionStringsTuned);
  tft.setTextSize(1);
  tft.setTextColor(COLOR_TEXT_DIM);
  tft.setCursor(20, 130);
  tft.print("Strings Tuned");
  
  // Total stats
  drawCard(165, 70, 145, 75);
  tft.setTextSize(1);
  tft.setTextColor(COLOR_TEXT_DIM);
  tft.setCursor(175, 78);
  tft.print("LIFETIME");
  tft.setTextSize(3);
  tft.setTextColor(COLOR_PRIMARY);
  tft.setCursor(190, 100);
  tft.print(stats.totalStringsTuned);
  tft.setTextSize(1);
  tft.setTextColor(COLOR_TEXT_DIM);
  tft.setCursor(175, 130);
  tft.print("Total Tuned");
  
  // Success rate
  drawCard(10, 155, 145, 60);
  tft.setTextSize(1);
  tft.setTextColor(COLOR_TEXT_DIM);
  tft.setCursor(20, 163);
  tft.print("SUCCESS RATE");
  
  int total = stats.successfulTunes + stats.failedTunes;
  int successRate = total > 0 ? (stats.successfulTunes * 100) / total : 100;
  tft.setTextSize(3);
  tft.setTextColor(COLOR_SUCCESS);
  tft.setCursor(40, 180);
  tft.print(successRate);
  tft.print("%");
  
  // Avg time
  drawCard(165, 155, 145, 60);
  tft.setTextSize(1);
  tft.setTextColor(COLOR_TEXT_DIM);
  tft.setCursor(175, 163);
  tft.print("AVG TIME");
  tft.setTextSize(2);
  tft.setTextColor(COLOR_WARNING);
  tft.setCursor(180, 185);
  tft.print(stats.avgTuningTime, 1);
  tft.print("s");
  
  // Instructions
  tft.setTextSize(1);
  tft.setTextColor(COLOR_TEXT_DIM);
  tft.setCursor(50, 225);
  tft.print("Press any button to return");
}

void drawSettingsScreen() {
  tft.fillScreen(COLOR_BG);
  
  drawBatteryIcon(250, 5);
  
  drawCard(10, 10, 300, 45);
  tft.setTextSize(3);
  tft.setTextColor(COLOR_ACCENT);
  tft.setCursor(60, 20);
  tft.print("SETTINGS");
  
  // Tolerance
  drawCard(10, 65, 300, 40);
  tft.setTextSize(1);
  tft.setTextColor(COLOR_TEXT_DIM);
  tft.setCursor(20, 73);
  tft.print("Tune Tolerance:");
  tft.setTextSize(2);
  tft.setTextColor(COLOR_SUCCESS);
  tft.setCursor(20, 88);
  tft.print("+/- ");
  tft.print(TUNE_TOLERANCE);
  tft.print(" cents");
  
  // Servo speed
  drawCard(10, 115, 300, 40);
  tft.setTextSize(1);
  tft.setTextColor(COLOR_TEXT_DIM);
  tft.setCursor(20, 123);
  tft.print("Servo Speed:");
  tft.setTextSize(2);
  tft.setTextColor(COLOR_WARNING);
  tft.setCursor(20, 138);
  tft.print(SERVO_MOVE_PERIOD);
  tft.print(" ms");
  
  // Stability
  drawCard(10, 165, 300, 40);
  tft.setTextSize(1);
  tft.setTextColor(COLOR_TEXT_DIM);
  tft.setCursor(20, 173);
  tft.print("Stability Count:");
  tft.setTextSize(2);
  tft.setTextColor(COLOR_PRIMARY);
  tft.setCursor(20, 188);
  tft.print(TUNE_STABLE_COUNT);
  tft.print(" samples");
  
  tft.setTextSize(1);
  tft.setTextColor(COLOR_TEXT_DIM);
  tft.setCursor(30, 220);
  tft.print("Long press buttons to adjust");
}

void updateTuningScreen(float freq, const String& note, int cents) {
  // Update signal level
  int signalPercent = constrain(map(signalLevel, 0, 100, 0, 100), 0, 100);
  uint16_t signalColor = (signalLevel > NOISE_THRESHOLD) ? COLOR_SUCCESS : COLOR_DANGER;
  
  tft.fillRect(100, 47, 200, 15, COLOR_CARD);
  drawProgressBar(100, 47, 200, 15, signalPercent, 100, signalColor);
  
  // Update frequency
  tft.fillRect(20, 93, 125, 18, COLOR_CARD);
  tft.setTextSize(2);
  tft.setTextColor(COLOR_PRIMARY);
  tft.setCursor(20, 95);
  if (freq > 0) {
    tft.print((int)freq);
    tft.setTextSize(1);
    tft.print("Hz");
  } else {
    tft.setTextSize(1);
    tft.print("--");
  }
  
  // Update note
  tft.fillRect(175, 93, 125, 18, COLOR_CARD);
  tft.setTextSize(2);
  tft.setTextColor(COLOR_WARNING);
  tft.setCursor(200, 95);
  tft.print(note);
  
  // Update frequency graph
  if (freq > 0) {
    addToFrequencyHistory(freq);
    int stringNum = identifyString(freq);
    if (stringNum >= 0) {
      float targetFreq = tuningModes[tuningMode].freqs[stringNum];
      drawFrequencyGraph(10, 128, 300, 40, targetFreq);
    }
  }
  
  // Update cents meter
  drawCentsMeter(10, 183, 300, 35, cents);
  
  // Update cents value and status
  tft.fillRect(10, 223, 300, 15, COLOR_BG);
  tft.setTextSize(2);
  uint16_t centsColor = COLOR_SUCCESS;
  if (abs(cents) > TUNE_TOLERANCE && abs(cents) <= 15) centsColor = COLOR_WARNING;
  else if (abs(cents) > 15) centsColor = COLOR_DANGER;
  
  tft.setTextColor(centsColor);
  tft.setCursor(20, 223);
  if (abs(cents) <= TUNE_TOLERANCE) {
    tft.print("IN TUNE!");
  } else if (servoIsMoving) {
    tft.print("TUNING...");
  } else if (cents < 0) {
    tft.print("TOO FLAT");
  } else {
    tft.print("TOO SHARP");
  }
  
  tft.setCursor(200, 223);
  if (cents > 0) tft.print("+");
  tft.print(cents);
  tft.print(" c");
}

// ===== BUTTON HANDLING =====

void initButtons() {
  pinMode(BTN_POWER, INPUT_PULLUP);
  pinMode(BTN_START_STOP, INPUT_PULLUP);
  pinMode(BTN_STRING_SEL, INPUT_PULLUP);
  pinMode(BTN_TUNING_MODE, INPUT_PULLUP);
}

bool readButton(int buttonIndex, int pin) {
  bool reading = digitalRead(pin);
  
  if (reading != lastButtonState[buttonIndex]) {
    lastDebounceTime[buttonIndex] = millis();
  }
  
  bool pressed = false;
  if ((millis() - lastDebounceTime[buttonIndex]) > debounceDelay) {
    if (reading != buttonState[buttonIndex]) {
      buttonState[buttonIndex] = reading;
      if (buttonState[buttonIndex] == LOW) {
        pressed = true;
      }
    }
  }
  
  lastButtonState[buttonIndex] = reading;
  return pressed;
}

void handleButtons() {
  // Power button
  if (readButton(0, BTN_POWER)) {
    if (currentState == STATE_OFF) {
      currentState = STATE_STANDBY;
      stats.sessionStartTime = millis();
      stats.sessionStringsTuned = 0;
      drawStandbyScreen();
      Serial.println("System ON");
    } else if (currentState == STATE_STATISTICS) {
      currentState = STATE_STANDBY;
      drawStandbyScreen();
    } else if (currentState == STATE_SETTINGS) {
      currentState = STATE_STANDBY;
      drawStandbyScreen();
    } else {
      // Long press for power off - check if held for 2 seconds
      unsigned long pressStart = millis();
      while (digitalRead(BTN_POWER) == LOW && millis() - pressStart < 2000) {
        delay(10);
      }
      if (millis() - pressStart >= 2000) {
        currentState = STATE_OFF;
        tft.fillScreen(ILI9341_BLACK);
        if (servoAttached) {
          servoPos = 90;
          tunerServo.write(servoPos);
          delay(200);
          tunerServo.detach();
          servoAttached = false;
        }
        Serial.println("System OFF");
      } else {
        // Short press - go to statistics
        currentState = STATE_STATISTICS;
        drawStatisticsScreen();
      }
    }
  }
  
  if (currentState == STATE_OFF) return;
  
  // Start/Stop button
  if (readButton(1, BTN_START_STOP)) {
    if (currentState == STATE_TUNING || currentState == STATE_AUTO_TUNE_ALL) {
      currentState = STATE_STANDBY;
      autoTuneInProgress = false;
      servoPos = 90;
      targetServoPos = 90;
      if (servoAttached) {
        tunerServo.write(servoPos);
        delay(200);
        tunerServo.detach();
        servoAttached = false;
      }
      
      // Record tuning time
      if (currentTuneStartTime > 0) {
        unsigned long tuneTime = millis() - currentTuneStartTime;
        stats.totalTuningTime += tuneTime;
        stats.avgTuningTime = (stats.totalTuningTime / 1000.0f) / (stats.totalStringsTuned > 0 ? stats.totalStringsTuned : 1);
      }
      
      drawStandbyScreen();
      Serial.println("Tuning stopped");
    } else if (currentState == STATE_STANDBY) {
      // Check if long press for AUTO TUNE ALL
      unsigned long pressStart = millis();
      while (digitalRead(BTN_START_STOP) == LOW && millis() - pressStart < 1000) {
        delay(10);
      }
      
      if (millis() - pressStart >= 1000) {
        // Long press - Auto tune all
        currentState = STATE_AUTO_TUNE_ALL;
        autoTuneInProgress = true;
        autoTuneCurrentString = 0;
        autoTuneStringStartTime = millis();
        currentTuneStartTime = millis();
        stableCount = 0;
        if (!servoAttached) {
          tunerServo.attach(SERVO_PIN, 500, 2500);
          servoAttached = true;
        }
        drawAutoTuneAllScreen();
        Serial.println("AUTO TUNE ALL started");
      } else {
        // Short press - normal tuning
        currentState = STATE_TUNING;
        drawTuningScreen();
        stableCount = 0;
        currentTuneStartTime = millis();
        if (!servoAttached) {
          tunerServo.attach(SERVO_PIN, 500, 2500);
          servoAttached = true;
        }
        Serial.println("Tuning started");
      }
    } else {
      currentState = STATE_STANDBY;
      drawStandbyScreen();
    }
  }
  
  // String select button
  if (readButton(2, BTN_STRING_SEL)) {
    if (currentState == STATE_STRING_SELECT) {
      if (isAutoMode) {
        isAutoMode = false;
        selectedString = 0;
      } else {
        selectedString++;
        if (selectedString > 5) {
          isAutoMode = true;
          selectedString = -1;
          currentState = STATE_STANDBY;
          drawStandbyScreen();
          Serial.println("String selection complete");
          return;
        }
      }
      drawStringSelectScreen();
      Serial.printf("String select: %s\n", isAutoMode ? "AUTO" : STRING_NAMES[selectedString]);
    } else if (currentState == STATE_STATISTICS) {
      currentState = STATE_STANDBY;
      drawStandbyScreen();
    } else {
      currentState = STATE_STRING_SELECT;
      drawStringSelectScreen();
    }
  }
  
  // Tuning mode button
  if (readButton(3, BTN_TUNING_MODE)) {
    if (currentState == STATE_MODE_SELECT) {
      tuningMode = (TuningMode)((tuningMode + 1) % 4);
      drawModeSelectScreen();
      Serial.printf("Tuning mode: %s\n", tuningModes[tuningMode].name);
    } else if (currentState == STATE_STATISTICS) {
      currentState = STATE_STANDBY;
      drawStandbyScreen();
    } else if (currentState == STATE_SETTINGS) {
      currentState = STATE_STANDBY;
      drawStandbyScreen();
    } else {
      // Check for long press to enter settings
      unsigned long pressStart = millis();
      while (digitalRead(BTN_TUNING_MODE) == LOW && millis() - pressStart < 1500) {
        delay(10);
      }
      
      if (millis() - pressStart >= 1500) {
        currentState = STATE_SETTINGS;
        drawSettingsScreen();
        Serial.println("Entered settings");
      } else {
        currentState = STATE_MODE_SELECT;
        drawModeSelectScreen();
      }
    }
  }
}

// ===== AUDIO PROCESSING (MODIFIED FOR PIEZO) =====

void readSinglePiezoValue() {
  unsigned long currentTime = millis();
  
  if (currentTime - lastReadTime >= READ_INTERVAL) {
    // Read single value from piezo
    int adcValue = analogRead(PIEZO_PIN);
    
    // Convert to voltage (assuming 12-bit ADC and 3.3V reference)
    float voltage = (adcValue / 4095.0) * 3.3;
    
    // Print the reading
    Serial.println("═══════════════════════════════════");
    Serial.print("Piezo ADC Value: ");
    Serial.print(adcValue);
    Serial.print(" / 4095");
    Serial.print(" (");
    Serial.print(voltage, 3);
    Serial.println(" V)");
    Serial.println("═══════════════════════════════════");
    
    lastReadTime = currentTime;
  }
}

void captureSamples() {
  unsigned long period = 1000000UL / (unsigned long)SAMPLING_FREQ;
  unsigned long startTime = micros();
  
  for (int i = 0; i < SAMPLES; i++) {
    unsigned long tTarget = startTime + (i * period);
    while (micros() < tTarget) { }
    
    // Read from piezo sensor
    int adcValue = analogRead(PIEZO_PIN);
    
    // Apply gain adjustment if needed
    vReal[i] = (double)adcValue * PIEZO_GAIN_ADJUST;
    vImag[i] = 0.0;
  }
}

void preprocessSignal() {
  // Calculate mean for DC offset removal
  double mean = 0;
  for (int i = 0; i < SAMPLES; i++) {
    mean += vReal[i];
  }
  mean /= SAMPLES;

  // Remove DC offset and apply window function
  signalLevel = 0;
  for (int i = 0; i < SAMPLES; i++) {
    // DC blocking (especially important for piezo sensors)
    if (USE_DC_BLOCK) {
      vReal[i] -= mean;
    }
    
    // Hamming window for better frequency resolution
    double w = 0.54 - 0.46 * cos(2.0 * PI * i / (SAMPLES - 1));
    vReal[i] *= w;
    
    // Calculate signal level
    signalLevel += fabs(vReal[i]);
  }
  signalLevel /= SAMPLES;
  
  // Debug: Print signal level periodically
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 1000) {
    Serial.print("Signal level: ");
    Serial.println(signalLevel);
    lastPrint = millis();
  }
}

float findPeakFrequency() {
  // Perform FFT
  FFT.compute(vReal, vImag, SAMPLES, FFT_FORWARD);
  FFT.complexToMagnitude(vReal, vImag, SAMPLES);

  // Calculate frequency resolution
  double df = SAMPLING_FREQ / SAMPLES;
  int minBin = (int)(F_MIN / df);
  int maxBin = (int)(F_MAX / df);
  if (maxBin >= SAMPLES/2) maxBin = SAMPLES/2 - 1;

  // Find peak in frequency range
  int peakBin = minBin;
  double peakMag = 0;
  for (int i = minBin; i <= maxBin; i++) {
    if (vReal[i] > peakMag) {
      peakMag = vReal[i];
      peakBin = i;
    }
  }
  
  // Check if signal is strong enough
  if (peakMag < NOISE_THRESHOLD) {
    return 0.0f;
  }

  // Parabolic interpolation for better frequency accuracy
  if (peakBin > minBin && peakBin < maxBin) {
    double y1 = vReal[peakBin - 1];
    double y2 = vReal[peakBin];
    double y3 = vReal[peakBin + 1];
    double d = (y1 - y3) / (2.0 * (y1 - 2.0 * y2 + y3));
    double bin = peakBin + d;
    
    float freq = bin * df;
    
    // Debug: Print detected frequency periodically
    static unsigned long lastFreqPrint = 0;
    if (millis() - lastFreqPrint > 500 && freq > F_MIN) {
      Serial.print("Detected: ");
      Serial.print(freq);
      Serial.print(" Hz (mag: ");
      Serial.print(peakMag);
      Serial.println(")");
      lastFreqPrint = millis();
    }
    
    return freq;
  }
  
  return peakBin * df;
}

float smoothFreq(float f) {
  if (f <= 0) return 0.0f;
  freqHistory[histIdx] = f;
  histIdx = (histIdx + 1) % SMOOTH_WINDOW;
  float sum = 0; int c = 0;
  for (int i = 0; i < SMOOTH_WINDOW; i++) {
    if (freqHistory[i] > 0) { sum += freqHistory[i]; c++; }
  }
  return c ? sum / c : 0.0f;
}

int identifyString(float f) {
  if (f <= 0) return -1;
  
  // Auto tune all mode - lock to current string
  if (currentState == STATE_AUTO_TUNE_ALL && autoTuneInProgress) {
    return autoTuneCurrentString;
  }
  
  // Manual mode
  if (!isAutoMode) return selectedString;
  
  // Auto-detect
  int best = -1; 
  float bestDiff = 1e9;
  float* targetFreqs = tuningModes[tuningMode].freqs;
  
  for (int i = 0; i < 6; i++) {
    float diff = fabsf(f - targetFreqs[i]);
    // Increased tolerance window for better string detection
    if (diff < targetFreqs[i] * 0.26f && diff < bestDiff) {
      bestDiff = diff; 
      best = i;
    }
  }
  return best;
}

void freqToNote(float f, String &name, int &cents) {
  if (f <= 0) { name = "--"; cents = 0; return; }
  float midi = 69.0f + 12.0f * log2f(f / 440.0f);
  int noteNum = roundf(midi);
  int idx = noteNum % 12; if (idx < 0) idx += 12;
  name = NOTE_NAMES[idx];
  
  int stringNum = identifyString(f);
  if (stringNum >= 0) {
    float targetFreq = tuningModes[tuningMode].freqs[stringNum];
    cents = roundf(1200.0f * log2f(f / targetFreq));
  } else {
    float fNote = 440.0f * powf(2.0f, (noteNum - 69) / 12.0f);
    cents = roundf(1200.0f * log2f(f / fNote));
  }
}

// ===== SERVO CONTROL =====

void attachServoIfNeeded() {
  if (!servoAttached) {
    tunerServo.attach(SERVO_PIN, 500, 2500);
    servoAttached = true;
    delay(50);
  }
}

void detachServoIfNeeded() {
  if (servoAttached && currentState != STATE_TUNING && currentState != STATE_AUTO_TUNE_ALL) {
    tunerServo.detach();
    servoAttached = false;
  }
}

void updateServoFromCents(int cents) {
  if (currentState != STATE_TUNING && currentState != STATE_AUTO_TUNE_ALL) return;
  
  unsigned long now = millis();
  
  // Check if in tune - SIMPLIFIED LOGIC
  if (abs(cents) <= TUNE_TOLERANCE) {
    stableCount++;  // Just increment if within tolerance
    
    if (stableCount >= TUNE_STABLE_COUNT) {
      servoIsMoving = false;
      
      // Update statistics
      stats.totalStringsTuned++;
      stats.sessionStringsTuned++;
      stats.successfulTunes++;
      
      if (currentTuneStartTime > 0) {
        unsigned long tuneTime = millis() - currentTuneStartTime;
        stats.totalTuningTime += tuneTime;
        stats.avgTuningTime = (stats.totalTuningTime / 1000.0f) / stats.totalStringsTuned;
        currentTuneStartTime = 0;
      }
      
      // Show success animation
      showSuccessAnimation = true;
      successAnimationFrame = 0;
      successAnimationStartTime = millis();  // Start the timer
      
      Serial.println("✓ IN TUNE!");
      
      // LOCK THE SERVO - don't allow further movement
      targetServoPos = servoPos;
      lastCents = cents;
      
      return;  // Exit early - let the main loop handle the animation
    }
  } else {
    stableCount = 0;  // Reset only when OUT of tolerance
  }
  
  // Check timeout for auto tune all
  if (currentState == STATE_AUTO_TUNE_ALL && millis() - autoTuneStringStartTime > AUTO_TUNE_TIMEOUT) {
    Serial.println("String tuning timeout - skipping");
    stats.failedTunes++;
    autoTuneCurrentString++;
    stableCount = 0;
    
    if (autoTuneCurrentString >= 6) {
      currentState = STATE_STANDBY;
      autoTuneInProgress = false;
      drawStandbyScreen();
    } else {
      autoTuneStringStartTime = millis();
      currentTuneStartTime = millis();
      drawAutoTuneAllScreen();
    }
    return;
  }
  
  // Check if servo is moving
  if (servoIsMoving) {
    if (now - servoMoveStartTime < SERVO_MOVE_DURATION) {
      return;
    } else {
      servoIsMoving = false;
    }
  }
  
  // Rate limiting
  if (now - lastServoMove < SERVO_MOVE_PERIOD) return;
  
  // Calculate step
  int step = 1;
  int absCents = abs(cents);
  
  if (absCents > 30) step = 5;
  else if (absCents > 20) step = 3;
  else if (absCents > 10) step = 2;
  else step = 1;
  
  // Move servo
  if (cents < 0) {
    targetServoPos = servoPos + step;
  } else {
    targetServoPos = servoPos - step;
  }
  
  targetServoPos = constrain(targetServoPos, 0, 180);
  
  if (targetServoPos != servoPos) {
    attachServoIfNeeded();
    tunerServo.write(targetServoPos);
    servoPos = targetServoPos;
    servoIsMoving = true;
    servoMoveStartTime = now;
    lastServoMove = now;
  }
  
  lastCents = cents;
}

void checkSuccessAnimationComplete() {
  // Check if animation has been showing for 3 seconds
  if (showSuccessAnimation && (millis() - successAnimationStartTime >= SUCCESS_DISPLAY_TIME)) {
    showSuccessAnimation = false;
    successAnimationFrame = 0;
    
    // Move to next string based on mode
    if (currentState == STATE_AUTO_TUNE_ALL) {
      autoTuneCurrentString++;
      stableCount = 0;
      
      if (autoTuneCurrentString >= 6) {
        // All strings done!
        currentState = STATE_STANDBY;
        autoTuneInProgress = false;
        if (servoAttached) {
          servoPos = 90;
          tunerServo.write(servoPos);
          delay(200);
          tunerServo.detach();
          servoAttached = false;
        }
        drawStandbyScreen();
        Serial.println("AUTO TUNE ALL COMPLETE!");
      } else {
        autoTuneStringStartTime = millis();
        currentTuneStartTime = millis();
        drawAutoTuneAllScreen();
        Serial.printf("Moving to next string: %s\n", STRING_NAMES[autoTuneCurrentString]);
      }
    } else if (currentState == STATE_TUNING) {
      // Manual tuning mode
      if (!isAutoMode && selectedString < 5) {
        selectedString++;
        stableCount = 0;
        currentTuneStartTime = millis();
        drawTuningScreen();
        Serial.printf("Moving to next string: %s\n", STRING_NAMES[selectedString]);
      } else if (!isAutoMode && selectedString >= 5) {
        // Last string done
        currentState = STATE_STANDBY;
        if (servoAttached) {
          servoPos = 90;
          tunerServo.write(servoPos);
          delay(200);
          tunerServo.detach();
          servoAttached = false;
        }
        drawStandbyScreen();
        Serial.println("Manual tuning complete!");
      } else {
        // Auto mode or done - just redraw
        drawTuningScreen();
      }
    }
  }
}

// ===== SETUP =====

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n\n╔════════════════════════════════════════╗");
  Serial.println("║  AUTO GUITAR TUNER - PIEZO v3.0        ║");
  Serial.println("╚════════════════════════════════════════╝\n");

  vReal = (double*)malloc(SAMPLES * sizeof(double));
  vImag = (double*)malloc(SAMPLES * sizeof(double));
  if (!vReal || !vImag) {
    Serial.println("❌ ERROR: Memory allocation failed");
    while (1) delay(1000);
  }
  Serial.println("✓ Memory allocated");

  // Configure ADC for piezo sensor
  analogReadResolution(12);  // 12-bit resolution (0-4095)
  analogSetAttenuation(ADC_11db);  // Full range 0-3.3V
  Serial.println("✓ ADC configured for piezo");

  SPI.begin();
  delay(100);
  tft.begin();
  tft.setRotation(3);
  tft.fillScreen(COLOR_BG);
  Serial.println("✓ Display initialized");

  initButtons();
  Serial.println("✓ Buttons initialized");

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  tunerServo.setPeriodHertz(50);
  Serial.println("✓ Servo configured");

  // Initialize statistics
  stats.sessionStartTime = millis();
  
  drawStandbyScreen();
  
  Serial.println("\n╔════════════════════════════════════════╗");
  Serial.println("║           FEATURES ENABLED             ║");
  Serial.println("╠════════════════════════════════════════╣");
  Serial.println("║ ✓ Piezo Sensor Input                   ║");
  Serial.println("║ ✓ Manual Tuning                        ║");
  Serial.println("║ ✓ Auto Tune All Strings               ║");
  Serial.println("║ ✓ Frequency History Graph              ║");
  Serial.println("║ ✓ Success Animations                   ║");
  Serial.println("║ ✓ Session Statistics                   ║");
  Serial.println("║ ✓ Battery Indicator                    ║");
  Serial.println("║ ✓ Settings Menu                        ║");
  Serial.println("║ ✓ 4 Tuning Modes                       ║");
  Serial.println("╚════════════════════════════════════════╝\n");
  
  Serial.println("BUTTON GUIDE:");
  Serial.println("  • POWER: Stats (short) / Off (long 2s)");
  Serial.println("  • START: Tune (short) / Auto All (long 1s)");
  Serial.println("  • STRING SEL: Choose string mode");
  Serial.println("  • MODE: Select mode / Settings (long 1.5s)");
  Serial.println("\n🎸 Ready to tune with PIEZO sensor!\n");
  Serial.println("NOTE: Pluck strings directly for best results");
  Serial.println("Monitoring signal level...\n");
}

// ===== MAIN LOOP =====

void loop() {
  readSinglePiezoValue();

  handleButtons();
  
  // Battery simulation (slowly decrease)
  static unsigned long lastBatteryUpdate = 0;
  if (millis() - lastBatteryUpdate > 60000) {  // Every minute
    if (batteryLevel > 0) batteryLevel--;
    lastBatteryUpdate = millis();
  }
  
  if (currentState == STATE_TUNING || currentState == STATE_AUTO_TUNE_ALL) {
    attachServoIfNeeded();
    
    // Check if success animation should complete
    checkSuccessAnimationComplete();
    
    // Don't process audio if showing success animation
    if (!showSuccessAnimation) {
      captureSamples();
      preprocessSignal();

      float freq = 0.0f;
      String note = "--";
      int cents = 0;

      // Lower threshold for piezo - they tend to have better SNR
      if (signalLevel > 5.0f) {
        float raw = findPeakFrequency();
        freq = smoothFreq(raw);
        freqToNote(freq, note, cents);
        
        int stringNum = identifyString(freq);
        if (stringNum >= 0 && freq > 0) {
          updateServoFromCents(cents);
        }
      }

      if (currentState == STATE_TUNING) {
        updateTuningScreen(freq, note, cents);
      } else if (currentState == STATE_AUTO_TUNE_ALL) {
        updateAutoTuneAllScreen(freq, cents);
      }
    }
    
    // Draw success animation if active (outside the audio processing block)
    if (showSuccessAnimation) {
      drawSuccessAnimation();
    }
    
    delay(20);
    yield();
  } else {
    detachServoIfNeeded();
    delay(20);
    yield();
  }
}
