#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>  // Changed from Adafruit_ILI9341.h
#include <SPI.h>
#include <arduinoFFT.h>
#include <ESP32Servo.h>

// ===== TFT DISPLAY (PCB PINOUT - ST7789 Waveshare 2.4") =====
#define TFT_MOSI  11  // D_IN / SDA
#define TFT_CLK   12  // CLK / SCL
#define TFT_CS    8   // CS
#define TFT_DC    7   // DC
#define TFT_RST   6   // RST
#define TFT_BL    10  // Backlight

// ST7789 constructor - note: 240x320 are the native dimensions
// We'll use rotation to get 320x240 landscape
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

// ===== BUTTONS (PCB PINOUT) =====
#define BTN_TOGGLE    15  // IO3 - Main action button
#define BTN_SELECT    46  // IO46 - Selection/navigation button

// Button debouncing and press detection
unsigned long lastDebounceTime[2] = {0, 0};
const unsigned long debounceDelay = 50;
bool lastButtonState[2] = {HIGH, HIGH};
bool buttonState[2] = {HIGH, HIGH};
unsigned long buttonPressStart[2] = {0, 0};
bool buttonLongPressTriggered[2] = {false, false};

// ===== PIEZO SENSOR (PCB PINOUT) =====
const int PIEZO_PIN = 2;  // IO2 - ADC input from amplifier

// ===== SERVO MOTOR (PCB PINOUT) =====
const int SERVO_PIN = 45;  // IO45 - PWM signal to motor

// ===== FFT Configuration =====
const uint16_t SAMPLES = 2048;
const double SAMPLING_FREQ = 8192;
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
const unsigned long AUTO_TUNE_TIMEOUT = 30000;

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

// Analysis settings
const float F_MIN = 70.0f;
const float F_MAX = 1000.0f;
const float NOISE_THRESHOLD = 15.0;
int TUNE_TOLERANCE = 5;

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

unsigned long lastReadTime = 0;
const unsigned long READ_INTERVAL = 3000;

// ===== STATISTICS =====
struct Statistics {
  int totalStringsTuned;
  int sessionStringsTuned;
  unsigned long totalTuningTime;
  unsigned long sessionStartTime;
  int successfulTunes;
  int failedTunes;
  float avgTuningTime;
} stats = {0, 0, 0, 0, 0, 0, 0.0};

unsigned long currentTuneStartTime = 0;

// ===== SERVO =====
Servo tunerServo;
int servoPos = 90;
int targetServoPos = 90;
unsigned long lastServoMove = 0;
uint32_t SERVO_MOVE_PERIOD = 150;
bool servoAttached = false;

int TUNE_STABLE_COUNT = 5;
int stableCount = 0;
int lastCents = 0;

bool servoIsMoving = false;
unsigned long servoMoveStartTime = 0;
const uint32_t SERVO_MOVE_DURATION = 120;

// ===== ANIMATION =====
int animationFrame = 0;
unsigned long lastAnimationTime = 0;
bool showSuccessAnimation = false;
int successAnimationFrame = 0;
unsigned long successAnimationStartTime = 0;
const unsigned long SUCCESS_DISPLAY_TIME = 3000;

// ===== BATTERY SIMULATION =====
int batteryLevel = 100;

// ===== COLOR SCHEME =====
// ST7789 uses the same 16-bit RGB565 format as ILI9341
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
float PIEZO_GAIN_ADJUST = 1.0;
bool USE_DC_BLOCK = true;

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
  tft.drawRect(x, y + 2, 20, 10, COLOR_TEXT_DIM);
  tft.fillRect(x + 20, y + 4, 2, 6, COLOR_TEXT_DIM);
  
  uint16_t fillColor = COLOR_SUCCESS;
  if (batteryLevel < 20) fillColor = COLOR_DANGER;
  else if (batteryLevel < 50) fillColor = COLOR_WARNING;
  
  int fillWidth = map(batteryLevel, 0, 100, 0, 16);
  if (fillWidth > 0) {
    tft.fillRect(x + 2, y + 4, fillWidth, 6, fillColor);
  }
  
  tft.setTextSize(1);
  tft.setTextColor(COLOR_TEXT_DIM);
  tft.setCursor(x + 24, y + 3);
  tft.print(batteryLevel);
  tft.print("%");
}

void drawFrequencyGraph(int x, int y, int w, int h, float targetFreq) {
  tft.fillRect(x, y, w, h, COLOR_BG);
  tft.drawRect(x, y, w, h, COLOR_TEXT_DIM);
  
  int targetY = y + h/2;
  tft.drawFastHLine(x, targetY, w, COLOR_SUCCESS);
  
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
  
  int centerX = 160;
  int centerY = 120;
  int size = 30 + (successAnimationFrame % 10);
  
  tft.fillCircle(centerX, centerY, size, COLOR_SUCCESS);
  
  tft.drawLine(centerX - 10, centerY, centerX - 3, centerY + 10, COLOR_TEXT);
  tft.drawLine(centerX - 3, centerY + 10, centerX + 12, centerY - 10, COLOR_TEXT);
  tft.drawLine(centerX - 9, centerY, centerX - 3, centerY + 9, COLOR_TEXT);
  tft.drawLine(centerX - 3, centerY + 9, centerX + 11, centerY - 10, COLOR_TEXT);
  
  for (int i = 0; i < 4; i++) {
    int angle = (successAnimationFrame * 10 + i * 90) % 360;
    int sx = centerX + cos(angle * PI / 180) * (40 + successAnimationFrame);
    int sy = centerY + sin(angle * PI / 180) * (40 + successAnimationFrame);
    tft.fillCircle(sx, sy, 3, COLOR_GOLD);
  }
  
  successAnimationFrame++;
  if (successAnimationFrame > 60) {
    successAnimationFrame = 0;
  }
}

// ===== UI SCREENS =====

void drawStandbyScreen() {
  tft.fillScreen(COLOR_BG);
  
  drawBatteryIcon(250, 5);
  
  drawCard(10, 10, 300, 60);
  tft.setTextSize(3);
  tft.setTextColor(COLOR_PRIMARY);
  tft.setCursor(50, 25);
  tft.print("GUITAR TUNER");
  
  drawCard(10, 80, 300, 50);
  tft.setTextSize(2);
  tft.setTextColor(COLOR_TEXT_DIM);
  tft.setCursor(20, 88);
  tft.print("Status:");
  tft.setTextSize(2);
  tft.setTextColor(COLOR_WARNING);
  tft.setCursor(110, 88);
  tft.print("READY");
  
  tft.setTextSize(1);
  tft.setTextColor(COLOR_ACCENT);
  tft.setCursor(20, 108);
  tft.print("Session: ");
  tft.print(stats.sessionStringsTuned);
  tft.print(" strings");
  
  drawCard(10, 140, 145, 38);
  tft.setTextSize(1);
  tft.setTextColor(COLOR_TEXT_DIM);
  tft.setCursor(20, 148);
  tft.print("MODE");
  tft.setTextSize(1);
  tft.setTextColor(COLOR_ACCENT);
  tft.setCursor(20, 160);
  tft.print(tuningModes[tuningMode].name);
  
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
  
  drawCard(10, 188, 300, 40);
  tft.setTextSize(1);
  tft.setTextColor(COLOR_TEXT_DIM);
  tft.setCursor(20, 195);
  tft.print("TOGGLE: Start  |  SELECT: String/Mode");
  tft.setTextSize(1);
  tft.setTextColor(COLOR_PURPLE);
  tft.setCursor(20, 210);
  tft.print("Long TOGGLE: Auto All | Long SELECT: Mode");
}

void drawTuningScreen() {
  tft.fillScreen(COLOR_BG);
  
  drawBatteryIcon(250, 5);
  
  drawCard(10, 5, 230, 30);
  tft.setTextSize(2);
  tft.setTextColor(COLOR_SUCCESS);
  tft.setCursor(20, 12);
  tft.print("TUNING");
  
  tft.setTextSize(1);
  tft.setTextColor(COLOR_TEXT_DIM);
  tft.setCursor(150, 12);
  tft.print(tuningModes[tuningMode].name);
  
  drawCard(10, 40, 300, 28);
  tft.setTextSize(1);
  tft.setTextColor(COLOR_TEXT_DIM);
  tft.setCursor(20, 47);
  tft.print("SIGNAL");
  
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
  
  tft.setTextSize(1);
  tft.setTextColor(COLOR_TEXT_DIM);
  tft.setCursor(20, 118);
  tft.print("TUNING PROGRESS");
  
  tft.setTextSize(1);
  tft.setTextColor(COLOR_TEXT_DIM);
  tft.setCursor(20, 173);
  tft.print("ACCURACY");
}

void drawAutoTuneAllScreen() {
  tft.fillScreen(COLOR_BG);
  
  drawBatteryIcon(250, 5);
  
  drawCard(10, 10, 300, 45);
  tft.setTextSize(3);
  tft.setTextColor(COLOR_PURPLE);
  tft.setCursor(30, 20);
  tft.print("AUTO TUNE");
  
  drawCard(10, 65, 300, 60);
  tft.setTextSize(2);
  tft.setTextColor(COLOR_TEXT);
  tft.setCursor(20, 75);
  tft.print("Progress:");
  
  int yStart = 95;
  for (int i = 0; i < 6; i++) {
    int x = 20 + (i % 3) * 95;
    int y = yStart + (i / 3) * 25;
    
    uint16_t color = COLOR_TEXT_DIM;
    if (i < autoTuneCurrentString) color = COLOR_SUCCESS;
    else if (i == autoTuneCurrentString) color = COLOR_WARNING;
    
    tft.setTextSize(2);
    tft.setTextColor(color);
    tft.setCursor(x, y);
    tft.print(STRING_NAMES[i]);
    
    if (i < autoTuneCurrentString) {
      tft.print(" ");
      tft.setTextColor(COLOR_SUCCESS);
      tft.print((char)251);
    } else if (i == autoTuneCurrentString) {
      tft.print(" ...");
    }
  }
  
  tft.setTextSize(1);
  tft.setTextColor(COLOR_TEXT_DIM);
  tft.setCursor(20, 150);
  tft.print("Overall Progress:");
  drawProgressBar(20, 163, 280, 20, autoTuneCurrentString, 6, COLOR_PURPLE);
  
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
  tft.setCursor(20, 228);
  tft.print("SELECT to cycle | TOGGLE to confirm");
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
  
  tft.setTextSize(1);
  tft.setTextColor(COLOR_TEXT_DIM);
  tft.setCursor(20, 228);
  tft.print("SELECT to cycle | TOGGLE to confirm");
}

void drawStatisticsScreen() {
  tft.fillScreen(COLOR_BG);
  
  drawBatteryIcon(250, 5);
  
  drawCard(10, 10, 300, 50);
  tft.setTextSize(3);
  tft.setTextColor(COLOR_GOLD);
  tft.setCursor(40, 22);
  tft.print("STATISTICS");
  
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
  tft.print("Long press SELECT to adjust");
}

void updateTuningScreen(float freq, const String& note, int cents) {
  int signalPercent = constrain(map(signalLevel, 0, 100, 0, 100), 0, 100);
  uint16_t signalColor = (signalLevel > NOISE_THRESHOLD) ? COLOR_SUCCESS : COLOR_DANGER;
  
  tft.fillRect(100, 47, 200, 15, COLOR_CARD);
  drawProgressBar(100, 47, 200, 15, signalPercent, 100, signalColor);
  
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
  
  tft.fillRect(175, 93, 125, 18, COLOR_CARD);
  tft.setTextSize(2);
  tft.setTextColor(COLOR_WARNING);
  tft.setCursor(200, 95);
  tft.print(note);
  
  if (freq > 0) {
    addToFrequencyHistory(freq);
    int stringNum = identifyString(freq);
    if (stringNum >= 0) {
      float targetFreq = tuningModes[tuningMode].freqs[stringNum];
      drawFrequencyGraph(10, 128, 300, 40, targetFreq);
    }
  }
  
  drawCentsMeter(10, 183, 300, 35, cents);
  
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

// ===== BUTTON HANDLING (2-BUTTON SYSTEM) =====

void initButtons() {
  pinMode(BTN_TOGGLE, INPUT_PULLUP);
  pinMode(BTN_SELECT, INPUT_PULLUP);
}

// Enhanced button reading with long press detection
bool readButton(int buttonIndex, int pin, bool &longPress, bool &veryLongPress) {
  bool reading = digitalRead(pin);
  longPress = false;
  veryLongPress = false;
  
  if (reading != lastButtonState[buttonIndex]) {
    lastDebounceTime[buttonIndex] = millis();
    if (reading == LOW) {
      buttonPressStart[buttonIndex] = millis();
      buttonLongPressTriggered[buttonIndex] = false;
    }
  }
  
  bool pressed = false;
  unsigned long pressDuration = 0;
  
  if ((millis() - lastDebounceTime[buttonIndex]) > debounceDelay) {
    if (reading != buttonState[buttonIndex]) {
      buttonState[buttonIndex] = reading;
      
      if (buttonState[buttonIndex] == HIGH && lastButtonState[buttonIndex] == LOW) {
        pressDuration = millis() - buttonPressStart[buttonIndex];
        
        if (pressDuration >= 2000 && !buttonLongPressTriggered[buttonIndex]) {
          veryLongPress = true;
          pressed = true;
        } else if (pressDuration >= 800 && !buttonLongPressTriggered[buttonIndex]) {
          longPress = true;
          pressed = true;
        } else if (pressDuration < 800) {
          pressed = true;
        }
      }
    }
    
    if (buttonState[buttonIndex] == LOW) {
      pressDuration = millis() - buttonPressStart[buttonIndex];
      
      if (pressDuration >= 2000 && !buttonLongPressTriggered[buttonIndex]) {
        veryLongPress = true;
        buttonLongPressTriggered[buttonIndex] = true;
        pressed = true;
      } else if (pressDuration >= 800 && !buttonLongPressTriggered[buttonIndex]) {
        longPress = true;
        buttonLongPressTriggered[buttonIndex] = true;
        pressed = true;
      }
    }
  }
  
  lastButtonState[buttonIndex] = reading;
  return pressed;
}

void handleButtons() {
  bool toggleLongPress = false;
  bool toggleVeryLongPress = false;
  bool selectLongPress = false;
  bool selectVeryLongPress = false;
  
  bool togglePressed = readButton(0, BTN_TOGGLE, toggleLongPress, toggleVeryLongPress);
  bool selectPressed = readButton(1, BTN_SELECT, selectLongPress, selectVeryLongPress);
  
  // TOGGLE BUTTON LOGIC
  if (togglePressed) {
    if (toggleVeryLongPress) {
      // Very long press (2s+) - Power off from any state
      currentState = STATE_OFF;
      tft.fillScreen(ST77XX_BLACK);  // Changed from ILI9341_BLACK
      if (servoAttached) {
        servoPos = 90;
        tunerServo.write(servoPos);
        delay(200);
        tunerServo.detach();
        servoAttached = false;
      }
      Serial.println("System OFF");
      
    } else if (toggleLongPress) {
      // Long press (800ms) - Auto tune all (from standby only)
      if (currentState == STATE_STANDBY) {
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
      }
      
    } else {
      // Short press - Main action button
      if (currentState == STATE_OFF) {
        // Power on
        currentState = STATE_STANDBY;
        stats.sessionStartTime = millis();
        stats.sessionStringsTuned = 0;
        drawStandbyScreen();
        Serial.println("System ON");
        
      } else if (currentState == STATE_STANDBY) {
        // Start tuning
        currentState = STATE_TUNING;
        drawTuningScreen();
        stableCount = 0;
        currentTuneStartTime = millis();
        if (!servoAttached) {
          tunerServo.attach(SERVO_PIN, 500, 2500);
          servoAttached = true;
        }
        Serial.println("Tuning started");
        
      } else if (currentState == STATE_TUNING || currentState == STATE_AUTO_TUNE_ALL) {
        // Stop tuning
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
        
        if (currentTuneStartTime > 0) {
          unsigned long tuneTime = millis() - currentTuneStartTime;
          stats.totalTuningTime += tuneTime;
          stats.avgTuningTime = (stats.totalTuningTime / 1000.0f) / (stats.totalStringsTuned > 0 ? stats.totalStringsTuned : 1);
        }
        
        drawStandbyScreen();
        Serial.println("Tuning stopped");
        
      } else if (currentState == STATE_STRING_SELECT) {
        // Confirm string selection
        currentState = STATE_STANDBY;
        drawStandbyScreen();
        Serial.println("String selection confirmed");
        
      } else if (currentState == STATE_MODE_SELECT) {
        // Confirm mode selection
        currentState = STATE_STANDBY;
        drawStandbyScreen();
        Serial.println("Mode selection confirmed");
        
      } else {
        // Return to standby from other screens
        currentState = STATE_STANDBY;
        drawStandbyScreen();
      }
    }
  }
  
  // SELECT BUTTON LOGIC
  if (selectPressed) {
    if (selectVeryLongPress) {
      // Very long press (2s+) - Settings
      if (currentState == STATE_STANDBY) {
        currentState = STATE_SETTINGS;
        drawSettingsScreen();
        Serial.println("Entered settings");
      }
      
    } else if (selectLongPress) {
      // Long press (800ms) - Mode select or Statistics
      if (currentState == STATE_STANDBY) {
        currentState = STATE_MODE_SELECT;
        drawModeSelectScreen();
        Serial.println("Mode selection");
      } else if (currentState == STATE_TUNING || currentState == STATE_AUTO_TUNE_ALL) {
        // Show statistics while tuning
        SystemState prevState = currentState;
        currentState = STATE_STATISTICS;
        drawStatisticsScreen();
      }
      
    } else {
      // Short press - String selection or cycle
      if (currentState == STATE_STANDBY) {
        currentState = STATE_STRING_SELECT;
        drawStringSelectScreen();
        Serial.println("String selection");
        
      } else if (currentState == STATE_STRING_SELECT) {
        // Cycle through strings
        if (isAutoMode) {
          isAutoMode = false;
          selectedString = 0;
        } else {
          selectedString++;
          if (selectedString > 5) {
            isAutoMode = true;
            selectedString = -1;
          }
        }
        drawStringSelectScreen();
        Serial.printf("String: %s\n", isAutoMode ? "AUTO" : STRING_NAMES[selectedString]);
        
      } else if (currentState == STATE_MODE_SELECT) {
        // Cycle through modes
        tuningMode = (TuningMode)((tuningMode + 1) % 4);
        drawModeSelectScreen();
        Serial.printf("Mode: %s\n", tuningModes[tuningMode].name);
        
      } else if (currentState == STATE_STATISTICS || currentState == STATE_SETTINGS) {
        // Return to standby
        currentState = STATE_STANDBY;
        drawStandbyScreen();
      }
    }
  }
}

// ===== AUDIO PROCESSING =====

void readSinglePiezoValue() {
  unsigned long currentTime = millis();
  
  if (currentTime - lastReadTime >= READ_INTERVAL) {
    int adcValue = analogRead(PIEZO_PIN);
    float voltage = (adcValue / 4095.0) * 3.3;
    
    Serial.println("═══════════════════════════════════");
    Serial.print("Piezo ADC: ");
    Serial.print(adcValue);
    Serial.print(" / 4095 (");
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
    
    int adcValue = analogRead(PIEZO_PIN);
    vReal[i] = (double)adcValue * PIEZO_GAIN_ADJUST;
    vImag[i] = 0.0;
  }
}

void preprocessSignal() {
  double mean = 0;
  for (int i = 0; i < SAMPLES; i++) {
    mean += vReal[i];
  }
  mean /= SAMPLES;

  signalLevel = 0;
  for (int i = 0; i < SAMPLES; i++) {
    if (USE_DC_BLOCK) {
      vReal[i] -= mean;
    }
    
    double w = 0.54 - 0.46 * cos(2.0 * PI * i / (SAMPLES - 1));
    vReal[i] *= w;
    
    signalLevel += fabs(vReal[i]);
  }
  signalLevel /= SAMPLES;
  
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 1000) {
    Serial.print("Signal: ");
    Serial.println(signalLevel);
    lastPrint = millis();
  }
}

float findPeakFrequency() {
  FFT.compute(vReal, vImag, SAMPLES, FFT_FORWARD);
  FFT.complexToMagnitude(vReal, vImag, SAMPLES);

  double df = SAMPLING_FREQ / SAMPLES;
  int minBin = (int)(F_MIN / df);
  int maxBin = (int)(F_MAX / df);
  if (maxBin >= SAMPLES/2) maxBin = SAMPLES/2 - 1;

  int peakBin = minBin;
  double peakMag = 0;
  for (int i = minBin; i <= maxBin; i++) {
    if (vReal[i] > peakMag) {
      peakMag = vReal[i];
      peakBin = i;
    }
  }
  
  if (peakMag < NOISE_THRESHOLD) {
    return 0.0f;
  }

  if (peakBin > minBin && peakBin < maxBin) {
    double y1 = vReal[peakBin - 1];
    double y2 = vReal[peakBin];
    double y3 = vReal[peakBin + 1];
    double d = (y1 - y3) / (2.0 * (y1 - 2.0 * y2 + y3));
    double bin = peakBin + d;
    
    float freq = bin * df;
    
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
  
  if (currentState == STATE_AUTO_TUNE_ALL && autoTuneInProgress) {
    return autoTuneCurrentString;
  }
  
  if (!isAutoMode) return selectedString;
  
  int best = -1; 
  float bestDiff = 1e9;
  float* targetFreqs = tuningModes[tuningMode].freqs;
  
  for (int i = 0; i < 6; i++) {
    float diff = fabsf(f - targetFreqs[i]);
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
  
  if (abs(cents) <= TUNE_TOLERANCE) {
    stableCount++;
    
    if (stableCount >= TUNE_STABLE_COUNT) {
      servoIsMoving = false;
      
      stats.totalStringsTuned++;
      stats.sessionStringsTuned++;
      stats.successfulTunes++;
      
      if (currentTuneStartTime > 0) {
        unsigned long tuneTime = millis() - currentTuneStartTime;
        stats.totalTuningTime += tuneTime;
        stats.avgTuningTime = (stats.totalTuningTime / 1000.0f) / stats.totalStringsTuned;
        currentTuneStartTime = 0;
      }
      
      showSuccessAnimation = true;
      successAnimationFrame = 0;
      successAnimationStartTime = millis();
      
      Serial.println("✓ IN TUNE!");
      
      targetServoPos = servoPos;
      lastCents = cents;
      
      return;
    }
  } else {
    stableCount = 0;
  }
  
  if (currentState == STATE_AUTO_TUNE_ALL && millis() - autoTuneStringStartTime > AUTO_TUNE_TIMEOUT) {
    Serial.println("Timeout - skipping");
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
  
  if (servoIsMoving) {
    if (now - servoMoveStartTime < SERVO_MOVE_DURATION) {
      return;
    } else {
      servoIsMoving = false;
    }
  }
  
  if (now - lastServoMove < SERVO_MOVE_PERIOD) return;
  
  int step = 1;
  int absCents = abs(cents);
  
  if (absCents > 30) step = 5;
  else if (absCents > 20) step = 3;
  else if (absCents > 10) step = 2;
  else step = 1;
  
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
  if (showSuccessAnimation && (millis() - successAnimationStartTime >= SUCCESS_DISPLAY_TIME)) {
    showSuccessAnimation = false;
    successAnimationFrame = 0;
    
    if (currentState == STATE_AUTO_TUNE_ALL) {
      autoTuneCurrentString++;
      stableCount = 0;
      
      if (autoTuneCurrentString >= 6) {
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
        Serial.printf("Next: %s\n", STRING_NAMES[autoTuneCurrentString]);
      }
    } else if (currentState == STATE_TUNING) {
      if (!isAutoMode && selectedString < 5) {
        selectedString++;
        stableCount = 0;
        currentTuneStartTime = millis();
        drawTuningScreen();
        Serial.printf("Next: %s\n", STRING_NAMES[selectedString]);
      } else if (!isAutoMode && selectedString >= 5) {
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
  Serial.println("║  AUTO GUITAR TUNER - ST7789 v1.0       ║");
  Serial.println("╚════════════════════════════════════════╝\n");

  vReal = (double*)malloc(SAMPLES * sizeof(double));
  vImag = (double*)malloc(SAMPLES * sizeof(double));
  if (!vReal || !vImag) {
    Serial.println("❌ ERROR: Memory allocation failed");
    while (1) delay(1000);
  }
  Serial.println("✓ Memory allocated");

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  Serial.println("✓ ADC configured (IO2)");

  // Configure SPI with custom pins for ST7789
  SPI.begin(TFT_CLK, -1, TFT_MOSI, TFT_CS);
  delay(100);
  
  // ST7789 initialization - Waveshare 2.4" is 240x320
  // init() takes width and height parameters
  tft.init(240, 320);
  
  // Set rotation for landscape mode (320x240)
  // You may need to adjust this (0-3) depending on your mounting orientation
  // Rotation 1 or 3 gives landscape; try both to see which orientation works
  tft.setRotation(3);
  
  tft.fillScreen(COLOR_BG);
  Serial.println("✓ ST7789 Display initialized (240x320)");
  
  // Enable backlight
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);
  Serial.println("✓ Backlight enabled");

  initButtons();
  Serial.println("✓ Buttons initialized (IO15, IO46)");

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  tunerServo.setPeriodHertz(50);
  Serial.println("✓ Servo configured (IO45)");

  stats.sessionStartTime = millis();
  
  drawStandbyScreen();
  
  Serial.println("\n╔════════════════════════════════════════╗");
  Serial.println("║         PCB CONFIGURATION              ║");
  Serial.println("╠════════════════════════════════════════╣");
  Serial.println("║ Display: ST7789 Waveshare 2.4\"         ║");
  Serial.println("║ Pins: IO11,12,8,7,6,10                 ║");
  Serial.println("║ Buttons: IO15 (Toggle), IO46 (Select)  ║");
  Serial.println("║ Servo: IO45                            ║");
  Serial.println("║ Piezo: IO2                             ║");
  Serial.println("╠════════════════════════════════════════╣");
  Serial.println("║ BUTTON GUIDE:                          ║");
  Serial.println("║ • TOGGLE: Start/Stop (short)           ║");
  Serial.println("║           Auto All (long 800ms)        ║");
  Serial.println("║           Power Off (very long 2s)     ║");
  Serial.println("║ • SELECT: String select (short)        ║");
  Serial.println("║           Mode select (long 800ms)     ║");
  Serial.println("║           Settings (very long 2s)      ║");
  Serial.println("╚════════════════════════════════════════╝\n");
  
  Serial.println("🎸 Ready to tune!\n");
}

// ===== MAIN LOOP =====

void loop() {
  readSinglePiezoValue();
  handleButtons();
  
  static unsigned long lastBatteryUpdate = 0;
  if (millis() - lastBatteryUpdate > 60000) {
    if (batteryLevel > 0) batteryLevel--;
    lastBatteryUpdate = millis();
  }
  
  if (currentState == STATE_TUNING || currentState == STATE_AUTO_TUNE_ALL) {
    attachServoIfNeeded();
    
    checkSuccessAnimationComplete();
    
    if (!showSuccessAnimation) {
      captureSamples();
      preprocessSignal();

      float freq = 0.0f;
      String note = "--";
      int cents = 0;

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
