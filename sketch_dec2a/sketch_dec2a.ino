#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>
#include <ESP32Servo.h>
#include <math.h>

// ===== TFT DISPLAY =====
#define TFT_MOSI  11
#define TFT_CLK   12
#define TFT_CS    8
#define TFT_DC    7
#define TFT_RST   6
#define TFT_BL    10

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

// ===== BUTTONS =====
#define BTN_TOGGLE    46
#define BTN_SELECT    3

bool lastButtonState[2] = {LOW, LOW};
unsigned long buttonPressStart[2] = {0, 0};
bool buttonLongPressTriggered[2] = {false, false};

// ===== PIEZO SENSOR =====
const int PIEZO_PIN = 2;

// ===== SERVO MOTOR =====
const int SERVO_PIN = 45;

// ===== AUTOCORRELATION CONFIG =====
const uint16_t SAMPLES = 1024;
const double SAMPLING_FREQ = 8192.0;
int16_t *sampleBuffer;

// Frequency range for guitar (E2=82Hz to E4=330Hz)
const float F_MIN = 75.0f;
const float F_MAX = 450.0f;
float NOISE_THRESHOLD = 4.0f;

// ===== PITCH TRACKING =====
float lastValidFreq = 0.0f;
unsigned long lastValidTime = 0;
const unsigned long HOLD_TIME = 300;

// ===== SYSTEM STATES =====
enum SystemState {
  STATE_OFF,
  STATE_STANDBY,
  STATE_TUNING,
  STATE_AUTO_TUNE_ALL,
  STATE_STRING_SELECT,
  STATE_MODE_SELECT
};

SystemState currentState = STATE_STANDBY;
int tuningMode = 0;
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
  const char* noteNames[6];  // Note names for each string in this tuning
};

TuningDef tuningModes[] = {
  {"STANDARD",    {82.41, 110.0, 146.83, 196.0, 246.94, 329.63},  {"E2", "A2", "D3", "G3", "B3", "E4"}},
  {"Eb Standard", {77.78, 103.83, 138.59, 185.0, 233.08, 311.13}, {"Eb2", "Ab2", "Db3", "Gb3", "Bb3", "Eb4"}},
  {"Drop D",      {73.42, 110.0, 146.83, 196.0, 246.94, 329.63},  {"D2", "A2", "D3", "G3", "B3", "E4"}},
  {"Open G",      {73.42, 98.0, 146.83, 196.0, 246.94, 293.66},   {"D2", "G2", "D3", "G3", "B3", "D4"}}
};

const char* NOTE_NAMES[] = {"C","C#","D","D#","E","F","F#","G","G#","A","A#","B"};

int TUNE_TOLERANCE = 10;
float signalLevel = 0;
unsigned long currentTuneStartTime = 0;

// ===== SERVO =====
Servo tunerServo;
int servoPos = 105;  // Center for 210° servo
int targetServoPos = 105;
unsigned long lastServoMove = 0;
uint32_t SERVO_MOVE_PERIOD = 100;  // Reduced from 150 for more responsive tuning
bool servoAttached = false;

// Servo limits (210° range: 0-210)
const int SERVO_MIN = 0;
const int SERVO_MAX = 210;
const int SERVO_CENTER = 105;
const int SERVO_SOFT_MIN = 15;   // Warn before hitting physical limit
const int SERVO_SOFT_MAX = 195;  // Warn before hitting physical limit

// Limit warning state
bool servoLimitReached = false;
bool servoReturningToCenter = false;  // Motor is moving to center, waiting for second SELECT
bool needsTightenRoom = false;  // true = ran out of tighten room, false = ran out of loosen room
bool useWideDetection = false;  // Use wider frequency detection after limit reset

int lastCents = 0;

// ===== IN TUNE STABILITY =====
unsigned long inTuneStartTime = 0;
bool wasInTune = false;
const unsigned long IN_TUNE_DURATION = 500;

// ===== STRUM DETECTION =====
bool hadSignal = false;

// ===== SERVO CONFIRMATION =====
bool waitingForConfirm = true;  // Wait for SELECT button before servo moves

// ===== SUCCESS ANIMATION =====
bool showSuccessAnimation = false;
int successAnimationFrame = 0;
unsigned long successAnimationStartTime = 0;
const unsigned long SUCCESS_DISPLAY_TIME = 2000;

// ===== COLORS =====
#define COLOR_BG        0x0000
#define COLOR_CARD      0x18E3
#define COLOR_PRIMARY   0x07FF
#define COLOR_SUCCESS   0x07E0
#define COLOR_WARNING   0xFD20
#define COLOR_DANGER    0xF800
#define COLOR_TEXT      0xFFFF
#define COLOR_TEXT_DIM  0x8410

// ===== SAMPLE CAPTURE =====

void captureSamples() {
  unsigned long periodUs = 1000000UL / (unsigned long)SAMPLING_FREQ;

  for (int i = 0; i < SAMPLES; i++) {
    unsigned long start = micros();
    sampleBuffer[i] = analogRead(PIEZO_PIN);
    while (micros() - start < periodUs);
  }
}

void removeDC() {
  int32_t sum = 0;
  for (int i = 0; i < SAMPLES; i++) {
    sum += sampleBuffer[i];
  }
  int16_t mean = sum / SAMPLES;
  for (int i = 0; i < SAMPLES; i++) {
    sampleBuffer[i] -= mean;
  }
}

// ===== AUTOCORRELATION PITCH DETECTION =====

float calculateSignalLevel() {
  int32_t sum = 0;
  for (int i = 0; i < SAMPLES; i++) {
    sum += abs(sampleBuffer[i]);
  }
  return (float)sum / SAMPLES;
}

float detectPitchAutocorrelation(float expectedFreq) {
  removeDC();

  signalLevel = calculateSignalLevel();
  if (signalLevel < NOISE_THRESHOLD) {
    return 0.0f;
  }

  int globalMinLag = (int)(SAMPLING_FREQ / F_MAX);
  int globalMaxLag = (int)(SAMPLING_FREQ / F_MIN);
  if (globalMaxLag > SAMPLES / 2) globalMaxLag = SAMPLES / 2;
  if (globalMinLag < 2) globalMinLag = 2;

  int minLag = globalMinLag;
  int maxLag = globalMaxLag;

  if (expectedFreq > 0.0f) {
    int centerLag = (int)(SAMPLING_FREQ / expectedFreq);
    // Narrower window: 0.7x to 1.3x to avoid harmonics
    int localMin = (int)(centerLag * 0.7f);
    int localMax = (int)(centerLag * 1.3f);
    if (localMin < globalMinLag) localMin = globalMinLag;
    if (localMax > globalMaxLag) localMax = globalMaxLag;
    minLag = localMin;
    maxLag = localMax;
  }

  int32_t maxCorr = 0;
  int bestLag = 0;

  for (int lag = minLag; lag <= maxLag; lag++) {
    int32_t corr = 0;
    for (int i = 0; i < SAMPLES - lag; i++) {
      corr += (int32_t)sampleBuffer[i] * sampleBuffer[i + lag];
    }
    if (corr > maxCorr) {
      maxCorr = corr;
      bestLag = lag;
    }
  }

  if (bestLag == 0) return 0.0f;

  float detectedFreq = SAMPLING_FREQ / (float)bestLag;
  if (detectedFreq < F_MIN || detectedFreq > F_MAX) {
    return 0.0f;
  }

  // Always check for subharmonic (octave below) - harmonics are common on guitar
  // If detected frequency is significantly higher than expected, check for fundamental
  bool checkSubharmonic = false;
  if (expectedFreq > 0.0f && detectedFreq > expectedFreq * 1.4f) {
    // Detected freq is way higher than expected - likely a harmonic
    checkSubharmonic = true;
  } else if (expectedFreq <= 0.0f && detectedFreq > 150.0f) {
    // Auto mode - check higher frequencies for possible harmonics
    checkSubharmonic = true;
  }
  
  if (checkSubharmonic) {
    int doubleLag = bestLag * 2;
    if (doubleLag <= globalMaxLag) {
      int32_t corr2x = 0;
      for (int i = 0; i < SAMPLES - doubleLag; i++) {
        corr2x += (int32_t)sampleBuffer[i] * sampleBuffer[i + doubleLag];
      }
      // If subharmonic correlation is reasonably strong, use it
      if (corr2x > maxCorr * 0.5f) {
        bestLag = doubleLag;
        maxCorr = corr2x;
        detectedFreq = SAMPLING_FREQ / (float)bestLag;
        Serial.printf("Subharmonic correction: %.1f Hz\n", detectedFreq);
      }
    }
  }

  if (bestLag > minLag && bestLag < maxLag) {
    int32_t corrPrev = 0, corrCurr = maxCorr, corrNext = 0;
    for (int i = 0; i < SAMPLES - bestLag - 1; i++) {
      corrPrev += (int32_t)sampleBuffer[i] * sampleBuffer[i + bestLag - 1];
      corrNext += (int32_t)sampleBuffer[i] * sampleBuffer[i + bestLag + 1];
    }

    float denom = 2.0f * (corrPrev - 2.0f * corrCurr + corrNext);
    if (fabsf(denom) > 0.001f) {
      float delta = (float)(corrPrev - corrNext) / denom;
      delta = constrain(delta, -0.5f, 0.5f);
      float refinedLag = bestLag + delta;
      if (refinedLag > 0) {
        detectedFreq = SAMPLING_FREQ / refinedLag;
      }
    }
  }

  if (detectedFreq < F_MIN || detectedFreq > F_MAX) {
    return 0.0f;
  }

  return detectedFreq;
}

// ===== UI HELPER FUNCTIONS =====

void drawCenteredText(const char* text, int y, int size, uint16_t color) {
  tft.setTextSize(size);
  tft.setTextColor(color);
  int16_t x1, y1;
  uint16_t w, h;
  tft.getTextBounds(text, 0, 0, &x1, &y1, &w, &h);
  tft.setCursor((320 - w) / 2, y);
  tft.print(text);
}

void drawCentsMeter(int y, int cents) {
  int meterWidth = 280;
  int meterHeight = 40;
  int x = (320 - meterWidth) / 2;

  tft.fillRoundRect(x, y, meterWidth, meterHeight, 6, COLOR_CARD);

  int centerX = x + meterWidth / 2;
  tft.drawFastVLine(centerX, y + 8, meterHeight - 16, COLOR_TEXT_DIM);

  int tolerancePixels = map(TUNE_TOLERANCE, 0, 50, 0, meterWidth / 2);
  tft.fillRect(centerX - tolerancePixels, y + 4, tolerancePixels * 2, meterHeight - 8, 0x0320);

  int c = constrain(cents, -50, 50);
  int indicatorX = centerX + map(c, -50, 50, -meterWidth/2 + 15, meterWidth/2 - 15);

  uint16_t color = COLOR_SUCCESS;
  if (abs(cents) > TUNE_TOLERANCE && abs(cents) <= 15) color = COLOR_WARNING;
  else if (abs(cents) > 15) color = COLOR_DANGER;

  tft.fillCircle(indicatorX, y + meterHeight/2, 12, color);
  tft.drawCircle(indicatorX, y + meterHeight/2, 13, COLOR_TEXT);
}

void drawStringIndicator(int stringNum) {
  int y = 45;
  int boxWidth = 45;
  int spacing = 5;
  int totalWidth = 6 * boxWidth + 5 * spacing;
  int startX = (320 - totalWidth) / 2;

  for (int i = 0; i < 6; i++) {
    int x = startX + i * (boxWidth + spacing);
    bool isSelected = (stringNum == i);

    if (isSelected) {
      tft.fillRoundRect(x, y, boxWidth, 30, 4, COLOR_PRIMARY);
      tft.setTextColor(COLOR_BG);
    } else {
      tft.fillRoundRect(x, y, boxWidth, 30, 4, COLOR_CARD);
      tft.setTextColor(COLOR_TEXT_DIM);
    }

    tft.setTextSize(1);
    int16_t x1, y1;
    uint16_t w, h;
    // Use note names from current tuning mode
    tft.getTextBounds(tuningModes[tuningMode].noteNames[i], 0, 0, &x1, &y1, &w, &h);
    tft.setCursor(x + (boxWidth - w) / 2, y + 10);
    tft.print(tuningModes[tuningMode].noteNames[i]);
  }
}

// ===== UI SCREENS =====

void drawStandbyScreen() {
  tft.fillScreen(COLOR_BG);

  drawCenteredText("GUITAR TUNER", 30, 3, COLOR_PRIMARY);

  tft.fillRoundRect(20, 80, 280, 50, 8, COLOR_CARD);
  tft.setTextSize(1);
  tft.setTextColor(COLOR_TEXT_DIM);
  tft.setCursor(30, 90);
  tft.print("MODE");
  tft.setTextSize(2);
  tft.setTextColor(COLOR_TEXT);
  tft.setCursor(30, 105);
  tft.print(tuningModes[tuningMode].name);

  tft.setTextSize(1);
  tft.setTextColor(COLOR_TEXT_DIM);
  tft.setCursor(180, 90);
  tft.print("STRING");
  tft.setTextSize(2);
  tft.setTextColor(COLOR_PRIMARY);
  tft.setCursor(180, 105);
  if (isAutoMode) tft.print("AUTO");
  else tft.print(tuningModes[tuningMode].noteNames[selectedString]);

  tft.fillRoundRect(20, 150, 280, 70, 8, COLOR_CARD);
  tft.setTextSize(1);
  tft.setTextColor(COLOR_TEXT);
  tft.setCursor(35, 165);
  tft.print("TOGGLE: Start Tuning");
  tft.setCursor(35, 180);
  tft.print("SELECT: Choose String");
  tft.setTextColor(COLOR_TEXT_DIM);
  tft.setCursor(35, 200);
  tft.print("Long press for more options");
}

void drawTuningScreen() {
  tft.fillScreen(COLOR_BG);

  tft.setTextSize(1);
  tft.setTextColor(COLOR_TEXT_DIM);
  tft.setCursor(10, 10);
  tft.print(tuningModes[tuningMode].name);

  tft.setTextColor(COLOR_SUCCESS);
  tft.setCursor(260, 10);
  tft.print("TUNING");

  drawStringIndicator(-1);
}

void updateTuningDisplay(float freq, const String& note, int cents, int stringNum) {
  drawStringIndicator(stringNum);

  tft.fillRect(20, 85, 280, 75, COLOR_BG);

  if (freq > 0) {
    tft.setTextSize(5);
    tft.setTextColor(COLOR_TEXT);
    int16_t x1, y1;
    uint16_t w, h;
    tft.getTextBounds(note.c_str(), 0, 0, &x1, &y1, &w, &h);
    tft.setCursor((320 - w) / 2, 90);
    tft.print(note);

    tft.setTextSize(2);
    tft.setTextColor(COLOR_TEXT_DIM);
    char freqStr[16];
    sprintf(freqStr, "%d Hz", (int)freq);
    tft.getTextBounds(freqStr, 0, 0, &x1, &y1, &w, &h);
    tft.setCursor((320 - w) / 2, 140);
    tft.print(freqStr);
  } else {
    tft.setTextSize(4);
    tft.setTextColor(COLOR_TEXT_DIM);
    int16_t x1, y1;
    uint16_t w, h;
    tft.getTextBounds("---", 0, 0, &x1, &y1, &w, &h);
    tft.setCursor((320 - w) / 2, 100);
    tft.print("---");
  }

  drawCentsMeter(175, cents);

  tft.fillRect(0, 220, 320, 20, COLOR_BG);

  if (servoLimitReached && waitingForConfirm) {
    // Show limit warning overlay
    tft.fillRect(20, 85, 280, 75, COLOR_DANGER);
    if (!servoReturningToCenter) {
      // Step 1: Just hit limit
      drawCenteredText("SERVO LIMIT!", 95, 2, COLOR_TEXT);
      if (needsTightenRoom) {
        drawCenteredText("Need more room to TIGHTEN", 120, 1, COLOR_TEXT);
      } else {
        drawCenteredText("Need more room to LOOSEN", 120, 1, COLOR_TEXT);
      }
      drawCenteredText("Press SELECT to reposition", 222, 2, COLOR_WARNING);
    } else {
      // Step 2: Servo at center, waiting to resume
      drawCenteredText("REPOSITION MOTOR", 95, 2, COLOR_TEXT);
      drawCenteredText("Motor at center position", 120, 1, COLOR_TEXT);
      drawCenteredText("Press SELECT to resume", 222, 2, COLOR_WARNING);
    }
  } else if (waitingForConfirm) {
    drawCenteredText("Press SELECT to start", 222, 2, COLOR_WARNING);
  } else if (freq <= 0) {
    drawCenteredText("Play a string", 222, 2, COLOR_TEXT_DIM);
  } else if (abs(cents) <= TUNE_TOLERANCE) {
    drawCenteredText("IN TUNE", 222, 2, COLOR_SUCCESS);
  } else {
    char statusStr[20];
    sprintf(statusStr, "%s%d cents", cents > 0 ? "+" : "", cents);
    uint16_t color = abs(cents) > 15 ? COLOR_DANGER : COLOR_WARNING;
    drawCenteredText(statusStr, 222, 2, color);
  }
}

void drawAutoTuneAllScreen() {
  tft.fillScreen(COLOR_BG);

  drawCenteredText("AUTO TUNE", 20, 3, COLOR_PRIMARY);
  
  // Show tuning mode name
  tft.setTextSize(1);
  tft.setTextColor(COLOR_TEXT_DIM);
  tft.setCursor(10, 50);
  tft.print(tuningModes[tuningMode].name);

  // Draw string boxes
  int boxSize = 40;
  int spacing = 10;
  int totalWidth = 6 * boxSize + 5 * spacing;
  int startX = (320 - totalWidth) / 2;
  int y = 70;

  for (int i = 0; i < 6; i++) {
    int x = startX + i * (boxSize + spacing);

    uint16_t bgColor = COLOR_CARD;
    uint16_t textColor = COLOR_TEXT_DIM;

    if (i < autoTuneCurrentString) {
      bgColor = COLOR_SUCCESS;
      textColor = COLOR_BG;
    } else if (i == autoTuneCurrentString) {
      bgColor = COLOR_WARNING;
      textColor = COLOR_BG;
    }

    tft.fillRoundRect(x, y, boxSize, boxSize, 6, bgColor);
    tft.setTextSize(1);
    tft.setTextColor(textColor);
    int16_t x1, y1;
    uint16_t w, h;
    tft.getTextBounds(tuningModes[tuningMode].noteNames[i], 0, 0, &x1, &y1, &w, &h);
    tft.setCursor(x + (boxSize - w) / 2, y + (boxSize - h) / 2);
    tft.print(tuningModes[tuningMode].noteNames[i]);
  }

  // Show current note being tuned
  if (autoTuneCurrentString < 6) {
    tft.setTextSize(4);
    tft.setTextColor(COLOR_WARNING);
    const char* noteName = tuningModes[tuningMode].noteNames[autoTuneCurrentString];
    tft.setCursor(80, 135);
    tft.print(noteName);
    
    tft.setTextSize(2);
    tft.setTextColor(COLOR_TEXT_DIM);
    tft.setCursor(180, 145);
    tft.print((int)tuningModes[tuningMode].freqs[autoTuneCurrentString]);
    tft.print(" Hz");
  }

  drawCentsMeter(210, 0);
}

void updateAutoTuneDisplay(float freq, int cents) {
  // Redraw string boxes to show current string highlighted
  int boxSize = 40;
  int spacing = 10;
  int totalWidth = 6 * boxSize + 5 * spacing;
  int startX = (320 - totalWidth) / 2;
  int y = 70;

  for (int i = 0; i < 6; i++) {
    int x = startX + i * (boxSize + spacing);

    uint16_t bgColor = COLOR_CARD;
    uint16_t textColor = COLOR_TEXT_DIM;

    if (i < autoTuneCurrentString) {
      bgColor = COLOR_SUCCESS;
      textColor = COLOR_BG;
    } else if (i == autoTuneCurrentString) {
      bgColor = COLOR_WARNING;
      textColor = COLOR_BG;
    }

    tft.fillRoundRect(x, y, boxSize, boxSize, 6, bgColor);
    tft.setTextSize(1);
    tft.setTextColor(textColor);
    int16_t x1, y1;
    uint16_t w, h;
    tft.getTextBounds(tuningModes[tuningMode].noteNames[i], 0, 0, &x1, &y1, &w, &h);
    tft.setCursor(x + (boxSize - w) / 2, y + (boxSize - h) / 2);
    tft.print(tuningModes[tuningMode].noteNames[i]);
  }

  // Show current note being tuned in large text
  tft.fillRect(40, 130, 240, 55, COLOR_BG);
  if (autoTuneCurrentString < 6) {
    // Large note name
    tft.setTextSize(4);
    tft.setTextColor(COLOR_WARNING);
    int16_t x1, y1;
    uint16_t w, h;
    const char* noteName = tuningModes[tuningMode].noteNames[autoTuneCurrentString];
    tft.getTextBounds(noteName, 0, 0, &x1, &y1, &w, &h);
    tft.setCursor(80, 135);
    tft.print(noteName);
    
    // Target frequency
    tft.setTextSize(2);
    tft.setTextColor(COLOR_TEXT_DIM);
    tft.setCursor(180, 145);
    tft.print((int)tuningModes[tuningMode].freqs[autoTuneCurrentString]);
    tft.print(" Hz");
  }

  tft.fillRect(0, 190, 320, 50, COLOR_BG);
  drawCentsMeter(210, freq > 0 ? cents : 0);

  if (servoLimitReached && waitingForConfirm) {
    tft.fillRect(0, 130, 320, 60, COLOR_DANGER);
    if (!servoReturningToCenter) {
      drawCenteredText("SERVO LIMIT!", 135, 2, COLOR_TEXT);
      if (needsTightenRoom) {
        drawCenteredText("Need more room to TIGHTEN", 155, 1, COLOR_TEXT);
      } else {
        drawCenteredText("Need more room to LOOSEN", 155, 1, COLOR_TEXT);
      }
      drawCenteredText("Press SELECT to reposition", 192, 2, COLOR_WARNING);
    } else {
      drawCenteredText("REPOSITION MOTOR", 135, 2, COLOR_TEXT);
      drawCenteredText("Motor at center position", 155, 1, COLOR_TEXT);
      drawCenteredText("Press SELECT to resume", 192, 2, COLOR_WARNING);
    }
  } else if (waitingForConfirm) {
    drawCenteredText("Press SELECT to start", 192, 2, COLOR_WARNING);
  } else if (freq > 0) {
    char buf[32];
    sprintf(buf, "%d Hz (%s%d)", (int)freq, cents > 0 ? "+" : "", cents);
    drawCenteredText(buf, 192, 2, COLOR_TEXT);
  } else {
    drawCenteredText("Play string", 192, 2, COLOR_TEXT_DIM);
  }
}

void drawStringSelectScreen() {
  tft.fillScreen(COLOR_BG);

  drawCenteredText("SELECT STRING", 20, 2, COLOR_PRIMARY);
  
  // Show tuning mode name
  tft.setTextSize(1);
  tft.setTextColor(COLOR_TEXT_DIM);
  tft.setCursor(10, 45);
  tft.print(tuningModes[tuningMode].name);

  int boxWidth = 90;
  int boxHeight = 45;
  int spacing = 15;
  int startY = 60;

  bool autoSelected = isAutoMode;
  tft.fillRoundRect(115, startY, 90, 35, 6, autoSelected ? COLOR_PRIMARY : COLOR_CARD);
  tft.setTextSize(2);
  tft.setTextColor(autoSelected ? COLOR_BG : COLOR_TEXT);
  tft.setCursor(135, startY + 10);
  tft.print("AUTO");

  startY = 110;
  for (int i = 0; i < 6; i++) {
    int col = i % 3;
    int row = i / 3;
    int x = 25 + col * (boxWidth + spacing);
    int y = startY + row * (boxHeight + spacing);

    bool selected = (!isAutoMode && selectedString == i);

    tft.fillRoundRect(x, y, boxWidth, boxHeight, 6, selected ? COLOR_PRIMARY : COLOR_CARD);

    tft.setTextSize(2);
    tft.setTextColor(selected ? COLOR_BG : COLOR_TEXT);
    tft.setCursor(x + 20, y + 8);
    // Use note names from current tuning mode
    tft.print(tuningModes[tuningMode].noteNames[i]);

    tft.setTextSize(1);
    tft.setTextColor(selected ? COLOR_BG : COLOR_TEXT_DIM);
    tft.setCursor(x + 20, y + 30);
    tft.print((int)tuningModes[tuningMode].freqs[i]);
    tft.print(" Hz");
  }

  tft.setTextSize(1);
  tft.setTextColor(COLOR_TEXT_DIM);
  tft.setCursor(60, 225);
  tft.print("SELECT: cycle  |  TOGGLE: confirm");
}

void drawModeSelectScreen() {
  tft.fillScreen(COLOR_BG);

  drawCenteredText("TUNING MODE", 20, 2, COLOR_PRIMARY);

  int boxHeight = 40;
  int spacing = 10;
  int startY = 60;

  for (int i = 0; i < 4; i++) {
    int y = startY + i * (boxHeight + spacing);
    bool selected = (tuningMode == i);

    tft.fillRoundRect(30, y, 260, boxHeight, 6, selected ? COLOR_PRIMARY : COLOR_CARD);

    tft.setTextSize(2);
    tft.setTextColor(selected ? COLOR_BG : COLOR_TEXT);
    tft.setCursor(50, y + 12);
    tft.print(tuningModes[i].name);
  }

  tft.setTextSize(1);
  tft.setTextColor(COLOR_TEXT_DIM);
  tft.setCursor(60, 225);
  tft.print("SELECT: cycle  |  TOGGLE: confirm");
}

void drawSuccessAnimation() {
  if (!showSuccessAnimation) return;

  int centerX = 160;
  int centerY = 120;

  int size = 40 + (successAnimationFrame % 20);
  tft.fillCircle(centerX, centerY, size, COLOR_SUCCESS);

  tft.drawLine(centerX - 15, centerY, centerX - 5, centerY + 15, COLOR_TEXT);
  tft.drawLine(centerX - 5, centerY + 15, centerX + 20, centerY - 15, COLOR_TEXT);
  tft.drawLine(centerX - 14, centerY, centerX - 5, centerY + 14, COLOR_TEXT);
  tft.drawLine(centerX - 5, centerY + 14, centerX + 19, centerY - 15, COLOR_TEXT);

  successAnimationFrame++;
}

// ===== BUTTON HANDLING =====

void initButtons() {
  pinMode(BTN_TOGGLE, INPUT_PULLUP);
  pinMode(BTN_SELECT, INPUT_PULLUP);
}

int readButtonPress(int buttonIndex, int pin) {
  bool currentState = digitalRead(pin);
  int result = 0;

  if (currentState == LOW && lastButtonState[buttonIndex] == HIGH) {
    buttonPressStart[buttonIndex] = millis();
    buttonLongPressTriggered[buttonIndex] = false;
  }

  if (currentState == HIGH && lastButtonState[buttonIndex] == LOW) {
    unsigned long duration = millis() - buttonPressStart[buttonIndex];

    if (!buttonLongPressTriggered[buttonIndex]) {
      if (duration >= 2000) result = 3;
      else if (duration >= 800) result = 2;
      else if (duration >= 50) result = 1;
    }
  }

  if (currentState == LOW && !buttonLongPressTriggered[buttonIndex]) {
    unsigned long duration = millis() - buttonPressStart[buttonIndex];

    if (duration >= 2000) {
      buttonLongPressTriggered[buttonIndex] = true;
      result = 3;
    } else if (duration >= 800) {
      buttonLongPressTriggered[buttonIndex] = true;
      result = 2;
    }
  }

  lastButtonState[buttonIndex] = currentState;
  return result;
}

void handleButtons() {
  int toggleAction = readButtonPress(0, BTN_TOGGLE);
  int selectAction = readButtonPress(1, BTN_SELECT);

  if (toggleAction > 0) {
    if (toggleAction == 3) {
      currentState = STATE_OFF;
      tft.fillScreen(COLOR_BG);
      if (servoAttached) {
        servoPos = SERVO_CENTER;
        tunerServo.write(servoPos);
        delay(200);
        tunerServo.detach();
        servoAttached = false;
      }

    } else if (toggleAction == 2) {
      if (currentState == STATE_STANDBY) {
        currentState = STATE_AUTO_TUNE_ALL;
        autoTuneInProgress = true;
        autoTuneCurrentString = 0;
        autoTuneStringStartTime = millis();
        currentTuneStartTime = millis();
        hadSignal = false;
        wasInTune = false;
        inTuneStartTime = 0;
        waitingForConfirm = true;  // Wait for SELECT before moving servo
        if (!servoAttached) {
          tunerServo.attach(SERVO_PIN, 500, 2500);
          servoAttached = true;
        }
        drawAutoTuneAllScreen();
      }

    } else if (toggleAction == 1) {
      if (currentState == STATE_OFF) {
        currentState = STATE_STANDBY;
        drawStandbyScreen();

      } else if (currentState == STATE_STANDBY) {
        currentState = STATE_TUNING;
        drawTuningScreen();
        currentTuneStartTime = millis();
        hadSignal = false;
        wasInTune = false;
        inTuneStartTime = 0;
        waitingForConfirm = true;  // Wait for SELECT before moving servo
        if (!servoAttached) {
          tunerServo.attach(SERVO_PIN, 500, 2500);
          servoAttached = true;
        }

      } else if (currentState == STATE_TUNING || currentState == STATE_AUTO_TUNE_ALL) {
        currentState = STATE_STANDBY;
        autoTuneInProgress = false;
        servoPos = SERVO_CENTER;
        targetServoPos = SERVO_CENTER;
        hadSignal = false;
        wasInTune = false;
        inTuneStartTime = 0;
        servoLimitReached = false;
        servoReturningToCenter = false;
        useWideDetection = false;
        if (servoAttached) {
          tunerServo.write(servoPos);
          delay(200);
          tunerServo.detach();
          servoAttached = false;
        }
        drawStandbyScreen();

      } else if (currentState == STATE_STRING_SELECT || currentState == STATE_MODE_SELECT) {
        currentState = STATE_STANDBY;
        drawStandbyScreen();

      } else {
        currentState = STATE_STANDBY;
        drawStandbyScreen();
      }
    }
  }

  if (selectAction > 0) {
    if (selectAction == 2) {
      if (currentState == STATE_STANDBY) {
        currentState = STATE_MODE_SELECT;
        drawModeSelectScreen();
      }

    } else if (selectAction == 1) {
      // If tuning and waiting for confirmation, handle the flow
      if ((currentState == STATE_TUNING || currentState == STATE_AUTO_TUNE_ALL) && waitingForConfirm) {
        if (servoLimitReached && !servoReturningToCenter) {
          // Step 1: Limit reached, first SELECT press -> move servo to center
          servoReturningToCenter = true;
          servoPos = SERVO_CENTER;
          tunerServo.write(servoPos);
          targetServoPos = SERVO_CENTER;
          lastValidFreq = 0;  // Reset held frequency
          lastValidTime = 0;
          hadSignal = false;
          Serial.println("SELECT pressed - servo returning to center, reposition motor then press SELECT again");
        } else if (servoLimitReached && servoReturningToCenter) {
          // Step 2: Servo at center, second SELECT press -> resume tuning
          servoLimitReached = false;
          servoReturningToCenter = false;
          waitingForConfirm = false;
          lastValidFreq = 0;  // Reset held frequency
          lastValidTime = 0;
          hadSignal = false;
          useWideDetection = true;  // Use wider detection until we get stable signal
          Serial.println("SELECT pressed - resuming tuning with wide detection");
        } else {
          // Normal start (not from limit)
          waitingForConfirm = false;
          servoLimitReached = false;
          servoReturningToCenter = false;
          lastValidFreq = 0;  // Reset held frequency
          lastValidTime = 0;
          hadSignal = false;
          Serial.println("SELECT pressed - servo enabled");
        }
      } else if (currentState == STATE_STANDBY) {
        currentState = STATE_STRING_SELECT;
        drawStringSelectScreen();

      } else if (currentState == STATE_STRING_SELECT) {
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

      } else if (currentState == STATE_MODE_SELECT) {
        tuningMode = (tuningMode + 1) % 4;
        drawModeSelectScreen();
      }
    }
  }
}

// ===== PITCH HELPERS =====

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
    if (diff < bestDiff) {
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
  int idx = noteNum % 12;
  if (idx < 0) idx += 12;
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
  
  // Don't move servo until user presses SELECT
  if (waitingForConfirm) return;

  unsigned long now = millis();

  // In-tune detection with stability requirement
  if (abs(cents) <= TUNE_TOLERANCE) {
    if (!wasInTune) {
      inTuneStartTime = now;
      wasInTune = true;
      Serial.println("Entered in tune zone...");
    } else if (now - inTuneStartTime >= IN_TUNE_DURATION) {
      showSuccessAnimation = true;
      successAnimationFrame = 0;
      successAnimationStartTime = now;
      wasInTune = false;
      inTuneStartTime = 0;
      Serial.printf("IN TUNE (held for %lums at %d cents)\n", IN_TUNE_DURATION, cents);
      lastCents = cents;
      return;
    }
    // In zone, waiting for stability - don't move servo
    return;
  } else {
    // Outside tune zone - reset
    if (wasInTune) {
      Serial.println("Left in tune zone, resetting");
    }
    wasInTune = false;
    inTuneStartTime = 0;
  }

  // Rate limiting
  if (now - lastServoMove < SERVO_MOVE_PERIOD) return;

  // Calculate step size based on how far off we are
  int step = 1;
  int absCents = abs(cents);

  if (absCents > 30) step = 4;
  else if (absCents > 20) step = 3;
  else if (absCents > 10) step = 2;
  else step = 1;

  // Flat (negative cents) = need to tighten = increase servo angle
  // Sharp (positive cents) = need to loosen = decrease servo angle
  if (cents < 0) {
    targetServoPos = servoPos + step;
  } else {
    targetServoPos = servoPos - step;
  }

  targetServoPos = constrain(targetServoPos, SERVO_MIN, SERVO_MAX);

  // Check if approaching servo limits
  if (targetServoPos <= SERVO_SOFT_MIN || targetServoPos >= SERVO_SOFT_MAX) {
    // We're at the limit and still need to move
    servoLimitReached = true;
    needsTightenRoom = (cents < 0);  // negative cents = need to tighten more
    waitingForConfirm = true;  // Pause tuning
    
    Serial.printf("SERVO LIMIT REACHED - need to %s more, press SELECT to reposition\n", 
                  needsTightenRoom ? "tighten" : "loosen");
    return;
  }

  if (targetServoPos != servoPos) {
    attachServoIfNeeded();
    tunerServo.write(targetServoPos);
    Serial.printf("Servo: %d -> %d (cents: %d, step: %d)\n",
                  servoPos, targetServoPos, cents, step);
    servoPos = targetServoPos;
    lastServoMove = now;
  }

  lastCents = cents;
}

void checkSuccessAnimationComplete() {
  if (showSuccessAnimation && (millis() - successAnimationStartTime >= SUCCESS_DISPLAY_TIME)) {
    showSuccessAnimation = false;
    successAnimationFrame = 0;
    wasInTune = false;
    hadSignal = false;

    if (currentState == STATE_AUTO_TUNE_ALL) {
      autoTuneCurrentString++;

      if (autoTuneCurrentString >= 6) {
        currentState = STATE_STANDBY;
        autoTuneInProgress = false;
        if (servoAttached) {
          servoPos = SERVO_CENTER;
          tunerServo.write(servoPos);
          delay(200);
          tunerServo.detach();
          servoAttached = false;
        }
        drawStandbyScreen();
        Serial.println("AUTO TUNE ALL COMPLETE");
      } else {
        autoTuneStringStartTime = millis();
        currentTuneStartTime = millis();
        waitingForConfirm = true;  // Wait for SELECT before tuning next string
        lastValidFreq = 0;  // Reset held frequency for new string
        lastValidTime = 0;  // Reset hold timer
        drawAutoTuneAllScreen();
        Serial.printf("Next string: %s - press SELECT when ready\n", tuningModes[tuningMode].noteNames[autoTuneCurrentString]);
      }
    } else if (currentState == STATE_TUNING) {
      waitingForConfirm = true;  // Wait for SELECT before tuning next string
      lastValidFreq = 0;  // Reset held frequency for new string
      lastValidTime = 0;  // Reset hold timer
      drawTuningScreen();
      Serial.println("String tuned - press SELECT when ready for next");
    }
  }
}

// ===== SETUP =====

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== GUITAR TUNER v3.5 (strum detection fix) ===\n");

  sampleBuffer = (int16_t*)malloc(SAMPLES * sizeof(int16_t));
  if (!sampleBuffer) {
    Serial.println("Memory allocation failed");
    while (1) delay(1000);
  }

  analogReadResolution(12);
  analogSetAttenuation(ADC_6db);

  SPI.begin(TFT_CLK, -1, TFT_MOSI, TFT_CS);
  delay(100);

  tft.init(240, 320);
  tft.invertDisplay(false);
  tft.setRotation(3);

  uint8_t madctl = ST77XX_MADCTL_MX | ST77XX_MADCTL_MY | ST77XX_MADCTL_MV | ST77XX_MADCTL_RGB;
  tft.sendCommand(ST77XX_MADCTL, &madctl, 1);

  tft.fillScreen(COLOR_BG);

  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);

  initButtons();

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  tunerServo.setPeriodHertz(50);

  drawStandbyScreen();

  Serial.println("Ready");
}

// ===== MAIN LOOP =====

void loop() {
  handleButtons();

  if (currentState == STATE_TUNING || currentState == STATE_AUTO_TUNE_ALL) {
    attachServoIfNeeded();
    checkSuccessAnimationComplete();

    if (!showSuccessAnimation) {
      float expected = -1.0f;
      if (currentState == STATE_AUTO_TUNE_ALL) {
        expected = tuningModes[tuningMode].freqs[autoTuneCurrentString];
      } else if (currentState == STATE_TUNING && !isAutoMode && selectedString >= 0) {
        expected = tuningModes[tuningMode].freqs[selectedString];
      }
      
      // After limit reset, use wide detection (no expected) until we get signal
      float detectExpected = expected;
      if (useWideDetection) {
        detectExpected = -1.0f;  // Auto-detect any frequency
      }

      captureSamples();
      float rawFreq = detectPitchAutocorrelation(detectExpected);
      
      // Track raw signal for strum detection (before hold logic)
      bool hasRawSignal = (rawFreq > 0);
      
      float freq = rawFreq;

      // Relaxed clamp - skip if using wide detection
      if (!useWideDetection && expected > 0 && freq > 0) {
        // For auto-tune, be stricter - only accept frequencies within 50% of target
        // This prevents detecting other strings
        if (currentState == STATE_AUTO_TUNE_ALL) {
          if (freq < expected * 0.5f || freq > expected * 1.5f) {
            freq = 0.0f;
            hasRawSignal = false;
          }
        } else {
          // For manual mode, allow wider range
          if (freq < expected * 0.25f || freq > expected * 2.2f) {
            freq = 0.0f;
            hasRawSignal = false;
          }
        }
      }
      
      // Once we get a valid signal, switch back to narrow detection
      if (useWideDetection && hasRawSignal) {
        useWideDetection = false;
        Serial.println("Got signal - switching back to narrow detection");
      }

      // Hold logic for display smoothing only
      if (freq > 0) {
        lastValidFreq = freq;
        lastValidTime = millis();
      } else if (millis() - lastValidTime < HOLD_TIME) {
        freq = lastValidFreq;
        // Note: hasRawSignal stays false here - this is key!
      }

      String note = "--";
      int cents = 0;
      int stringNum = -1;

      // Detect new strum based on RAW signal for state reset
      if (hasRawSignal && !hadSignal) {
        Serial.println(">>> NEW STRUM DETECTED - resetting state <<<");
        wasInTune = false;
        inTuneStartTime = 0;
      }
      hadSignal = hasRawSignal;

      if (freq > 0) {
        freqToNote(freq, note, cents);
        stringNum = identifyString(freq);

        // Always let the servo try to correct whenever we have a frequency (even held)
        updateServoFromCents(cents);

        static unsigned long lastPrint = 0;
        if (millis() - lastPrint > 200) {
          Serial.printf("Freq: %.1f Hz | String: %d | Cents: %d | Signal: %.0f | Raw: %s\n",
                        freq, stringNum, cents, signalLevel, hasRawSignal ? "YES" : "held");
          lastPrint = millis();
        }
      }
      // When freq == 0, just don't move the servo (no frequency to tune to)

      if (currentState == STATE_TUNING) {
        updateTuningDisplay(freq, note, cents, stringNum);
      } else if (currentState == STATE_AUTO_TUNE_ALL) {
        updateAutoTuneDisplay(freq, cents);
      }
    }

    if (showSuccessAnimation) {
      drawSuccessAnimation();
    }

    delay(10);
    yield();
  } else {
    detachServoIfNeeded();
    delay(20);
    yield();
  }
}
