#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>
#include <ESP32Servo.h>

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
const uint16_t SAMPLES = 1024;  // Smaller buffer = faster
const double SAMPLING_FREQ = 4096.0;  // Lower sample rate for guitar range
int16_t *sampleBuffer;  // Use int16 instead of double to save memory

// Frequency range for guitar (E2=82Hz to E4=330Hz)
const float F_MIN = 75.0f;
const float F_MAX = 350.0f;
const float NOISE_THRESHOLD = 50.0f;  // Minimum signal amplitude

// ===== PITCH TRACKING =====
float lastValidFreq = 0.0f;
unsigned long lastValidTime = 0;
const unsigned long HOLD_TIME = 300;  // Hold last pitch for 300ms

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
};

TuningDef tuningModes[] = {
  {"STANDARD", {82.41, 110.0, 146.83, 196.0, 246.94, 329.63}},
  {"Eb Standard", {77.78, 103.83, 138.59, 185.0, 233.08, 311.13}},
  {"Drop D", {73.42, 110.0, 146.83, 196.0, 246.94, 329.63}},
  {"Open G", {73.42, 98.0, 146.83, 196.0, 246.94, 293.66}}
};

const char* STRING_NAMES[] = {"E2", "A2", "D3", "G3", "B3", "E4"};
const char* NOTE_NAMES[] = {"C","C#","D","D#","E","F","F#","G","G#","A","A#","B"};

int TUNE_TOLERANCE = 5;
float signalLevel = 0;
unsigned long currentTuneStartTime = 0;

// ===== SERVO =====
Servo tunerServo;
int servoPos = 90;
int targetServoPos = 90;
unsigned long lastServoMove = 0;
uint32_t SERVO_MOVE_PERIOD = 150;
bool servoAttached = false;

int lastCents = 0;

bool servoIsMoving = false;
unsigned long servoMoveStartTime = 0;
const uint32_t SERVO_MOVE_DURATION = 120;

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

// ===== AUTOCORRELATION PITCH DETECTION =====

void captureSamples() {
  // Fast sample capture
  unsigned long periodUs = 1000000UL / (unsigned long)SAMPLING_FREQ;
  
  for (int i = 0; i < SAMPLES; i++) {
    unsigned long start = micros();
    sampleBuffer[i] = analogRead(PIEZO_PIN) - 2048;  // Center around zero
    while (micros() - start < periodUs);
  }
}

float calculateSignalLevel() {
  int32_t sum = 0;
  for (int i = 0; i < SAMPLES; i++) {
    sum += abs(sampleBuffer[i]);
  }
  return (float)sum / SAMPLES;
}

float detectPitchAutocorrelation() {
  // Calculate signal level first
  signalLevel = calculateSignalLevel();
  
  if (signalLevel < NOISE_THRESHOLD) {
    return 0.0f;
  }
  
  // Autocorrelation-based pitch detection
  // We look for the lag where the signal correlates best with itself
  
  int minLag = (int)(SAMPLING_FREQ / F_MAX);  // ~12 samples for 350Hz
  int maxLag = (int)(SAMPLING_FREQ / F_MIN);  // ~55 samples for 75Hz
  
  // Limit maxLag to avoid going out of bounds
  if (maxLag > SAMPLES / 2) maxLag = SAMPLES / 2;
  
  int32_t maxCorr = 0;
  int bestLag = 0;
  
  // Calculate autocorrelation for each lag
  for (int lag = minLag; lag <= maxLag; lag++) {
    int32_t corr = 0;
    
    // Sum of products
    for (int i = 0; i < SAMPLES - lag; i++) {
      corr += (int32_t)sampleBuffer[i] * sampleBuffer[i + lag];
    }
    
    if (corr > maxCorr) {
      maxCorr = corr;
      bestLag = lag;
    }
  }
  
  // Need a strong correlation
  if (maxCorr < 1000000) {
    return 0.0f;
  }
  
  // Parabolic interpolation for sub-sample accuracy
  if (bestLag > minLag && bestLag < maxLag) {
    int32_t corrPrev = 0, corrNext = 0;
    
    for (int i = 0; i < SAMPLES - bestLag - 1; i++) {
      corrPrev += (int32_t)sampleBuffer[i] * sampleBuffer[i + bestLag - 1];
      corrNext += (int32_t)sampleBuffer[i] * sampleBuffer[i + bestLag + 1];
    }
    
    float delta = (float)(corrPrev - corrNext) / (2.0f * (corrPrev - 2.0f * maxCorr + corrNext));
    float refinedLag = bestLag + delta;
    
    if (refinedLag > 0) {
      return SAMPLING_FREQ / refinedLag;
    }
  }
  
  return SAMPLING_FREQ / (float)bestLag;
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
    tft.getTextBounds(STRING_NAMES[i], 0, 0, &x1, &y1, &w, &h);
    tft.setCursor(x + (boxWidth - w) / 2, y + 10);
    tft.print(STRING_NAMES[i]);
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
  else tft.print(STRING_NAMES[selectedString]);
  
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
  
  // Clear the entire note and frequency area
  tft.fillRect(20, 85, 280, 75, COLOR_BG);
  
  if (freq > 0) {
    // Large note display - centered
    tft.setTextSize(5);
    tft.setTextColor(COLOR_TEXT);
    int16_t x1, y1;
    uint16_t w, h;
    tft.getTextBounds(note.c_str(), 0, 0, &x1, &y1, &w, &h);
    tft.setCursor((320 - w) / 2, 90);
    tft.print(note);
    
    // Frequency below note - centered
    tft.setTextSize(2);
    tft.setTextColor(COLOR_TEXT_DIM);
    char freqStr[16];
    sprintf(freqStr, "%d Hz", (int)freq);
    tft.getTextBounds(freqStr, 0, 0, &x1, &y1, &w, &h);
    tft.setCursor((320 - w) / 2, 140);
    tft.print(freqStr);
  } else {
    // No signal - show placeholder
    tft.setTextSize(4);
    tft.setTextColor(COLOR_TEXT_DIM);
    int16_t x1, y1;
    uint16_t w, h;
    tft.getTextBounds("---", 0, 0, &x1, &y1, &w, &h);
    tft.setCursor((320 - w) / 2, 100);
    tft.print("---");
  }
  
  // Cents meter
  drawCentsMeter(175, cents);
  
  // Status text - clear and redraw
  tft.fillRect(0, 220, 320, 20, COLOR_BG);
  
  if (freq <= 0) {
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
    tft.setTextSize(2);
    tft.setTextColor(textColor);
    int16_t x1, y1;
    uint16_t w, h;
    tft.getTextBounds(STRING_NAMES[i], 0, 0, &x1, &y1, &w, &h);
    tft.setCursor(x + (boxSize - w) / 2, y + (boxSize - h) / 2);
    tft.print(STRING_NAMES[i]);
  }
  
  if (autoTuneCurrentString < 6) {
    tft.fillRoundRect(40, 130, 240, 50, 8, COLOR_CARD);
    tft.setTextSize(1);
    tft.setTextColor(COLOR_TEXT_DIM);
    tft.setCursor(55, 140);
    tft.print("NOW TUNING:");
    tft.setTextSize(2);
    tft.setTextColor(COLOR_WARNING);
    tft.setCursor(55, 158);
    tft.print(STRING_NAMES[autoTuneCurrentString]);
    tft.print(" - ");
    tft.print((int)tuningModes[tuningMode].freqs[autoTuneCurrentString]);
    tft.print(" Hz");
  }
}

void updateAutoTuneDisplay(float freq, int cents) {
  tft.fillRect(40, 190, 240, 45, COLOR_BG);
  
  if (freq > 0) {
    char buf[32];
    sprintf(buf, "%d Hz (%s%d)", (int)freq, cents > 0 ? "+" : "", cents);
    drawCenteredText(buf, 195, 2, COLOR_TEXT);
    drawCentsMeter(210, cents);
  } else {
    drawCenteredText("Waiting...", 200, 2, COLOR_TEXT_DIM);
  }
}

void drawStringSelectScreen() {
  tft.fillScreen(COLOR_BG);
  
  drawCenteredText("SELECT STRING", 20, 2, COLOR_PRIMARY);
  
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
    tft.setCursor(x + 25, y + 8);
    tft.print(STRING_NAMES[i]);
    
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
        servoPos = 90;
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
        if (!servoAttached) {
          tunerServo.attach(SERVO_PIN, 500, 2500);
          servoAttached = true;
        }
        
      } else if (currentState == STATE_TUNING || currentState == STATE_AUTO_TUNE_ALL) {
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
      if (currentState == STATE_STANDBY) {
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
  
  unsigned long now = millis();
  
  // INSTANT SUCCESS: If within tolerance, immediately trigger success!
  if (abs(cents) <= TUNE_TOLERANCE) {
    servoIsMoving = false;
    showSuccessAnimation = true;
    successAnimationFrame = 0;
    successAnimationStartTime = millis();
    
    Serial.printf("IN TUNE! (%d cents)\n", cents);
    targetServoPos = servoPos;
    lastCents = cents;
    return;
  }
  
  // Check timeout for auto tune all
  if (currentState == STATE_AUTO_TUNE_ALL && millis() - autoTuneStringStartTime > AUTO_TUNE_TIMEOUT) {
    Serial.println("Timeout - skipping");
    autoTuneCurrentString++;
    
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
  
  // Wait for servo to finish moving before next adjustment
  if (servoIsMoving) {
    if (now - servoMoveStartTime < SERVO_MOVE_DURATION) {
      return;
    } else {
      servoIsMoving = false;
    }
  }
  
  if (now - lastServoMove < SERVO_MOVE_PERIOD) return;
  
  // Calculate step size based on how far off we are
  int step = 1;
  int absCents = abs(cents);
  
  if (absCents > 30) step = 5;
  else if (absCents > 20) step = 3;
  else if (absCents > 10) step = 2;
  else step = 1;
  
  // Move servo in correct direction
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
      } else {
        autoTuneStringStartTime = millis();
        currentTuneStartTime = millis();
        drawAutoTuneAllScreen();
      }
    } else if (currentState == STATE_TUNING) {
      if (!isAutoMode && selectedString < 5) {
        selectedString++;
        currentTuneStartTime = millis();
        drawTuningScreen();
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
  Serial.println("\n=== GUITAR TUNER v3.0 (Autocorrelation) ===\n");

  // Allocate sample buffer
  sampleBuffer = (int16_t*)malloc(SAMPLES * sizeof(int16_t));
  if (!sampleBuffer) {
    Serial.println("Memory allocation failed!");
    while (1) delay(1000);
  }

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

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
  
  Serial.println("Ready! Using autocorrelation for pitch detection.");
}

// ===== MAIN LOOP =====

void loop() {
  handleButtons();
  
  if (currentState == STATE_TUNING || currentState == STATE_AUTO_TUNE_ALL) {
    attachServoIfNeeded();
    checkSuccessAnimationComplete();
    
    if (!showSuccessAnimation) {
      // Capture and detect pitch
      captureSamples();
      float freq = detectPitchAutocorrelation();
      
      // Hold last valid frequency briefly after signal drops
      if (freq > 0) {
        lastValidFreq = freq;
        lastValidTime = millis();
      } else if (millis() - lastValidTime < HOLD_TIME) {
        freq = lastValidFreq;
      }
      
      String note = "--";
      int cents = 0;
      int stringNum = -1;
      
      if (freq > 0) {
        freqToNote(freq, note, cents);
        stringNum = identifyString(freq);
        
        if (stringNum >= 0) {
          updateServoFromCents(cents);
        }
        
        // Debug output
        static unsigned long lastPrint = 0;
        if (millis() - lastPrint > 200) {
          Serial.printf("Freq: %.1f Hz | Note: %s | Cents: %d | Signal: %.0f\n", 
                        freq, note.c_str(), cents, signalLevel);
          lastPrint = millis();
        }
      }

      if (currentState == STATE_TUNING) {
        updateTuningDisplay(freq, note, cents, stringNum);
      } else if (currentState == STATE_AUTO_TUNE_ALL) {
        updateAutoTuneDisplay(freq, cents);
      }
    }
    
    if (showSuccessAnimation) {
      drawSuccessAnimation();
    }
    
    delay(10);  // Faster loop with autocorrelation
    yield();
  } else {
    detachServoIfNeeded();
    delay(20);
    yield();
  }
}
