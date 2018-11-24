/*
 * Envelope Generator / LFO
 * 
 * Controls: A/D/S/R
 * Inputs: gate, retrigger
 * Outputs: envelope (0v~+5v)
 * 
 * TODO:
 *  - two outputs: 1) adsr envelope, and 2) ad/ar or lfo
 */
#include "U8glib.h"

U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NO_ACK);  // Display which does not send AC

#define CHARWIDTH           5
#define CHARHEIGHT          8
#define DISPLAY_WIDTH       128
#define DISPLAY_HEIGHT      64


// Macros for clearing and setting bits
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))


// Interrupt frequency (16,000,000 / 510)
// 510 is divisor rather than 512 since with phase correct PWM
// an interrupt occurs after one up/down count
const float clock = 31372.5;

// OLD
float attack_duration = 0.5, decay_duration = 0.2, sustain_level = 0.75, release_duration = 1.0;


#define TRIGGER_INPUT_PIN 0
#define ADSR_OUTPUT_PIN 9

#define ADSR_ATTACK_INPUT_PIN A0
#define ADSR_DECAY_INPUT_PIN A1
#define ADSR_SUSTAIN_INPUT_PIN A2
#define ADSR_RELEASE_INPUT_PIN A3
#define ADSR_OFFSET_INPUT_PIN A4
#define ADSR_WIDTH_INPUT_PIN A5
#define ADSR_INVERT_INPUT_PIN 1
#define ADSR_REPEAT_INPUT_PIN 2

// ADSR parameters
#define ADSR_TIME_MIN 1000 // micros
#define ADSR_TIME_MAX 8000000 // micros (don't exceed 8.388.608 -> overflow)
#define ADSR_SLOPE 1.0
#define ADSR_TRIGGER_UPDATE_INTERVAL 1000 // micros
int adsr_offset = 0;
int adsr_width = 255;
boolean adsr_invert = false;
boolean adsr_repeat = false;
unsigned long adsrAttackTime = 500000;
unsigned long adsrDecayTime = 500000;
unsigned long adsrSustainLevel = 255;
unsigned long adsrReleaseTime = 500000;
int adsrStartValue;
int adsrValue;

// Trigger
unsigned long lastTriggerUpdateTime = 0;
unsigned long lastTriggerTime = 0;
boolean trigger;


// Draws a printf style string at the current cursor position
void displayln(const char* format, ...)
{
  char buffer[32];
  
  va_list args;
  va_start(args, format);
  vsprintf(buffer, format, args);
  va_end(args);
  
  int len = strlen(buffer);
  for (uint8_t i = 0; i < len; i++) {
    display.write(buffer[i]);
  }
}

void drawEnvelope()
{
  int attack_width = DISPLAY_WIDTH*attack_duration/4.0, decay_width = DISPLAY_WIDTH*decay_duration/4.0, sustain_height = (DISPLAY_HEIGHT-1)*sustain_level+1, release_width = DISPLAY_WIDTH*release_duration/4.0;
  int x0, x1, s;
  x0 = 0;
  x1 = attack_width;
  display.drawLine(x0, DISPLAY_HEIGHT-1, x1, 0, WHITE);
  for (int y=DISPLAY_HEIGHT-1; y>0; y-=4)
    display.drawPixel(x1, y, WHITE);
  x0 = x1;
  x1 = x1+decay_width;
  s = DISPLAY_HEIGHT-sustain_height;
  display.drawLine(x0, 0, x1, s, WHITE);
  x0 = x1;
  x1 = DISPLAY_WIDTH-release_width;
  display.drawFastHLine(x0, s, x1-x0, WHITE);
  for (int y=DISPLAY_HEIGHT-1; y>s; y-=4) {
    display.drawPixel(x0, y, WHITE);
    display.drawPixel(x1, y, WHITE);
  }
  display.drawLine(x1, s, DISPLAY_WIDTH-1, DISPLAY_HEIGHT-1, WHITE);
}

void drawInverseEnvelope()
{
  drawEnvelope();
}

/******************************************************/

void setup() {
  // Set up the display
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // Initialize with the I2C addr 0x3D (for the 128x64)
  display.setTextColor(WHITE);

  pinMode(TRIGGER_INPUT_PIN, INPUT);
  analogWrite(ADSR_OUTPUT_PIN, 0);
  Serial.begin(9600);
}

void loop() {
  unsigned long curTime = micros();

  // Read parameter values every PARAM_READ_INTERVAL ms
  if (curTime / 1000 > lastParamTime + PARAM_READ_INTERVAL) {
    readParameters();
    updateDisplay();
    lastParamTime = curTime / 1000;
  }

  // Check trigger
  if (curTime > lastTriggerUpdateTime + ADSR_TRIGGER_UPDATE_INTERVAL) {
    boolean triggerVal = digitalRead(TRIGGER_INPUT_PIN);
    if (triggerVal != trigger) {
      // Trigger changed!
      trigger = triggerVal;
      lastTriggerTime = curTime;
    }

    lastTriggerUpdateTime = curTime;
  }

  // Set ADSR voltage
  adjustADSR(curTime);
}

void adjustADSR(long curTime) {
  long position = curTime - lastTriggerTime;

  if (position == 0) {
    adsrStartValue = adsrValue;
  }
  if (adsr_repeat && trigger) {
    position %= (adsrAttackTime + adsrDecayTime + adsrReleaseTime);
  }

  if (trigger) {
    // Trigger on
    if (position < adsrAttackTime) {
      adsrValue = map(position, 0, adsrAttackTime, adsrStartValue, 255);
    } else if (position < (adsrAttackTime + adsrDecayTime)) {
      adsrValue = map(position - adsrAttackTime, 0, adsrDecayTime, 255, adsrSustainLevel);
    } else if (adsr_repeat && position < (adsrAttackTime + adsrDecayTime + adsrReleaseTime))  {
      adsrValue = map(position - adsrAttackTime - adsrDecayTime, 0, adsrReleaseTime, adsrSustainLevel, 0); // Release
    } else {
      adsrValue = adsrSustainLevel;
    }
  } else {
    // Trigger off
    if (position < adsrReleaseTime) {
      adsrValue = map(position, 0, adsrReleaseTime, adsrStartValue, 0);
    } else {
      adsrValue = 0;
    }
  }

  int value = map(adsrValue, 0, 255, 0, adsr_width); // Scale for width
  value = map(value, 0, 255, adsr_offset, 255); // Adjust for offset
  if (adsr_invert) {
    value = 255 - value;
  }
  analogWrite(ADSR_OUTPUT_PIN, value);
}

void updateDisplay() {
  display.clearDisplay();
  display.setCursor(10,10);
  displayln("%d", gateState);
  drawEnvelope();
  display.display();
}

void readParameters() {
  int value = analogRead(ADSR_ATTACK_INPUT_PIN);
  adsrAttackTime = slopeMap(value, ADSR_TIME_MIN, ADSR_TIME_MAX, ADSR_SLOPE);
  value = analogRead(ADSR_DECAY_INPUT_PIN);
  adsrDecayTime = slopeMap(value, ADSR_TIME_MIN, ADSR_TIME_MAX, ADSR_SLOPE);
  value = analogRead(ADSR_SUSTAIN_INPUT_PIN);
  adsrSustainLevel = map(value, 0, analog_input_max, 0, 255);
  value = analogRead(ADSR_RELEASE_INPUT_PIN);
  adsrReleaseTime = slopeMap(value, ADSR_TIME_MIN, ADSR_TIME_MAX, ADSR_SLOPE);
  value = analogRead(ADSR_OFFSET_INPUT_PIN);
  adsr_offset = map(value, 0, analog_input_max, 0, 255);
  value = analogRead(ADSR_WIDTH_INPUT_PIN);
  adsr_width = map(value, 0, analog_input_max, 0, 255);
  value = digitalRead(ADSR_INVERT_INPUT_PIN);
  adsr_invert = value != 0;
  value = digitalRead(ADSR_REPEAT_INPUT_PIN);
  adsr_repeat = value != 0;
}


/*
 * Own map() implementation that supports big long's and slope.
 */
long slopeMap(long paramValue, long min, long max, float slope) {
  long value = min + (max - min) / analog_input_max * paramValue;
  float index = (float)paramValue / analog_input_max;
  return value - ((max - min) * slope * index * (1.0 - pow(index, 2)));
}

long longMapFrom1024(long paramValue, long min, long max) {
  return min + (max - min) / analog_input_max * paramValue;
}

long longMapTo256(long paramValue, long inMin, long inMax) {
  return (paramValue - inMin) * 255 / (inMax - inMin);
}

