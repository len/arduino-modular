/**
 * 
 * TODO:
 *   - synchronize to external clock?
 *   - reset (pulse input)
 *   - hold (gate input, holds voltage)
 *   - polarity option (0~+5v, -5~+5v, inverted?)
 *   - output additional square waves through some digital pins
 *   - 2 pots for continuous wave shape (like wavetable vco)
 *   - 2 level pots? 2 offset pots?
 *   - 
 */

#include "U8glib.h"

U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NO_ACK);  // Display which does not send AC

#define CHARWIDTH           5
#define CHARHEIGHT          8
#define DISPLAY_WIDTH       128
#define DISPLAY_HEIGHT      64

#include "avr/pgmspace.h"

// Waves
#include "ramp256.h"
#include "saw256.h"
#include "tri256.h"
#include "sine256.h"
#include "sq256.h"

// Macros for clearing and setting bits
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

// Interrupt frequency (16,000,000 / 510)
// 510 is divisor rather than 512 since with phase correct PWM
// an interrupt occurs after one up/down count
const float clock = 31372.5;

#define MODE_SWITCH_PIN 8
#define LFO1_FREQUENCY_PIN A0
#define LFO1_SHAPE_PIN A3
#define LFO1_OUTPUT_PIN 11 //OCR2A
#define LFO1_SQUARE_OUTPUT_PIN 12
#define LFO2_FREQUENCY_PIN A2
#define LFO2_PHASE_PIN A1
#define LFO2_SHAPE_PIN A4
#define LFO2_OUTPUT_PIN 3 //OCR2B
#define LFO2_SQUARE_OUTPUT_PIN 4

#define MIN_INCREMENT 8192

// Wave table pointers
byte *waveTables[] = {ramp256, saw256, tri256, sine256, sq256};
#define NUM_WAVES (sizeof(waveTables) / sizeof(byte *))

// Interrupt vars are volatile
volatile byte tickCounter;               // Counts interrupt "ticks". Reset every 125  
volatile byte fourMilliCounter;          // Counter incremented every 4ms

volatile unsigned int LFO1_shape = 0;
volatile unsigned long LFO1_increment;
volatile unsigned long LFO1_accumulator=0;
volatile byte LFO1_offset;
volatile unsigned int LFO1_depth;

volatile unsigned int LFO2_shape = 0;
volatile unsigned long LFO2_increment;
volatile unsigned long LFO2_accumulator;
volatile byte LFO2_offset;
volatile byte LFO2_phase = 0;
volatile unsigned int LFO2_depth;

volatile byte sync_mode = 1;


inline
byte wavetable_read(unsigned int shape, byte offset)
{
  byte i = shape >> 8; // 2 bits
  byte s = shape & 0xff; // 8 bits
  return (pgm_read_byte_near(waveTables[i] + offset) * (0xff - s) + pgm_read_byte_near(waveTables[i+1] + offset) * s) >> 8;
}


void displayf(int x, int y, const char* format, ...)
{
  char buffer[32];
  
  va_list args;
  va_start(args, format);
  vsprintf(buffer, format, args);
  va_end(args);
  
  u8g.drawStr(x, y, buffer);
}

void drawWaveforms0()
{
  int i=0;
  for (int x=0; x<128; x++) {
    u8g.drawPixel(x,31 - wavetable_read(LFO1_shape, i) / 12);
    u8g.drawPixel(x,63 - wavetable_read(LFO2_shape, i + LFO2_phase & 0xff) / 12);
    i += 2;
  }
}

void drawWaveforms()
{
  byte i=0,j=LFO2_phase,di,dj,yi,yj,xpos;
  if (!sync_mode)
    return drawWaveforms0();
  if (LFO1_increment >= LFO2_increment) {
//    xpos = LFO1_offset/2;
    di = LFO1_increment * 2 / LFO2_increment;
    dj = 2;
  } else {
//    xpos = LFO2_offset/2;
    di = 2;
    dj = LFO2_increment * 2 / LFO1_increment;
  }
  for (byte x=0; x<128; x++) {
    yi = 31 - wavetable_read(LFO1_shape, i) / 12;
    yj = 63 - wavetable_read(LFO2_shape, j) / 12;
/*    if (x == xpos) {
      u8g.drawVLine(x,yi-1,3);
      u8g.drawVLine(x,yj-1,3);
    } else {*/
      u8g.drawPixel(x,yi);
      u8g.drawPixel(x,yj);
//    }
    i += di;
    j += dj;
  }
}

void updateDisplay0()
{
  u8g.setFont(u8g_font_unifont);
  //u8g.setFont(u8g_font_osb21);
  displayf(0,15, "%ld", LFO1_increment);
  displayf(0,31, "%ld", LFO2_increment);
  displayf(0,47, "%d", wavetable_read(LFO1_shape, LFO1_offset));
  displayf(0,63, "%ld", LFO1_offset);
}

void updateDisplay()
{
  u8g.firstPage();  
  do {
    drawWaveforms();
//    updateDisplay0();
  } while( u8g.nextPage() );
}

void setup()
{
  u8g.setColorIndex(1);

  // PWM Pins
  pinMode(LFO1_OUTPUT_PIN, OUTPUT);     // pin11= PWM:A
  pinMode(LFO2_OUTPUT_PIN, OUTPUT);     // pin 3= PWM:B

  // Initialize timers
  setup_timer2();
}

void loop()
{
  while(1) {
    if (fourMilliCounter > 25) {                 // Every 1/10 second
      fourMilliCounter=0;

      LFO1_shape = analogRead(LFO1_SHAPE_PIN);
      LFO2_shape = analogRead(LFO2_SHAPE_PIN);

      // LFO 1
      LFO1_increment = pow(1.01846, analogRead(LFO1_FREQUENCY_PIN)) + MIN_INCREMENT;
//      LFO1_depth = 8 - (analogRead(LFO1_DEPTH_PIN) >> 7);

      // LFO 2
      if (sync_mode) {
        int multiplier_table[] = {1,2,3,4,5,8};
        int m = analogRead(LFO2_FREQUENCY_PIN) * 11 / 1024;
        LFO2_increment = m<=5 ? LFO1_increment * multiplier_table[5-m] : LFO1_increment / multiplier_table[m-5];
      } else {
        LFO2_increment = pow(1.01846, analogRead(LFO2_FREQUENCY_PIN)) + MIN_INCREMENT;
      }
//      LFO2_depth = 8 - (analogRead(LFO2_DEPTH_PIN) >> 7);
      LFO2_phase = analogRead(LFO2_PHASE_PIN) >> 2;
      
      updateDisplay();
    }
  }
}

//******************************************************************
// timer2 setup
void setup_timer2() {
  // Prescaler 1
  sbi (TCCR2B, CS20);
  cbi (TCCR2B, CS21);
  cbi (TCCR2B, CS22);

  // Non-inverted PWM
  cbi (TCCR2A, COM2A0);
  sbi (TCCR2A, COM2A1);
  cbi (TCCR2A, COM2B0);
  sbi (TCCR2A, COM2B1);

  // Phase Correct PWM
  sbi (TCCR2A, WGM20);
  cbi (TCCR2A, WGM21);
  cbi (TCCR2B, WGM22);

  // Enable interrupt
  sbi (TIMSK2,TOIE2);
}

////////////////////////////////////////////////////////////////
//
// Timer2 Interrupt Service
// Frequency = 16,000,000 / 510 = 31372.5
//
ISR(TIMER2_OVF_vect) {
  unsigned long temp;

  // Count every four milliseconds
  if(tickCounter++ == 125) {
    fourMilliCounter++;
    tickCounter=0;
  }

  if (reset) {
    LFO1_accumulator = LFO2_accumulator = 0;
    reset = 0;
  } else {
    LFO1_accumulator += LFO1_increment;
    LFO2_accumulator += LFO2_increment;
  }
  
  LFO1_offset       = LFO1_accumulator >> 24; // high order byte
  temp              = wavetable_read(LFO1_shape, LFO1_offset);
/*  temp              = LFO1_base - (temp >> LFO1_depth);*/
  OCR2A             = temp;

  LFO2_offset        = LFO2_accumulator >> 24; // high order byte
  temp               = wavetable_read(LFO2_shape, (LFO2_offset + LFO2_phase) & 0xff);
/*  temp               = LFO2_base + (temp >> LFO2_depth);
  OCR2B              = (temp > 255) ? 255 : temp; */
  OCR2B              = temp;
}

