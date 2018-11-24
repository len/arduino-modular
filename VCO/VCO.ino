/*
 * VCO based on wavetables.
 * 
 * - 2 oscillators with continuously variable shape interpolating between 4 waveforms
 * - unison mode with detuning of the second oscillator
 * - FM and AM modes, the second oscillator modulates the first
 * - ring modulation mode
 * 
 */

/*#include "U8glib.h"

U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NO_ACK);  // Display which does not send AC

#define CHARWIDTH           5
#define CHARHEIGHT          8
#define DISPLAY_WIDTH       128
#define DISPLAY_HEIGHT      64
*/

#include <MozziGuts.h>
//#define OSCIL_DITHER_PHASE 1
#include <Oscil.h>
#include <RollingAverage.h>

#define CONTROL_RATE 64 // powers of 2 please

#define FREQ_COARSE_PIN A0
#define FREQ_FINE_PIN A5
#define FREQ_VOCT_PIN A6
#define SHAPE1_PIN A1
#define SHAPE2_PIN A2
#define RATIO_PIN A3
#define PARAM_PIN A3 // modulation parameter (index in FM, depth in AM) or detune in dual osc mode
#define INT_MOD_PIN D3

#define NO_MODULATION 0
#define AMPLITUDE_MODULATION 1
#define FREQUENCY_MODULATION 2
#define RING_MODULATION 3
//#define SHAPE_MODULATION 4

#include "WaveTable.h"

WaveTable wavetable1, wavetable2;

int internal_modulation = NO_MODULATION;

Q24n8 freq1; // unsigned long with 24 integer bits and 8 fractional bits
Q8n8 ratio = float_to_Q8n8(3.0f); // freq2 to freq1 ratio, unsigned int with 8 integer bits and 8 fractional bits
Q24n8 freq2; // unsigned long with 24 integer bits and 8 fractional bits

Q8n8 fm_index = float_to_Q8n8(2.0f); // constant version
Q16n16 fm_deviation;
Q8n8 am_index = float_to_Q8n8(1.0f);
int shape1, shape2;

RollingAverage <int, 16> freq_coarse_average;
RollingAverage <int, 16> ratio_average;


void setup() {
  startMozzi(CONTROL_RATE);
  Serial.begin(9600);
}


void updateControl() {
  unsigned int v;
  // compute frequency from coarse and fine pots and 1v/oct cv input
  v = freq_coarse_average.next(mozziAnalogRead(FREQ_COARSE_PIN));
//  Serial.println(v);
/*  // 1024 -> 440*16, 512 -> 440, 0 -> 440/16
  freq1 = 440L << ((v >> 7) + 4);
  freq1 = (freq1 * (128 + (v & 127))) >> 7;*/

  // 1024 -> 440*16, 512 -> 440, 0 -> 440/16
  freq1 = 440L << ((v >> 7) + 4);
  freq1 = (freq1 * (128 + (v & 127))) >> 7;
//  Serial.println(freq1);
//  freq1 += mozziAnalogRead(FREQ_FINE_PIN);
//  freq1 += mozziAnalogRead(FREQ_VOCT_PIN);
  // read parameters
  shape1 = mozziAnalogRead(SHAPE1_PIN);
  shape2 = mozziAnalogRead(SHAPE2_PIN);
  ratio = (ratio_average.next(mozziAnalogRead(RATIO_PIN)) >> 1) + 1; // discarding one bit of information, although it might be mostly noise
  freq2 = (freq1 * ratio) >> 8; // (Q24n8   Q8n8) >> 8 = Q24n8, beware of overflow
  switch (internal_modulation) {
    case FREQUENCY_MODULATION:
      fm_index = mozziAnalogRead(PARAM_PIN);
      fm_deviation = (freq2>>8) * fm_index; // (Q24n8>>8)   Q8n8 = Q24n8, beware of overflow
      break;
    case AMPLITUDE_MODULATION:
      am_index = mozziAnalogRead(PARAM_PIN);
      break;
    case RING_MODULATION:
    case NO_MODULATION:
      // detune
      break;
  }
  wavetable1.setFreq_Q24n8(freq1);
  wavetable2.setFreq_Q24n8(freq2);
  wavetable1.setShape(shape1);
  wavetable2.setShape(shape2);
}


int updateAudio() {
  Q15n16 fm_mod;
  long am_mod;
  
  switch (internal_modulation) {
    case FREQUENCY_MODULATION:
      fm_mod = fm_deviation * wavetable2.next() >> 8;
      return (int)wavetable1.phMod(fm_mod) << 6; // wavetable1 is 8 bit wave, so shift up to 14 bits
    case AMPLITUDE_MODULATION:
      am_mod = (128u + wavetable2.next()) * ((byte)128 + am_index);
      return (am_mod * wavetable1.next()) >> 10; // 16 bit * 8 bit = 24 bit, then >>10 = 14 bit
    case RING_MODULATION:
      return (wavetable1.next() * wavetable2.next()) >> 2;
   }
   // no modulation, just add the two waves:
   return (wavetable1.next() + wavetable2.next()) << 5;
}


void loop() {
  audioHook(); // required here
}


