#ifndef _WaveTable_h_
#define _WaveTable_h_

#include <tables/triangle2048_int8.h>
#include <tables/saw2048_int8.h>
#include <tables/square_no_alias_2048_int8.h>
#include <tables/sin2048_int8.h>

#define SAW_WAVE SAW2048_DATA
#define TRIANGLE_WAVE TRIANGLE2048_DATA
#define SINE_WAVE SIN2048_DATA
#define SQUARE_WAVE SQUARE_NO_ALIAS_2048_DATA

#define WAVE_NUM_CELLS 2048

class WaveTable {
  Oscil<WAVE_NUM_CELLS, AUDIO_RATE> *osc1, *osc2, *osc3, *osc4;
  int shape; // 0 to 1023, 10 bits
  int s1, s2, s3, s4;

public:
  WaveTable() {
    osc1 = new Oscil<WAVE_NUM_CELLS, AUDIO_RATE>(SAW_WAVE);
    osc2 = new Oscil<WAVE_NUM_CELLS, AUDIO_RATE>(TRIANGLE_WAVE);
    osc3 = new Oscil<WAVE_NUM_CELLS, AUDIO_RATE>(SINE_WAVE);
    osc4 = new Oscil<WAVE_NUM_CELLS, AUDIO_RATE>(SQUARE_WAVE);
    shape = 0;
    s1 = 256;
    s2 = s3 = s4 = 0;
  }

  void setShape(int value)
  {
    shape = value;
    if (value < 256) {
      s1 = 255 - value;
      s2 = value;
      s3 = 0;
      s4 = 0;
    } else {
      s1 = 0;
      value -= 256;
      if (value < 256) {
        s2 = 255 - value;
        s3 = value;
        s4 = 0;
      } else {
        s2 = 0;
        value -= 256;
        if (value < 256) {
          s3 = 255 - value;
          s4 = value;
        } else {
          s3 = 0;
          value -= 256;
          s4 = 255 - value;
          s1 = value;
        }
      }
    }
  }

  inline
  void setFreq(float frequency)
  {
    osc1->setFreq(frequency);
    osc2->setFreq(frequency);
    osc3->setFreq(frequency);
    osc4->setFreq(frequency);
  }

  inline
  void setFreq_Q16n16(Q16n16 frequency)
  {
    osc1->setFreq_Q16n16(frequency);
    osc2->setFreq_Q16n16(frequency);
    osc3->setFreq_Q16n16(frequency);
    osc4->setFreq_Q16n16(frequency);
  }

  inline
  void setFreq_Q24n8(Q24n8 frequency)
  {
    osc1->setFreq_Q24n8(frequency);
    osc2->setFreq_Q24n8(frequency);
    osc3->setFreq_Q24n8(frequency);
    osc4->setFreq_Q24n8(frequency);
  }

  inline
  int8_t next()
  {
    return (osc1->next() * s1 + osc2->next() * s2 + osc3->next() * s3 + osc4->next() * s4) >> 8;
  }

  inline
  int8_t phMod(Q15n16 phmod_proportion)
  {
    return (osc1->phMod(phmod_proportion) * s1 + osc2->phMod(phmod_proportion) * s2 + osc3->phMod(phmod_proportion) * s3 + osc4->phMod(phmod_proportion) * s4) >> 8;
  }
};

#endif

