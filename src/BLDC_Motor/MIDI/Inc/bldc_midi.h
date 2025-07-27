#ifndef __BLDC_MIDI_H__
#define __BLDC_MIDI_H__

#include <stdint.h>
#include "FOC_utils.h"

#define MAX_NOTES 3
#define SAMPLE_RATE 10000
#define MAX_TONE_PIANO 21


void audio_loop(foc_t *hfoc, float e_theta_rad);

#endif