#include <Arduino.h>
#include "board.h"
#include "audio.h"

static int pinPWM_;


void audio_Config(int pinPWM) {
  pinPWM_       = pinPWM;
  }

void audio_SetFrequency(int32_t freqHz) {
  if (freqHz > 0) {
    digitalWrite(pinAudioEn, 1);
    tone(pinPWM_, freqHz, 10000);
    }
  else {
    digitalWrite(pinAudioEn, 0);
    noTone(pinPWM_);
    }
  }


void audio_GenerateTone(int32_t freqHz, int ms) {
    digitalWrite(pinAudioEn, 1);
    tone(pinPWM_, freqHz, ms);
    delay(ms);
    digitalWrite(pinAudioEn, 0);
    }
