#pragma once

#ifdef AUDIO_ENABLE
#define STARTUP_SONG SONG(PLANCK_SOUND)
#endif

#define MIDI_BASIC

#define ENCODER_RESOLUTION 4

#define CHORDAL_HOLD
#define USB_SUSPEND_WAKEUP_DELAY 0
#define AUTO_SHIFT_MODIFIERS
#define SERIAL_NUMBER "bVZAA/Xbb0d9"
#define LAYER_STATE_8BIT

#define RGB_MATRIX_STARTUP_SPD 60



// Options not available in Oryx (only QMK)
#define RETRO_SHIFT
