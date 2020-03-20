#ifndef GLOBAL_H
#define GLOBAL_H

#include "common.h"

#define NONE        0
#define VOL_UP      1
#define VOL_DOWN    2
#define TRACK_UP    3
#define TRACK_DOWN  4
#define SOURCE      5
#define PRE_UP      6
#define PRE_UP_HOLD 7
#define PICKUP      8
#define HANGUP      9

#define RELEASE     128

#define ENFT      P3_bit.P3_1
#define ERRFT     P1_bit.P1_0
#define STB       P3_bit.P3_0
#define REVERSE   P0_bit.P0_7
#define SPEED     P6_bit.P6_3
#define IGNITION  P6_bit.P6_5
#define ILLUM     P6_bit.P6_4
#define PARK      P3_bit.P3_5
#define IROUT     P1_bit.P1_2
#define CONTOUT   P1_bit.P1_1
#define MUTESENSE P3_bit.P3_7

typedef enum
{
  DISPLAY_MODE_UNKNOWN,
  DISPLAY_MODE_RADIO,
  DISPLAY_MODE_BC,
  DISPLAY_MODE_SETTINGS
}TDISPLAY_MODE;

struct global_def
{
  u8  ignition;           // car outputs
  u8  illumination;
  u8  reverse;
  u16 speed;
  u8  parkbrake;
  u16 delay;
  u16 sleeptimer;         // sleep
  u8  sleep;
  u8  count;
  u8  hold;
  u8  held_button;
  u8  cartype;
  u16 timeout;
  u8  wait;
  bool phone_kit_present;
  TDISPLAY_MODE display_mode;
};

extern struct global_def global;

#endif