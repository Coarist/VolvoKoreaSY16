#ifndef VAUXHALLSTALK_INTERNAL_H
#define VAUXHALLSTALK_INTERNAL_H
#include "vauxhall_stalk.h"

#define KEY_RELEASE_TIMEOUT   350

typedef enum
{
  MENUNAVI_NONE,
  MENUNAVI_MAIN,
  MENUNAVI_SETTINGS,
  MENUNAVI_WHEELRIGHT,
  MENUNAVI_WHEELLEFT,
  MENUNAVI_LEFT_ARROW,
  MENUNAVI_RIGHT_ARROW,
  MENUNAVI_BC,
  MENUNAVI_OK
}TMENUNAVI_BUTTON;

typedef struct
{
  BUTTON button;
  BUTTON old_button;
  BUTTON last_button;
  TMENUNAVI_BUTTON menunavi_button;
  TMENUNAVI_BUTTON old_menunavi_button;
  u16 button_held_timer;
  u16 key_release_timer;
  u16 last_button_time;
  u8 navibutton_timer;
  u8 navibutton_counter;
  u8 extend_button_timer;
}TVX_STALK;

TVX_STALK vx_stalk;

VAUXHALL_STALK VauxhallStalk;

static void process_input_keys(void);
static void process_menunavi_buttons(void);
static void send_key_can_packet (u8 byte1,u8 byte2,u8 byte3);
static void process_keypresses(void);

#endif


