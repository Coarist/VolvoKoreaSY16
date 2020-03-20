#include <ior8c22_23.h>
#include <intrinsics.h>
#include "common.h"
#include "global.h"
#include "radioside.h"
#include "vauxhall_stalk.h"
#include "diags.h"


#define ANALOG_BUTTON_HOLD_TIME   100  
#define ANALOG_BUTTON_GAP_TIME    100

#define PHONE_HOLD_TIME 1500
#define KEYBUFFSIZE   30

u8  key_buff[KEYBUFFSIZE],key_in = 0,key_out = 0;

u16  timer_count;
u8  timer_divider;
u8  timer_modify;
u8  timer_mod = 0;

struct t
{
  u8  sending;
  u8  stage;
  u8  length;
  u8  bitpos;
  u8  bytepos;
};

struct t txbuff;

/////////////////////////////////////////////////////////////////////////////////////////////////////

void SetButton(void)
{
  static u16 old_speed = 0;
  static u8 LastKeySent;
  u32 timer_temp;
  
  if (!txbuff.sending)
    do_button();        // get the next button press to send out
  else
    tx();
  
  switch ( VauxhallStalk.Button )
  {
    case BUTTON_VOICE:
      if ( !VauxhallStalk.TimeHeld )
      { // just been pressed
        if ( LastKeySent != RELEASE )
        {
          add_key(RELEASE);
        }
        add_key(PRE_UP);
        LastKeySent = PRE_UP;
      }
      break;
    case BUTTON_HANGUP:
      if ( !VauxhallStalk.TimeHeld )
      {
        if ( LastKeySent != RELEASE )
        {
          add_key(RELEASE);
        }
        add_key(SOURCE);
        LastKeySent = SOURCE;
      }
      break;
    case BUTTON_TRACKUP:
      if ( !VauxhallStalk.TimeHeld )
      {
        if ( LastKeySent != RELEASE )
        {
          add_key(RELEASE);
        }
        add_key(TRACK_UP);
        LastKeySent = TRACK_UP;
      }
      break;
    case BUTTON_TRACKDOWN:
      if ( !VauxhallStalk.TimeHeld )
      {
        if ( LastKeySent != RELEASE )
        {
          add_key(RELEASE);
        }
        add_key(TRACK_DOWN);
        LastKeySent = TRACK_DOWN;
      }
      break;
    case BUTTON_VOLUP:
      if ( !VauxhallStalk.TimeHeld )
      {
        if ( LastKeySent != RELEASE )
        {
          add_key(RELEASE);
        }
        add_key(VOL_UP);
        LastKeySent = VOL_UP;
      }
      break;
    case BUTTON_VOLDOWN:
      if ( !VauxhallStalk.TimeHeld )
      {
        if ( LastKeySent != RELEASE )
        {
          add_key(RELEASE);
        }
        add_key(VOL_DOWN);
        LastKeySent = VOL_DOWN;
      }
      break;
    case BUTTON_NONE:
      if ( !VauxhallStalk.TimeHeld )
      {
        add_key(RELEASE);
        LastKeySent = RELEASE;
      }
      break;
  }
  
  if (global.ignition ==1)
  {
    IGNITION = 1;
  }
  else
  {
    IGNITION = 0;
  }
  
  if (global.illumination == 1)
  {
    ILLUM = 1;
  }
  else
  {
    ILLUM = 0;
  }
  
  if (global.reverse == 1)
  {
    REVERSE = 1;
  }
  else
  {
    REVERSE = 0;
  }
  
  if (global.parkbrake == 1)
  {
    PARK = 1;
  }
  else
  {
    PARK = 0;
  }
  
  // now do the speed pulse
  if (global.speed != old_speed ) // need to modify timer
  {
    timer_modify = 0;
    old_speed = global.speed;
    if (!global.speed)    // speed is zero!
    {
      timer_divider = 0;
      timer_count = 0xFFFF;
    }
    else if (global.speed < 6)   // use divider
    {
      timer_divider = 5;
      timer_temp = (u32)100000 / (u32)(global.speed<<1);
      timer_temp -= 4;
      timer_count = timer_temp & 0xFFFF;
    }
    else                    // no divider , normal interrupt
    {
      timer_divider = 1;
      timer_temp = (u32)500000 / (u32)(global.speed<<1);
      timer_temp -= 4;
      timer_count = timer_temp & 0xFFFF;
    }
    timer_modify = 1;
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void do_button(void)
{
  u8 key;
  
  key = get_key();
  if (key == 0xFF)
  {
    return;
  }
  switch (key)
  {
  case VOL_UP:
    DEBUG("VOL+\r\n");
    break;
  case VOL_DOWN:
    DEBUG("VOL-\r\n");
    break;
  case TRACK_UP:
    DEBUG("TRK+\r\n");
    break;
  case TRACK_DOWN:
    DEBUG("TRK-\r\n");
    break;
  case SOURCE:
    DEBUG("SOURCE\r\n");
    break;
  case PICKUP:
    DEBUG("PICKUP\r\n");
    break;
  case HANGUP:
    DEBUG("HANGUP\r\n");
   break;
  case RELEASE:
    DEBUG("RELEASE\r\n");
    break;
  }
  switch (key)
  {
  case VOL_UP:
    while (PD0_bit.PD0_3 == 0)  // keep trying till it's set!
    {
      PRCR =4;                // unprotect port 0
      PD0_bit.PD0_3 = 1;
    }
    P0_bit.P0_3 = 0;
    global.delay = ANALOG_BUTTON_HOLD_TIME;
    txbuff.sending = 1;
    break;
  case VOL_DOWN:
    while (PD0_bit.PD0_2 == 0)  // keep trying till it's set!
    {
      PRCR =4;                // unprotect port 0
      PD0_bit.PD0_2 = 1;
    }
    P0_bit.P0_2 = 0;
    global.delay = ANALOG_BUTTON_HOLD_TIME;
    txbuff.sending = 1;
    break;
  case TRACK_UP:
    while (PD0_bit.PD0_4 == 0)  // keep trying till it's set!
    {
      PRCR =4;                // unprotect port 0
      PD0_bit.PD0_4 = 1;
    }
    P0_bit.P0_4 = 0;
    global.delay = ANALOG_BUTTON_HOLD_TIME;
    txbuff.sending = 1;
    break;
  case TRACK_DOWN:
    PD6_bit.PD6_0 = 1;
    P6_bit.P6_0 = 0;
    global.delay = ANALOG_BUTTON_HOLD_TIME;
    txbuff.sending = 1;
    break;
  case SOURCE:
    while (PD0_bit.PD0_6 == 0)  // keep trying till it's set!
    {
      PRCR =4;                // unprotect port 0
      PD0_bit.PD0_6 = 1;
    }
    P0_bit.P0_6 = 0;
    global.delay = ANALOG_BUTTON_HOLD_TIME;   // 200mS hold for Sony
    txbuff.sending = 1;
    break;
  case PRE_UP:
    PD1_bit.PD1_1 = 1;
    P1_bit.P1_1 = 0;
    while (PD0_bit.PD0_4 == 0)  // keep trying till it's set!
    {
      PRCR =4;                // unprotect port 0
      PD0_bit.PD0_4 = 1;
    }
    P0_bit.P0_4 = 0;
    global.delay = ANALOG_BUTTON_HOLD_TIME;
    txbuff.sending = 1;
    break;      
  case PICKUP:
    PD1_bit.PD1_1 = 1;          // shift
    P1_bit.P1_1 = 0;
    while (PD0_bit.PD0_5 == 0)  // keep trying till it's set!
    {
      PRCR =4;                  // unprotect port 0
      PD0_bit.PD0_5 = 1;
    }
    P0_bit.P0_5 = 0;
    global.delay = ANALOG_BUTTON_HOLD_TIME;
    txbuff.sending = 1;
    break;
  case HANGUP:
    PD1_bit.PD1_1 = 1;          // shift
    P1_bit.P1_1 = 0;
    PD3_bit.PD3_3 = 1;          // DISP
    P3_bit.P3_3 = 0;
    global.delay = ANALOG_BUTTON_HOLD_TIME;
    txbuff.sending = 1;
    break;

  case RELEASE:
    PD6_bit.PD6_0 = 0;      // set Track Down pin back to input
    PD1_bit.PD1_2 = 0;      // set Power back to input
    PD1_bit.PD1_1 = 0;      // set Shift back to input
    while (PD0 & 0x5C)      // keep trying till they're clear!
    {
      PRCR =4;                // unprotect port 0
      PD0 &= 0xA3;            // set Track up, vol up & vol down back to inputs
    }
    global.delay = ANALOG_BUTTON_GAP_TIME;
    txbuff.sending = 1;
    break;
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void release_all_buttons(void)
{
  PD6_bit.PD6_0 = 0;      // set Track Down pin back to input
  PD1_bit.PD1_2 = 0;      // set Power back to input
  PD1_bit.PD1_1 = 0;      // set Shift back to input
  while (PD0 & 0x5C)      // keep trying till they're clear!
  {
    PRCR =4;                // unprotect port 0
    PD0 &= 0xa3;            // set Track up, preset up, vol up & vol down back to inputs
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void tx(void)
{
  if ( global.delay )
    global.delay--;
  else
  {
    txbuff.sending = 0;
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void add_key(u8 key)
{
  key_buff[key_in] = key;
  if (key_in)
    key_in--;
  else
    key_in = KEYBUFFSIZE-1;
}

//////////////////////////////////////////////////////////////////////////////////////////////////

u8 get_key(void)
{
  u8 temp;
  if (key_in == key_out)
    return 0xff;          // no key presses to return
  temp = key_buff[key_out];
  if (key_out)
    key_out--;
  else
    key_out = KEYBUFFSIZE-1;
  return temp;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////                SPEED PULSE TIMER INTERRUPT               //////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma vector = 8
static __interrupt void TimerRD0Intr (void)
{
  static u8 divider = 0,divider_set = 0;
    TRD0 = 0;                 // reset counter
    if (TRDSR0) TRDSR0 = 0;
    TRDSTR = 1;           // start the timer now!
 
    if ( divider )            // divider used for slower speed pulses
    {
        divider--;
    }
    if ( !divider )
    {  
        divider = divider_set;
        if ( divider_set )    // only toggle pin if speed is > 0 
            SPEED ^= 1;
    }
    if ( timer_modify )
    {
        TRDGRA0 = timer_count;          // load new value into compare register
        if ( divider_set != timer_divider )
            divider = timer_divider;    // update new divider if different to last
        divider_set = timer_divider;    // set if speed > 0
        timer_modify = 0;    
    }   
}

