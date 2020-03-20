#include <ior8c22_23.h>
#include <intrinsics.h>
#include "common.h"
#include "can.h"
#include "global.h"
#include "carside.h"
#include "main.h"
#include "radioside.h"
#include "diags.h"

static void ConfigureClock(void);
static void ConfigurePorts(void);
static void ConfigureTimers(void);
static void MSFunctions(void);

struct global_def global;
volatile u8 timer_flag,delay;

__no_init volatile u32 WakeByIgnitionToken @ 0xffc;

// approx 1000 per second
#define IGNONDELAY          2000
#define IGNACTIVEDELAY      10000


////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////  MAIN PROGRAM  //////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

void main( void )
{
  __disable_interrupt();
  InitDiags();
  DEBUG("\r\nVauxhall CAN Stalk to Pioneer Software Start\r\n");
  ConfigureClock();
  ConfigurePorts();
  ConfigureTimers();
  InitCarSide();
  __enable_interrupt();
  cspro = 0;
  cspro = 1;
  wdts = 0xFF;                     // start watchdog timer


  for(;;)
  {
    while( !timer_flag)
    {
      can_int(); // check for any received packets
      DiagsProcessing(); // send out any diags
      // wait here for timer to interrupt and set flag
    }
    timer_flag = 0;     // reset flag
    MSFunctions();
  }
}
/********************************************************************************************************************************/

static void MSFunctions(void)
{
    static bool IgnitionWakeActive = false;
    static bool IgnitionWake = false;
    static u32 IgnitionOnTime = 0;
    static u32 IgnitionOffTime = 0;

    // reset watchdog
    wdtr = 0x00;
    wdtr = 0xFF;

    global.timeout++;

    SetButton();
    CANSide();
    CarSide();

    if (global.sleep)
    {
      CANSleep ();
      STB = 1;                  // switch off CAN tranceiver
      IGNITION =0; // switch everything off before entering slow mode
      ILLUM = 0;
      REVERSE = 0;
      SPEED = 0;
      PARK = 0;
      PD1_bit.PD1_2 = 0;  // set stalk lines back to inputs
      PD1_bit.PD1_1 = 0;
      ////////////////////////// not actually sleep , but run in slow mode on internal oscillator /////////////
      __disable_interrupt();    // switch interrupts off for now
      prcr = 3;                 // unprotect CM0, CM1 and OCD registers ( and PM0 & PM1 for watchdog)
      ocd0 = 0;                 // disable oscillator stop detection
      ocd1 = 0;                 // disable oscillator stop detect interrupt
      ocd2 = 1;                 // select internal oscillator
      cm05 = 1;                 // switch external oscillator off
      prcr = 0;                 // protect registers
//      __enable_interrupt();

      IgnitionWake = false;
      IgnitionWakeActive = false;
      while (P6_bit.P6_2 && (!IgnitionWake) )
      {
        // wait here till the CAN receive line changes( pulls low)
        // reset watchdog
        wdtr = 0x00;
        wdtr = 0xFF;
        if ( P3_bit.P3_7 ) // Ign on ( ATT line )
        {
          IgnitionOffTime = 0;
          IgnitionOnTime++;
          if ( !IgnitionOnTime )
          {
            IgnitionOnTime--;
          }
        }
        else
        {
          IgnitionOnTime = 0;
          if ( IgnitionOffTime < IGNACTIVEDELAY )
          {
            IgnitionOffTime++;
          }
          else
          {
            IgnitionWakeActive = true;
          }
        }
        if ( IgnitionWakeActive && ( IgnitionOnTime > IGNONDELAY ) )
        {
          IgnitionWake = true;
          WakeByIgnitionToken = IGNITIONWAKETOKEN;
        }
      }
      while (1)
      {
        PRCR = 0x02;
        PM0 = 0x04; // try to do a software reset to speed things up
        // wait here till watchdog resets
      }
    }

}
/********************************************************************************************************************************/




/////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////                        START OF CONFIGURATION ROUTINES                    ///////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

/********************************************************************************************************************************/

static void ConfigureClock(void)
{
  // Protect off
  prcr = 3;

  // Xin Xout
  cm13 = 1;

  // XCIN-XCOUT drive capacity select bit : HIGH
  cm15 = 1;

  // Xin on
  cm05 = 0;

  // Main clock = No division mode
  cm16 = 0;

  // Main clock = No division mode
  cm17 = 0;

  // CM16 and CM17 enable
  cm06 = 0;

  // generate reset on underflow
  pm12 = 1;

  // Waiting for stablilisation of oscillator
  asm("nop");
  asm("nop");
  asm("nop");
  asm("nop");

  // Main clock change
  ocd2 = 0;

  // Protect on
  prcr = 0;
}
/********************************************************************************************************************************/

static void ConfigurePorts(void)
{
  // General pins
  PD2 = 0xff;
  P2 = 0;
  PD4 |= 0x18;
  P4 &= 0xe7;
  PD1 |= 0xf8;
  P1 &= 0x07;
  PD3_bit.PD3_7 = 0;      // Attenuate sense set as input
  PRCR =4;                // have to keep unprotecting as it resets bit to 0 after each address write!
  PD0_bit.PD0_7 = 1;      // Reverse drive output
  REVERSE = 0;
  PD6_bit.PD6_3 = 1;      // Speed pulse drive output
  SPEED = 0;
  PD6_bit.PD6_4 = 1;      // Illumination drive output
  ILLUM = 0;
  PD6_bit.PD6_5 = 1;      // Ignition drive output
  IGNITION = 0;
  PD3_bit.PD3_5 = 1;      // Park brake drive output
  PARK = 0;
  // CAN Tranceiver pins
  PD3_bit.PD3_0 = 1;      // Standby output pin
  STB = 0;
}
/********************************************************************************************************************************/

static void ConfigureTimers(void)
{
  // Timers will depend on which radio we using. Use TimerRB for main program flow and IR generation.
  // Use TimerRA for Speed pulse generation.

  // Pioneer will run main loop round a 1mS timer
  TRBMR  = 0x10;    // select f8 as a source
  TRBPRE = 7;       // 8 loops of
  TRBPR = 249;      // 250 counts
  TRBIC = 1;        // enable interrupt
  TRBCR = 1  ;      // start timer
  // The speed pulse timer will only be started when it needs to be!
  TRDSTR = 0;           // don't start the timer yet!
  TRDMR = 0;            // leave mode register as default
  TRDPMR = 0;           // leave PWM mode register as default
  TRDFCR = 0x80;        // upper bit must be set
  TRDOER1 = 0xFF;       // set pins as I/O
  TRDOER2 = 0;          // leave output master enable register 2 as default
  TRDOCR = 0;           // leave output control regaster as default
  TRDCR0 = 0x04;        // select f32 as a count source
  TRDIORA0 = 8;         // set bit 3
  TRDIORC0 = 0x88;      // set pins as I/O
  TRDGRA0 = 0xA2C3;     // value equates to 1Hz. set this as the default
  TRDIER0 = 0x01;       // set the interrupt enable register to trigger on bit A compare only
  TRD0IC = 1;           // enable timer RD channel 0 interrupt
  TRDSTR = 1;           // start the timer now!
}
/********************************************************************************************************************************/

#pragma vector = 24
static __interrupt void TimerRbIntr (void)
{
  timer_flag = 1;
}
/********************************************************************************************************************************/

