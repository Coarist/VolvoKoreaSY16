#include "diags.h"
#include <ior8c22_23.h>
#include <intrinsics.h>

#define DIAGBUFFERSIZE  256

#ifdef DIAGS_ENABLED
static struct
{
  char buffer[DIAGBUFFERSIZE];
  u16 in;
  u16 out;
} diag;
#endif
/********************************************************************************************************************************/

void InitDiags(void)
{
#ifdef DIAGS_ENABLED
  PD6 &= 0x3F;        // make RXD1 & TXD1 inputs
  U1BRG = 25;        // set divider as 26 for 38400 baud
  U1MR = 5;           // set UART for 8 bits, internal clock, 1 stop bit & no parity
  U1C0 = 0;           // select f1 as count source & LSB first
  U1C1 = 0x05;        // enable Tx & Rx, set transmit int as buffer empty & receive int as continuous
  U1SR = 3;           // selects UART1
  PMR = 0x10;         // use TXD1 & RXD1 pins
  // set up interrupt
//  S1RIC = 2;
  diag.in = diag.out = 0;

#endif
}
/********************************************************************************************************************************/

void DiagsProcessing(void)
{
#ifdef DIAGS_ENABLED
  if ( diag.in != diag.out ) // there is data to send
  {
    if ( U1C1 & 2 ) // there is room in the uart for the next byte
    {
      U1TB = diag.buffer[diag.out];
      if ( diag.out )
        diag.out--;
      else
        diag.out = DIAGBUFFERSIZE-1;
    } 
  }
#endif
}
/********************************************************************************************************************************/

void SendDiag( char * text)
{
#ifdef DIAGS_ENABLED
  while ( *text )
  {
    diag.buffer[diag.in] = *text++;
    if ( diag.in )
      diag.in--;
    else
      diag.in = DIAGBUFFERSIZE-1;
  }
#endif
}
/********************************************************************************************************************************/



