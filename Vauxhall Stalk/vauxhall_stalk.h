#ifndef VAUXHALLSTALK_H
#define VAUXHALLSTALK_H
#include "can.h"

typedef enum
{
  BUTTON_NONE,
  BUTTON_VOLUP,
  BUTTON_VOLDOWN,
  BUTTON_TRACKUP,
  BUTTON_TRACKDOWN,
  BUTTON_HANGUP,
  BUTTON_VOICE
}BUTTON;

typedef struct 
{
  BUTTON Button;
  u16 TimeHeld;
}VAUXHALL_STALK;

extern VAUXHALL_STALK VauxhallStalk;

extern void VauxhallStalkInit(void);
extern void VauxhallStalkSide(void);
extern void process_stalk_packet( TCANPacket * canpkt );

#endif

