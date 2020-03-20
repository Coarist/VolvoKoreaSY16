#ifndef CARSIDEINTERNAL_H
#define CARSIDEINTERNAL_H
#include "common.h"
#include "can.h"
#include "carside.h"
#include "global.h"
#include "diags.h"
#include "vaux_nm.h"
#include "iso15765.h"
#include "string.h"
#include "vauxhall_stalk.h"

// the following line controls the programming sequence and sets if we are going to program the country code or not.
//#define PROGRAM_COUNTRY_CODE


#define CAN_STALK_ID          0x206
#define CAN_IGN_ID            0x450
#define CAN_DISPLAY_MODE_ID   0x2b0
#define CAN_DISPLAY_MODE2_ID  0x696
#define CAN_RADIO_ISO_RX      0x2c1
#define CAN_RADIO_ISO_TX      0x6c1
#define CAN_DIAGS_ISO_RX      0x241
#define CAN_DIAGS_ISO_TX      0x641
#define CAN_DIAGS_DTC_TX      0x541
#define CAN_PROG_ISO_TX       0x246
#define CAN_PROG_ISO_RX       0x646
#define CAN_TECH2_ID          0x101
#define CAN_GEAR_SPEED        0x4e8
          


#define NM_MATCH_ID           0x500
#define NM_MASK_ID            0x7f0

#ifdef DIAGS_ENABLED
#define ISOTXQUEUEDEPTH       3
#else
#define ISOTXQUEUEDEPTH       5
#endif

#define ISOMAXMESSAGESIZE     128
#define DISPLAY_REFRESH_TIME  5000  
#define ISO_TX_DELAY          20

static const u8 NMDataOff[] = {0x00,0x00,0x00,0x00};
static const u8 NMDataOn[] = {0x01,0x00,0x40,0x01};
static const u8 NMDataWake[] = {0x21,0x00,0x40,0x01};

#define FORMATTEDTEXTBLOCKSIZE  128
typedef enum
{
  ISOTX_IDLE,
  ISOTX_WAIT_COMPLETE,
  ISOTX_WAIT_TIMER
}ISO_TRANSMIT_STATE;


static struct
{
  u8 formatted_text_block[FORMATTEDTEXTBLOCKSIZE];
  u8 output_text[65];
  bool text_changed;
  bool text_refresh;
  bool need_to_clear;
  bool display_ready;
  bool display_on;
}vauxhall_display;

typedef struct
{
  u8 data[ISOMAXMESSAGESIZE];
  u16 length;
}ISOMessage;

static struct
{
  u32 gap_timer;
  u8 in;
  u8 out;
  u8 used;
  ISO_TRANSMIT_STATE iso_state;
  ISOMessage Message[ISOTXQUEUEDEPTH];
}ISOTxMessageQueue;

static const u8 StandardDisplayBlock[] = {0x40,0x00,0x03,0x03,0x10,0x00};
static const u8 FontSizeCommand[] = {0x00,0x1b,0x00,0x5b,0x00,0x66,0x00,0x53,0x00,0x5f,0x00,0x67,0x00,0x6d};
static const u8 JustifyCommand[] = {0x00,0x1b,0x00,0x5b,0x00,0x63,0x00,0x6d};
static const u8 ClearDisplayBlock[] = {0x41,0x00,0x06,0x03,0x10,0x11,0x12,0x90,0xb0};

static const u8 TextStringPioneer[] = "Pioneer";
static const u8 TextStringProgramOK[] = "PROGRAM OK";
static const u8 TextStringProgramFailed[] = "PROG FAIL";
// static const u8 TextStringUnknownDisplay[] = "DISPLAY ?";

static struct
{
  char * TextString;
  u16 OverlayTimer;
}DisplayText;

// diagnostic packets

static const u8 DiagCodeIndex[] =             {0x5a,0x73,0x30,0x30,0x30,0x20,0x30,0x30};
static const u8 DiagAudioIndex[] =            {0x5a,0x78,0x30,0x30,0x30,0x20,0x30,0x30};
static const u8 DiagTemperatureIndex[] =      {0x5a,0x79,0x30,0x30,0x30,0x20,0x30,0x30};
static const u8 DiagProductionDate[] =        {0x5a,0x7f,0x20,0x09,0x02,0x11};
static const u8 DiagSystemIdentification[] =  {0x5a,0x92,'C','O','N','N','E','C','T','S','2'};
static const u8 DiagSystemName[] =            {0x5a,0x97,'G','M',' ','S','T','A','L','K'};
static const u8 DiagIdentifier[] =            {0x5a,0x9a,0x02,0x0a};
static const u8 DiagNosticAddress[] =         {0x5a,0xb0,0x81};
static const u8 DiagSoftwareVersion[] =       {0x5a,0xc1,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x39};
static const u8 DiagPartNumber[] =            {0x5a,0xcb,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30};
static const u8 DiagHardwareNumber[] =        {0x5a,0xcc,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30};
static const u8 DiagAlphaCode[] =             {0x5a,0xdb,'A','A'};



static void ProcessPacket(TCANPacket * packet);
static void ConfigureCAN(void);
static void process_nm(void);
static void initialise_iso(void);
static void send_status(void);
static void display_text(void);
static u8 ISORoomLeftInBuffer( void );
static void create_text_block( u8 refresh );
static void  process_can_display_mode(TCANPacket * canpkt);
static void process_ISO_packets(void);
static u8 ISOAddMessageToBuffer( u8 * data, u16 length );
static void SendDIAGInfoString(u8 string_no);
static void ProcessDiags(void);
static void ProgrammingStateMachine(void);
static u8 FindCountryCode (u8 * code);
static u8 CheckDisplayCompatible (u16 DisplayID);
static void ForceCANWake(void);

typedef enum
{
  PGM_IGN_OFF,
  PGM_STARTUP,
  PGM_CHECK_DISPLAY_PRESENT,
  PGM_WAIT_DISPLAY_READY,
  PGM_GET_IDENTIFIER,
  PGM_WAIT_IDENTIFIER,

  // mid can config
  PGM_GET_MIDCANCONFIG1,
  PGM_WAIT_MIDCANCONFIG1,
  PGM_GET_MIDCANCONFIG2,
  PGM_WAIT_MIDCANCONFIG2,
  PGM_PROGRAMMIDCANSTART,
  PGM_PROGRAMMIDCANWAIT1,
  PGM_PROGRAMMIDCANWAIT2,
  
#ifdef PROGRAM_COUNTRY_CODE
// country code
  PGM_GET_VARIANTCOUNTRYCODE1,
  PGM_WAIT_VARIANTCOUNTRYCODE1,
  PGM_GET_VARIANTCOUNTRYCODE2,
  PGM_WAIT_VARIANTCOUNTRYCODE2,
  PGM_PROGRAMVARIANTCOUNTRYCODESTART,
  PGM_PROGRAMVARIANTCOUNTRYCODEWAIT1,
  PGM_PROGRAMVARIANTCOUNTRYCODEWAIT2,
#endif

  // radio present
  PGM_GET_VARIANTEHUPRESENT1,
  PGM_WAIT_VARIANTEHUPRESENT1,
  PGM_GET_VARIANTEHUPRESENT2,
  PGM_WAIT_VARIANTEHUPRESENT2,
  PGM_PROGRAMVARIANTEHUPRESENTSTART,
  PGM_PROGRAMVARIANTEHUPRESENTWAIT1,
  PGM_PROGRAMVARIANTEHUPRESENTWAIT2,
  
  PGM_COMPLETE_OK,
  PGM_FAILED,
  
  PGM_DELAY,
  PGM_FINISHED,
  PGM_END
}PROGRAM_STATE;

typedef struct
{
  u8  Code[4];
  u8  Name[22];
} COUNTRYCODES;

#ifdef PROGRAM_COUNTRY_CODE
#define NUMCOUNTRYCODES 14
static const COUNTRYCODES Country[NUMCOUNTRYCODES] = {
                                                      0x00,0x00,0x00,0x00,"Other/No Radio",
                                                      0x33,0x00,0x02,0x20,"Germany",
                                                      0x33,0x00,0x00,0x10,"Great Britain",
                                                      0x23,0x00,0x00,0x40,"Spain",
                                                      0x23,0x00,0x00,0x01,"Portugal/Netherlands",
                                                      0x23,0x00,0x00,0x80,"France",
                                                      0x00,0x00,0x20,0x00,"Italy",
                                                      0x33,0x00,0x00,0x40,"Sweden",
                                                      0x33,0x00,0x00,0x80,"Norway",
                                                      0x33,0x00,0x40,0x00,"Finland/Belgium",
                                                      0x33,0x00,0x00,0x02,"Denmark",
                                                      0x23,0x00,0x02,0x00,"Greece",
                                                      0x23,0x00,0x08,0x00,"Turkey/Poland",
                                                      0x33,0x00,0x10,0x00,"Switzerland"
                                                      };
#endif

#define NUMCOMPATIBLEDISPLAYS 1
static const u16 CompatibleDisplayIDs[NUMCOMPATIBLEDISPLAYS] = {  0x1002
                                                                };

extern volatile u32 WakeByIgnitionToken;

typedef enum
{
  CW_Start,
  CW_Active,
  CW_Wait5Sec,
  CW_CheckCanActive,
  CW_Wait2Sec,
  
  CW_NotRequired,
}CANWAKESTATE;


#endif
