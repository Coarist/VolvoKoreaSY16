#include <ior8c22_23.h>
#include <intrinsics.h>
#include "CarsideInternal.h"
#include <stdio.h>

#define DISLAYISOBUFFLEN 128
#define ISODisplayID  1
static struct
{
  ISO15765_Channel ChannelData;
  u8 Buffer[DISLAYISOBUFFLEN];
}DisplayISO;
#define DIAGSISOBUFFLEN 64
#define ISODiagsID  2
static struct
{
  ISO15765_Channel ChannelData;
  u8 Buffer[DIAGSISOBUFFLEN];
}DiagsISO;
#define PROGRAMISOBUFFLEN 64
#define ISOProgramID  3
static struct
{
  ISO15765_Channel ChannelData;
  u8 Buffer[PROGRAMISOBUFFLEN];
  bool Enabled;
}ProgramISO;
static bool ProgramIgnOn = false;
static u16 CANDataReceived = 0;

static const TCANInitData caninitdata =
{
  sizeof(TCANInitData),
  CIT_HIGH_SPEED,    // Can type = High speed
  2,    // Divider
  2,    // No prescaler
  1,    // One time sampling
  6,    // Propagation time segment
  8,    // Phase buffer segment 1
  6,    // Phase buffer segment 2
  2,    // Synchronization jump width
  100,  // Timeout in ms for successful packet send
        // Channels to receive: (zero = not used)
  { CAN_STALK_ID,CAN_IGN_ID,0x4E8,CAN_DISPLAY_MODE_ID,CAN_DISPLAY_MODE2_ID, CAN_RADIO_ISO_RX, CAN_DIAGS_ISO_RX, CAN_PROG_ISO_RX, CAN_TECH2_ID, -1, -1, -1, -1, NM_MATCH_ID, -1},
    
  CIF_CLRBUFTXER, // Flags
  0xffff,               // global mask 11 bit
  0xffff,               // global mask 29 bit
  NM_MASK_ID,               // mask A 11 bit ID ( network management )
  0xffff,               // mask A 29 bit ID
  0xffff,               // mask B 11 bit ID
  0xffff                // mask B 29 bit ID        
};


/********************************************************************************************************************************/
void InitCarSide(void)
{
  ConfigureCAN();
  initialise_iso();
  VauxhallStalkInit();
  DisplayText.TextString = (char*)TextStringPioneer;
}
/********************************************************************************************************************************/

static void initialise_iso(void)
{
  DEBUG("ISO Display Channel Init\r\n");
  ISO15765_Connect (&DisplayISO.ChannelData,ISODisplayID,CAN_RADIO_ISO_TX, CAN_RADIO_ISO_RX,DisplayISO.Buffer,DISLAYISOBUFFLEN,ISODIR_TX);
  DEBUG("ISO Diagnostics Channel Init\r\n");
  ISO15765_Connect (&DiagsISO.ChannelData,ISODiagsID,CAN_DIAGS_ISO_TX, CAN_DIAGS_ISO_RX,DiagsISO.Buffer,DIAGSISOBUFFLEN,ISODIR_BI);
  ProgramISO.Enabled = false;
}
/********************************************************************************************************************************/

void CarSide(void)
{
  TCANPacket pkt;
  static u16 busofftimer = 0;

  global.sleeptimer++;

  if (busofftimer)
  {
    busofftimer --;
  }
  
  pkt.cplen = sizeof(TCANPacket);
  switch (CANRx(&pkt))
  {
    case CANERR_RX_OK:
      switch ( pkt.tag >> 8 )
      {
      case ISODisplayID:
        ISO15765_ReportSuccess(&DisplayISO.ChannelData,pkt.tag);
        break;
      case ISODiagsID:
        ISO15765_ReportSuccess(&DiagsISO.ChannelData,pkt.tag);
        break;
      case ISOProgramID:
        if ( ProgramISO.Enabled )
          ISO15765_ReportSuccess(&ProgramISO.ChannelData,pkt.tag);
        break;
      }
      ProcessPacket(&pkt);
      global.sleeptimer = 0;      // reset counter as we are receiving CAN
      CANDataReceived++; // number of packets received
      break;
    case CANERR_RX_BUSOFF:
      if (busofftimer == 0)
      {
        ConfigureCAN();
        busofftimer = 250;
      }
      break;
    case CANERR_RX_BUSERR:
    case CANERR_RX_INVALIDPKT:
    case CANERR_RX_NODATA:
      switch ( pkt.tag >> 8 )
      {
      case ISODisplayID:
        ISO15765_ReportSuccess(&DisplayISO.ChannelData,pkt.tag);
        break;
      case ISODiagsID:
        ISO15765_ReportSuccess(&DiagsISO.ChannelData,pkt.tag);
        break;
      case ISOProgramID:
        if ( ProgramISO.Enabled )
          ISO15765_ReportSuccess(&ProgramISO.ChannelData,pkt.tag);
        break;
      }
      break;
    case CANERR_RX_OVRUN:
    case CANERR_RX_TXTIMEOUT:
    default:
      break;
  }
  if ( (global.sleeptimer >= 10000) )   // 10 seconds of no CAN data
  {
    global.ignition = 0;
    if ( vaux_nm_status() == NME_SLEEPING )
    {
      global.sleeptimer = 0;
      global.sleep = 1;
    }
  }

  vaux_nm_1ms();
  process_nm();
  ISO15765_RunCycle(&DisplayISO.ChannelData);
  ISO15765_RunCycle(&DiagsISO.ChannelData);
  if ( ProgramISO.Enabled )
    ISO15765_RunCycle(&ProgramISO.ChannelData);
  send_status();
  display_text();
  process_ISO_packets();
  ProcessDiags();
  VauxhallStalkSide();
  ProgrammingStateMachine();
  ForceCANWake();
}
/********************************************************************************************************************************/

static void ProcessPacket( TCANPacket * packet )
{
  if ( ( packet->id & NM_MASK_ID ) == NM_MATCH_ID )
  {
    vaux_nm_can (packet);
  }

  switch ( packet->id )
  {
  case CAN_DISPLAY_MODE2_ID:
    vauxhall_display.display_ready = true;
    break;

  case CAN_IGN_ID:                       // ignition & illumination
    // look for keys being removed
    if ( packet->data[2] == 0x00 ) // no keys
    {
      global.ignition = 0;
      SetNMData((u8*)NMDataOff);        
    }
    // look for ign on or crank
    if ( ( packet->data[2] == 0x05 ) || ( packet->data[2] == 0x06 ) || ( packet->data[2] == 0x07 ) )
    {
      global.ignition = 1;
      SetNMData((u8*)NMDataOn);        
    }
    if ( packet->data[2] == 0x06 )
      ProgramIgnOn = true;
    else
      ProgramIgnOn = false;
    if (packet->data[3])
    {
      global.illumination = 1;
    }
    else
    {
      global.illumination = 0;
    }
    break;
  case CAN_RADIO_ISO_RX:
    ISO15765_ProcessPkt(&DisplayISO.ChannelData,packet);
    break;
  case CAN_DISPLAY_MODE_ID:                       // display mode
    process_can_display_mode(packet);
    break;
  case CAN_GEAR_SPEED:                       // gear and speed packet
    if (packet->data[6] & 0x04)
    {
      global.reverse = 1;  
    }
    else
    {
      global.reverse = 0;
    }
    global.speed = (packet->data[4] <<1);  // speed in Km/h
    if (packet->data[5] & 0x80)
    {
      global.speed |= 0x0001;
    }
    if (global.speed <= 0x03)           // No Parkbrake data, so we'll apply parkbrake below 4km/h
    {
      global.parkbrake = 1;
    }
    else
    {
      global.parkbrake = 0;
    }
    break;
  case CAN_STALK_ID:
    process_stalk_packet(packet);
    break;
  case CAN_DIAGS_ISO_RX:
    ISO15765_ProcessPkt(&DiagsISO.ChannelData,packet);
    break;
  case CAN_PROG_ISO_RX:
    if ( ProgramISO.Enabled )
      ISO15765_ProcessPkt(&ProgramISO.ChannelData,packet);
    break;
  default:
    break;
  }
}
/********************************************************************************************************************************/

static void ConfigureCAN(void)
{

  // Call initialization functions

  if (CANInit((TCANInitData*)&caninitdata) == CANERR_INIT_OK)
  {
    DEBUG("CAN Init Success\r\n");
    CANFlush(CANF_BOTH_BUFFERS, 0);
  }
  else
  {
    DEBUG("CAN Init FAIL!\r\n");
  }
}
/******************************************************************************************/

static void process_nm(void)
{
  static bool old_ign = false;
  
  if ( global.ignition && !old_ign )
  {
    old_ign = true;
    vaux_nm_cmd(NMC_WAKE);
  }
  else if ( !global.ignition && old_ign )
  {
    old_ign = false;
    vaux_nm_cmd(NMC_SLEEP);
  }

  if ( vaux_node_avail(6) && ( vaux_nm_status() == NME_ACTIVE ) )
  {
    vauxhall_display.display_on = true;
  }
  else
  {
    vauxhall_display.display_on = false;
    vauxhall_display.display_ready = false;
  }
  if ( vaux_node_avail(7) && ( vaux_nm_status() == NME_ACTIVE ) )
  {
    global.phone_kit_present = true;
  }
  else
  {
    global.phone_kit_present = false;
  }
}
/******************************************************************************************/

static void send_status(void)
{
  TCANPacket pkt;
  static u16 status_timer;
  static u8 old_radio_on;

  pkt.cplen = sizeof(TCANPacket);

  if ( vaux_nm_status() != NME_ACTIVE )
  {
    status_timer = 0;
    return;
  }

  if ( global.ignition != old_radio_on ) // need to send the change
  {
    status_timer = 0; // force the change out now
    old_radio_on = global.ignition;
  }

  if ( status_timer )
  {
    status_timer--;
    return;
  }

  // we must need to send out the status now
  pkt.id = 0x691;
  pkt.dlc = 8;
  pkt.tag = 0;
  pkt.data[0] = 0x41;
  pkt.data[1] = 0x00;
  pkt.data[2] = 0x60;
  pkt.data[3] = 0x02;
  pkt.data[4] = 0x00;
  pkt.data[5] = 0x00;
  pkt.data[6] = 0x00;
  pkt.data[7] = 0x2e;
  // is the radio on
  if ( global.ignition )
  {
    pkt.data[4] = 0x82;
  }
  // send the packet
  CANTx(&pkt);
  status_timer = 2500;
}
/******************************************************************************************/

static void display_text(void)
{
  static u16 refresh_timer;

  if ( DisplayText.OverlayTimer )
  {
    DisplayText.OverlayTimer--;
  }
  else
  {
    DisplayText.TextString = (char*)TextStringPioneer;
  }
  if ( (!vauxhall_display.display_ready) || (!vauxhall_display.display_on) )
  {
    return;
  }
  if ( global.ignition )
  {
    refresh_timer++;

    if ( strcmp( (char*)vauxhall_display.output_text,DisplayText.TextString ) )
    {
      strcpy((char*)vauxhall_display.output_text,DisplayText.TextString);
      refresh_timer = 0;
      vauxhall_display.text_changed = true;
      vauxhall_display.text_refresh = false;
    }
    if ( refresh_timer > DISPLAY_REFRESH_TIME )
    {
      refresh_timer = 0;
      vauxhall_display.text_refresh = true;
    }

    if ( vauxhall_display.text_refresh || vauxhall_display.text_changed )
    {
      //we need to update the text line
      if ( ISORoomLeftInBuffer() )
      {
        if ( vauxhall_display.text_refresh )
          create_text_block(1);
        else
          create_text_block(0);
        ISOAddMessageToBuffer(vauxhall_display.formatted_text_block,vauxhall_display.formatted_text_block[2]+3);
        vauxhall_display.text_refresh = false;
        vauxhall_display.text_changed = false;
        vauxhall_display.need_to_clear = true;
      }
    }
  }
  else // radio is not on
  {
    if ( vauxhall_display.need_to_clear )
    {
      if ( ISORoomLeftInBuffer() )
      {
        ISOAddMessageToBuffer((u8*)ClearDisplayBlock,ClearDisplayBlock[2]+3);
        vauxhall_display.text_refresh = false;
        vauxhall_display.text_changed = false;
        vauxhall_display.need_to_clear = false;
        vauxhall_display.output_text[0] = false;
      }
    }
  }
}
/******************************************************************************************/

static u8 ISORoomLeftInBuffer( void )
{
  if ( ISOTxMessageQueue.used < ISOTXQUEUEDEPTH )
    return 1; // there is room in the buffer
  else
    return 0;
}
/******************************************************************************************/

static void create_text_block( u8 refresh )
{
  u32 string_length = 0;
  u8 * destination, *source;
  // first lets clear the block out
  memset(vauxhall_display.formatted_text_block,0,FORMATTEDTEXTBLOCKSIZE);
  // now put the standard packet in there
  memcpy(vauxhall_display.formatted_text_block,StandardDisplayBlock,sizeof(StandardDisplayBlock));
  // now set the fist byte if a refresh
  if ( refresh )
    vauxhall_display.formatted_text_block[0] = 0xc0;
  // and now add the text string, formatting as we go
  // first lets set a pointer to the start of where we can add text
  destination = &vauxhall_display.formatted_text_block[sizeof(StandardDisplayBlock)];
  // now lets get a pointer to the string of text to copy/format
  source = vauxhall_display.output_text;

  if ( strlen((char const *)source) < 11 ) // centre justify short strings
  {
    memcpy ( destination, JustifyCommand,sizeof(JustifyCommand) );
    destination += sizeof(JustifyCommand);
    string_length += ( sizeof(JustifyCommand) >> 1);
    destination[-3] = 0x63;

  }

  while ( *source && ( string_length < 200 ) ) // copy until the null
  {
    *destination++ = 0;
    *destination++ = *source++;
    string_length++;
  }

  // set the number of unicode chars
  vauxhall_display.formatted_text_block[5] += string_length;

  string_length <<= 1;
  string_length += vauxhall_display.formatted_text_block[2];
  vauxhall_display.formatted_text_block[2] = string_length;
  vauxhall_display.formatted_text_block[1] = string_length >> 8;
}
/******************************************************************************************/

static void  process_can_display_mode(TCANPacket * canpkt)
{
  // here we need to decide what mode the display is in
  // vauxhall.display_mode

  if ( canpkt->data[2] & 0x08 ) // not radio mode
  {
    if ( canpkt->data[1] & 0x08 )
      global.display_mode = DISPLAY_MODE_SETTINGS;
    else
      global.display_mode = DISPLAY_MODE_BC;
  }
  else
    global.display_mode = DISPLAY_MODE_RADIO;


}
/******************************************************************************************/

static void process_ISO_packets(void)
{
  switch ( ISOTxMessageQueue.iso_state )
  {
  case ISOTX_IDLE:
    if ( ISOTxMessageQueue.used )
    {
      // is the ISO Layer ready to send another packet out
      if ( ISO15765_Status(&DisplayISO.ChannelData) == ( ( (u16)ISO15765_GSTATE_IDLE << 8 ) | (u16)ISO15765_TSTATE_CONNOK ) )
      {
        // ok then send it
        ISO15765_ChTx ( &DisplayISO.ChannelData,ISOTxMessageQueue.Message[ISOTxMessageQueue.out].data, ISOTxMessageQueue.Message[ISOTxMessageQueue.out].length);
        if ( ISOTxMessageQueue.out )
          ISOTxMessageQueue.out--;
        else
          ISOTxMessageQueue.out = (ISOTXQUEUEDEPTH-1);
        ISOTxMessageQueue.used--;
        ISOTxMessageQueue.iso_state = ISOTX_WAIT_COMPLETE;
      }
    }
    break;
  case ISOTX_WAIT_COMPLETE:
    // check to see if the transport layer has finished sending the last packet
    if ( ISO15765_Status(&DisplayISO.ChannelData) == ( ( (u16)ISO15765_GSTATE_IDLE << 8 ) | (u16)ISO15765_TSTATE_CONNOK ) )
    {
      ISOTxMessageQueue.iso_state = ISOTX_WAIT_TIMER;
      ISOTxMessageQueue.gap_timer = ISO_TX_DELAY;
    }
    break;
  case ISOTX_WAIT_TIMER:
    if ( ISOTxMessageQueue.gap_timer )
    {
      ISOTxMessageQueue.gap_timer--;
    }
    else
    {
      ISOTxMessageQueue.iso_state = ISOTX_IDLE;
    }
    break;
  default:
      ISOTxMessageQueue.iso_state = ISOTX_IDLE;
    break;
  }
}
/******************************************************************************************/

static u8 ISOAddMessageToBuffer( u8 * data, u16 length )
{
  if ( ISOTxMessageQueue.used >= ISOTXQUEUEDEPTH )
    return 1; // mesasge buffer full
  if ( length > ISOMAXMESSAGESIZE )
    return 2; // message too long
  memcpy( ISOTxMessageQueue.Message[ISOTxMessageQueue.in].data,data,length);
  ISOTxMessageQueue.Message[ISOTxMessageQueue.in].length = length;
  if ( ISOTxMessageQueue.in )
    ISOTxMessageQueue.in--;
  else
    ISOTxMessageQueue.in = (ISOTXQUEUEDEPTH-1);
  ISOTxMessageQueue.used++;
  return 0;
}
/******************************************************************************************/

static void SendDIAGInfoString(u8 string_no)
{
  TCANPacket sendpacket;
  memset ( &sendpacket,0,sizeof(TCANPacket ) );
  sendpacket.cplen = sizeof(TCANPacket);
  sendpacket.dlc = 8;
  sendpacket.tag = 0;
  sendpacket.id = CAN_DIAGS_ISO_TX;
  switch ( string_no )
  {
  case 0x73:  // Code Index
    if ( ISO15765_Status(&DiagsISO.ChannelData) == ( ( (u16)ISO15765_GSTATE_IDLE << 8 ) | (u16)ISO15765_TSTATE_CONNOK ) )
    {
      // ok then send it
      ISO15765_ChTx ( &DiagsISO.ChannelData,(uint8*)DiagCodeIndex, sizeof(DiagCodeIndex));
    }
    break;
  case 0x78:  // Audio Index
    if ( ISO15765_Status(&DiagsISO.ChannelData) == ( ( (u16)ISO15765_GSTATE_IDLE << 8 ) | (u16)ISO15765_TSTATE_CONNOK ) )
    {
      // ok then send it
      ISO15765_ChTx ( &DiagsISO.ChannelData,(uint8*)DiagAudioIndex, sizeof(DiagAudioIndex));
    }
    break;
  case 0x79:  // Temperature Index
    if ( ISO15765_Status(&DiagsISO.ChannelData) == ( ( (u16)ISO15765_GSTATE_IDLE << 8 ) | (u16)ISO15765_TSTATE_CONNOK ) )
    {
      // ok then send it
      ISO15765_ChTx ( &DiagsISO.ChannelData,(uint8*)DiagTemperatureIndex, sizeof(DiagTemperatureIndex));
    }
    break;
  case 0x7f: // Production Date
    if ( ISO15765_Status(&DiagsISO.ChannelData) == ( ( (u16)ISO15765_GSTATE_IDLE << 8 ) | (u16)ISO15765_TSTATE_CONNOK ) )
    {
      // ok then send it
      ISO15765_ChTx ( &DiagsISO.ChannelData,(uint8*)DiagProductionDate, sizeof(DiagProductionDate));
    }
    break;
  case 0x92:  // System Identification
    if ( ISO15765_Status(&DiagsISO.ChannelData) == ( ( (u16)ISO15765_GSTATE_IDLE << 8 ) | (u16)ISO15765_TSTATE_CONNOK ) )
    {
      // ok then send it
      ISO15765_ChTx ( &DiagsISO.ChannelData,(uint8*)DiagSystemIdentification, sizeof(DiagSystemIdentification));
    }
    break;
  case 0x97:  // System Name
    if ( ISO15765_Status(&DiagsISO.ChannelData) == ( ( (u16)ISO15765_GSTATE_IDLE << 8 ) | (u16)ISO15765_TSTATE_CONNOK ) )
    {
      // ok then send it
      ISO15765_ChTx ( &DiagsISO.ChannelData,(uint8*)DiagSystemName, sizeof(DiagSystemName));
    }
    break;
  case 0x9a: // Identifier
    if ( ISO15765_Status(&DiagsISO.ChannelData) == ( ( (u16)ISO15765_GSTATE_IDLE << 8 ) | (u16)ISO15765_TSTATE_CONNOK ) )
    {
      // ok then send it
      ISO15765_ChTx ( &DiagsISO.ChannelData,(uint8*)DiagIdentifier, sizeof(DiagIdentifier));
    }
    break;
  case 0xb0: // ECU Diagnostic Address
    if ( ISO15765_Status(&DiagsISO.ChannelData) == ( ( (u16)ISO15765_GSTATE_IDLE << 8 ) | (u16)ISO15765_TSTATE_CONNOK ) )
    {
      // ok then send it
      ISO15765_ChTx ( &DiagsISO.ChannelData,(uint8*)DiagNosticAddress, sizeof(DiagNosticAddress));
    }
    break;
  case 0xc1:  // Software Version
    if ( ISO15765_Status(&DiagsISO.ChannelData) == ( ( (u16)ISO15765_GSTATE_IDLE << 8 ) | (u16)ISO15765_TSTATE_CONNOK ) )
    {
      // ok then send it
      ISO15765_ChTx ( &DiagsISO.ChannelData,(uint8*)DiagSoftwareVersion, sizeof(DiagSoftwareVersion));
    }
    break;
  case 0xcb: // Part Number
    if ( ISO15765_Status(&DiagsISO.ChannelData) == ( ( (u16)ISO15765_GSTATE_IDLE << 8 ) | (u16)ISO15765_TSTATE_CONNOK ) )
    {
      // ok then send it
      ISO15765_ChTx ( &DiagsISO.ChannelData,(uint8*)DiagPartNumber, sizeof(DiagPartNumber));
    }
    break;
  case 0xcc: // Hardware Number
    if ( ISO15765_Status(&DiagsISO.ChannelData) == ( ( (u16)ISO15765_GSTATE_IDLE << 8 ) | (u16)ISO15765_TSTATE_CONNOK ) )
    {
      // ok then send it
      ISO15765_ChTx ( &DiagsISO.ChannelData,(uint8*)DiagHardwareNumber, sizeof(DiagHardwareNumber));
    }
    break;
  case 0xdb: // Alpha Code
    if ( ISO15765_Status(&DiagsISO.ChannelData) == ( ( (u16)ISO15765_GSTATE_IDLE << 8 ) | (u16)ISO15765_TSTATE_CONNOK ) )
    {
      // ok then send it
      ISO15765_ChTx ( &DiagsISO.ChannelData,(uint8*)DiagAlphaCode, sizeof(DiagAlphaCode));
    }
    break;

    
  default:
    sendpacket.data[0] = 0x03;
    sendpacket.data[1] = 0x7f;
    sendpacket.data[2] = 0x1a;
    DEBUG("CAN Diag unknown info string\r\n");    
    CANTx(&sendpacket);
    break;
  }
}
/******************************************************************************************/

static void ProcessDiags(void)
{
  TCANPacket sendpacket;
  u16 id;
  u8 * pkt = 0;
  u16 length;
  if ( !ISO15765_IsPacketWaiting(&DiagsISO.ChannelData) )
    return;

  memset ( &sendpacket,0,sizeof(TCANPacket ) );
  sendpacket.cplen = sizeof(TCANPacket);
  sendpacket.dlc = 8;
  
  //clear the rx channel
  
  ISO15765_Rx (&DiagsISO.ChannelData, &id , pkt, &length);
  // there must be a packet from the diag tool so deal with it
  switch ( DiagsISO.Buffer[0] ) // what cmd
  {
  case 0x1a: // information
    SendDIAGInfoString(DiagsISO.Buffer[1]);
    break;
  case 0x20: // not sure, seems like a static response
    DiagsISO.Buffer[0] = 0x60;
    if ( ISO15765_Status(&DiagsISO.ChannelData) == ( ( (u16)ISO15765_GSTATE_IDLE << 8 ) | (u16)ISO15765_TSTATE_CONNOK ) )
    {
      // ok then send it
      ISO15765_ChTx ( &DiagsISO.ChannelData,DiagsISO.Buffer, 1);
    }
    break;
  case 0xa9: // DTC    
    if ( (DiagsISO.Buffer[1] == 0x81) && (DiagsISO.Buffer[2] == 0x12) )
    {
      sendpacket.id = CAN_DIAGS_DTC_TX;
      sendpacket.tag = 0;
      sendpacket.data[0] = 0x81;
      sendpacket.data[4] = 0x1e;
      CANTx(&sendpacket);
    }
    break;
  }

}
/******************************************************************************************/

static void ProgrammingStateMachine(void)
{
  static PROGRAM_STATE PgmState = PGM_IGN_OFF;
  static PROGRAM_STATE DelayReturnState;
  static u16 DelayTimer;
  static u8 Config[10];
  u8 ConfigTemp;
  
  __no_init static u32 PowerOnDetect;
  u16 id;
  u8 * pkt = 0;
  u16 RxLength = 0;
  
  if ( (PgmState != PGM_END) && !ProgramIgnOn )
  {
    PgmState = PGM_IGN_OFF;
    ProgramISO.Enabled = false;    
  }

  if ( (ISO15765_IsPacketWaiting(&ProgramISO.ChannelData)) && (ProgramISO.Enabled) )
  {
    //clear the rx channel
    ISO15765_Rx (&ProgramISO.ChannelData, &id , pkt, &RxLength);
  }
  
  switch ( PgmState )
  {
//************************************************
  case PGM_IGN_OFF:
    if ( ProgramIgnOn )
    {
      PgmState = PGM_STARTUP;
      DEBUG("PSM Ign On\r\n");
      DelayTimer = 4000;
      PgmState = PGM_DELAY;
      DelayReturnState = PGM_STARTUP;
    }   
    break;
//************************************************
  case PGM_STARTUP:
    // OK first we need to see if this was a power on reset
    if ( PowerOnDetect == 0xcafed00d )
    {
      DEBUG("PSM Already Run\r\n");
      PgmState = PGM_FINISHED;
    }
    else
    {
      PowerOnDetect = 0xcafed00d;
      DelayTimer = 2000;
      PgmState = PGM_CHECK_DISPLAY_PRESENT;
      DEBUG("PSM Start\r\n");
      DEBUG("ISO Programming Channel Init\r\n");
      ISO15765_Connect (&ProgramISO.ChannelData,ISOProgramID,CAN_PROG_ISO_TX, CAN_PROG_ISO_RX,ProgramISO.Buffer,PROGRAMISOBUFFLEN,ISODIR_BI);
      ProgramISO.Enabled = true;
    }
    break;
//************************************************
  case PGM_CHECK_DISPLAY_PRESENT:
    if ( DelayTimer )
      DelayTimer--;
    else
    {
      // timeout
      PgmState = PGM_FINISHED;
      DEBUG("PSM Error. Display Not Found\r\n");
    }
    if ( vaux_node_avail(6) && ( vaux_nm_status() == NME_ACTIVE ) )
    {
      // the display is present and the NM is active
      ProgramISO.Buffer[0] = 0x20;
      if ( ISO15765_Status(&ProgramISO.ChannelData) == ( ( (u16)ISO15765_GSTATE_IDLE << 8 ) | (u16)ISO15765_TSTATE_CONNOK ) )
      {
        // Are You There
        ISO15765_ChTx ( &ProgramISO.ChannelData,ProgramISO.Buffer, 1);
        PgmState = PGM_WAIT_DISPLAY_READY;
        DelayTimer = 2000;
      }
    }
    break;
//************************************************
   case PGM_WAIT_DISPLAY_READY:
    if ( DelayTimer )
      DelayTimer--;
    else
    {
      PgmState = PGM_FAILED;
      DEBUG("PSM Error. No Response to AYT?\r\n");
    }
    if ( RxLength ) // we have received a packet
    {
      if ( ( RxLength != 1 ) || ( ProgramISO.Buffer[0] != 0x60 ) )
      {
        PgmState = PGM_FAILED;
        DEBUG("PSM Error. Invalid Response to AYT?\r\n");    
      }
      else
      {
        // everything is OK so we need to get the display's information so we know if its the correct one before programming it
        PgmState = PGM_GET_IDENTIFIER;
      }
    }
    break;
//************************************************
  case PGM_GET_IDENTIFIER:
    // the display is present and the NM is active
    ProgramISO.Buffer[0] = 0x1a;
    ProgramISO.Buffer[1] = 0x9a;
    if ( ISO15765_Status(&ProgramISO.ChannelData) == ( ( (u16)ISO15765_GSTATE_IDLE << 8 ) | (u16)ISO15765_TSTATE_CONNOK ) )
    {
      // GET IDENTIFIER
      ISO15765_ChTx ( &ProgramISO.ChannelData,ProgramISO.Buffer, 2);
      PgmState = PGM_WAIT_IDENTIFIER;
      DelayTimer = 2000;
    }
    break;
//************************************************
  case PGM_WAIT_IDENTIFIER:
    if ( DelayTimer )
      DelayTimer--;
    else
    {
      PgmState = PGM_FAILED;
      DEBUG("PSM Error. No Response to Identifier?\r\n");
    }
    if ( RxLength ) // we have received a packet
    {
      if ( (ProgramISO.Buffer[0]!=0x5a) || (ProgramISO.Buffer[1]!=0x9a) )
      {
        PgmState = PGM_FAILED;
        DEBUG("PSM Error. Invalid Response to Identifier\r\n");    
      }
      else
      {
        // is this the correct identifier
        if (  CheckDisplayCompatible( (((u16)ProgramISO.Buffer[2])<<8) + ProgramISO.Buffer[3]) == 0xff )
        {
          PgmState = PGM_FINISHED;
//          DisplayText.TextString = (char*)TextStringUnknownDisplay;
//          DisplayText.OverlayTimer = 5000;
          DEBUG("PSM Error. Incorrect Display Found\r\n");    
        }
        else
        {
          DEBUG("PSM Found Display with known identifier\r\n");    
          PgmState = PGM_GET_MIDCANCONFIG1;
        }        
      }
    }
    break;
//************************************************
  case PGM_GET_MIDCANCONFIG1:
    ProgramISO.Buffer[0] = 0x1a;
    ProgramISO.Buffer[1] = 0xbb;
    if ( ISO15765_Status(&ProgramISO.ChannelData) == ( ( (u16)ISO15765_GSTATE_IDLE << 8 ) | (u16)ISO15765_TSTATE_CONNOK ) )
    {
      // GET IDENTIFIER
      ISO15765_ChTx ( &ProgramISO.ChannelData,ProgramISO.Buffer, 2);
      PgmState = PGM_WAIT_MIDCANCONFIG1;
      DelayTimer = 2000;
    }
    break;
//************************************************
  case PGM_WAIT_MIDCANCONFIG1:
    if ( DelayTimer )
      DelayTimer--;
    else
    {
      PgmState = PGM_FAILED;
      DEBUG("PSM Error. No Response to Mid CAN Config\r\n");
    }
    if ( RxLength ) // we have received a packet
    {
      if ( (ProgramISO.Buffer[0]!=0x5a) || (ProgramISO.Buffer[1]!=0xbb) )
      {
        PgmState = PGM_FAILED;
        DEBUG("PSM Error. Invalid Response to Mid CAN Config\r\n");    
      }
      else
      {
        // check the mid can config
        Config[0] = ProgramISO.Buffer[2];
        Config[1] = ProgramISO.Buffer[3];
        PgmState = PGM_GET_MIDCANCONFIG2;
      }
    }
    break;
//************************************************
  case PGM_GET_MIDCANCONFIG2:
    ProgramISO.Buffer[0] = 0x1a;
    ProgramISO.Buffer[1] = 0xbb;
    if ( ISO15765_Status(&ProgramISO.ChannelData) == ( ( (u16)ISO15765_GSTATE_IDLE << 8 ) | (u16)ISO15765_TSTATE_CONNOK ) )
    {
      // GET IDENTIFIER
      ISO15765_ChTx ( &ProgramISO.ChannelData,ProgramISO.Buffer, 2);
      PgmState = PGM_WAIT_MIDCANCONFIG2;
      DelayTimer = 2000;
    }
    break;
//************************************************
  case PGM_WAIT_MIDCANCONFIG2:
    if ( DelayTimer )
      DelayTimer--;
    else
    {
      PgmState = PGM_FAILED;
      DEBUG("PSM Error. No Response to Mid CAN Config2\r\n");
    }
    if ( RxLength ) // we have received a packet
    {
      if ( (ProgramISO.Buffer[0]!=0x5a) || (ProgramISO.Buffer[1]!=0xbb) )
      {
        PgmState = PGM_FAILED;
        DEBUG("PSM Error. Invalid Response to Mid CAN Config\r\n");    
      }
      else
      {
        // check the mid can config
        if ( ( Config[0] == ProgramISO.Buffer[2] ) && ( Config[1] == ProgramISO.Buffer[3] ) )
        {
          if ( ProgramISO.Buffer[2] & 0x02 )
          {
#ifdef PROGRAM_COUNTRY_CODE
            PgmState = PGM_GET_VARIANTCOUNTRYCODE1;
#else
            PgmState = PGM_GET_VARIANTEHUPRESENT1;
#endif
            DEBUG("PSM Mid CAN EHU Present\r\n");    
          }
          else
          {
            DEBUG("PSM Mid CAN EHU Not Present\r\n");    
            PgmState = PGM_PROGRAMMIDCANSTART;
          }
        }
        else
        {
          PgmState = PGM_FAILED;
          DEBUG("PSM Error. Mid CAN Config read error\r\n");    
        }
      }
    }
    break;
//************************************************
  case PGM_PROGRAMMIDCANSTART:
    ProgramISO.Buffer[0] = 0x3b;
    ProgramISO.Buffer[1] = 0xbb;
    ProgramISO.Buffer[2] = Config[0] | 0x02;
    ProgramISO.Buffer[3] = Config[1];
    
    if ( ISO15765_Status(&ProgramISO.ChannelData) == ( ( (u16)ISO15765_GSTATE_IDLE << 8 ) | (u16)ISO15765_TSTATE_CONNOK ) )
    {
      // GET IDENTIFIER
      ISO15765_ChTx ( &ProgramISO.ChannelData,ProgramISO.Buffer, 4);
      PgmState = PGM_PROGRAMMIDCANWAIT1;
      DelayTimer = 2000;
    }
    break;
//************************************************
  case PGM_PROGRAMMIDCANWAIT1:
    if ( DelayTimer )
      DelayTimer--;
    else
    {
      PgmState = PGM_FAILED;
      DEBUG("PSM Error. No Response to Mid CAN Program\r\n");
    }
    if ( RxLength ) // we have received a packet
    {
      if ( (ProgramISO.Buffer[0]!=0x7f) || (ProgramISO.Buffer[1]!=0x3b) || (ProgramISO.Buffer[2]!=0x78) )
      {
        DEBUG("PSM Error. Invalid Response to Mid CAN Program\r\n");
        PgmState = PGM_FAILED;
      }
      else
      {
        PgmState = PGM_PROGRAMMIDCANWAIT2;
        DelayTimer = 2000;
      }
    }
    break;
//************************************************
  case PGM_PROGRAMMIDCANWAIT2:
    if ( DelayTimer )
      DelayTimer--;
    else
    {
      PgmState = PGM_FAILED;
      DEBUG("PSM Error. No Response 2 to Mid CAN Program\r\n");
    }
    if ( RxLength ) // we have received a packet
    {
      if ( (ProgramISO.Buffer[0]!=0x7b) || (ProgramISO.Buffer[1]!=0xbb) )
      {
        PgmState = PGM_FAILED;
        DEBUG("PSM Error. Mid CAN Program Failed\r\n");    
      }
      else
      {
#ifdef PROGRAM_COUNTRY_CODE
        PgmState = PGM_GET_VARIANTCOUNTRYCODE1;
#else
        PgmState = PGM_GET_VARIANTEHUPRESENT1;
#endif
        DEBUG("PSM Mid CAN Program Successful\r\n");    
      }
    }
    break;
//************************************************
#ifdef PROGRAM_COUNTRY_CODE
  case PGM_GET_VARIANTCOUNTRYCODE1:
    ProgramISO.Buffer[0] = 0x1a;
    ProgramISO.Buffer[1] = 0x44;
    if ( ISO15765_Status(&ProgramISO.ChannelData) == ( ( (u16)ISO15765_GSTATE_IDLE << 8 ) | (u16)ISO15765_TSTATE_CONNOK ) )
    {
      // GET IDENTIFIER
      ISO15765_ChTx ( &ProgramISO.ChannelData,ProgramISO.Buffer, 2);
      PgmState = PGM_WAIT_VARIANTCOUNTRYCODE1;
      DelayTimer = 2000;
    }
    break;
//************************************************
  case PGM_WAIT_VARIANTCOUNTRYCODE1:
    if ( DelayTimer )
      DelayTimer--;
    else
    {
      PgmState = PGM_FAILED;
      DEBUG("PSM Error. No Response to Variant Country Code\r\n");
    }
    if ( RxLength ) // we have received a packet
    {
      if ( (ProgramISO.Buffer[0]!=0x5a) || (ProgramISO.Buffer[1]!=0x44) )
      {
        PgmState = PGM_FAILED;
        DEBUG("PSM Error. Invalid Response to Variant Country Code\r\n");    
      }
      else if ( RxLength == 9 )
      {
        // check the mid can config
        memcpy(Config,&ProgramISO.Buffer[2],7);
        PgmState = PGM_GET_VARIANTCOUNTRYCODE2;
      }
      else
      {
        PgmState = PGM_FAILED;
        DEBUG("PSM Error. Invalid Data Length for Variant Country Code\r\n");    
      }
    }
    break;
//************************************************
  case PGM_GET_VARIANTCOUNTRYCODE2:
    ProgramISO.Buffer[0] = 0x1a;
    ProgramISO.Buffer[1] = 0x44;
    if ( ISO15765_Status(&ProgramISO.ChannelData) == ( ( (u16)ISO15765_GSTATE_IDLE << 8 ) | (u16)ISO15765_TSTATE_CONNOK ) )
    {
      // GET IDENTIFIER
      ISO15765_ChTx ( &ProgramISO.ChannelData,ProgramISO.Buffer, 2);
      PgmState = PGM_WAIT_VARIANTCOUNTRYCODE2;
      DelayTimer = 2000;
    }
    break;
//************************************************
  case PGM_WAIT_VARIANTCOUNTRYCODE2:
    if ( DelayTimer )
      DelayTimer--;
    else
    {
      PgmState = PGM_FAILED;
      DEBUG("PSM Error. No Response to Variant Country Code 2\r\n");
    }
    if ( RxLength ) // we have received a packet
    {
      if ( (ProgramISO.Buffer[0]!=0x5a) || (ProgramISO.Buffer[1]!=0x44) )
      {
        PgmState = PGM_FAILED;
        DEBUG("PSM Error. Invalid Response to Variant Country Code 2\r\n");    
      }
      else
      {
        // check the variant A config
        if ( !memcmp(Config,&ProgramISO.Buffer[2],7) ) // the data is the same
        {
          // no we need to check the country code
          ConfigTemp=FindCountryCode(&ProgramISO.Buffer[5]);
          if ( ConfigTemp == 0xff )
          {
            DEBUG("PSM Variant Country Code Invalid\r\n");    
            PgmState = PGM_PROGRAMVARIANTCOUNTRYCODESTART;
          }
          else if ( ConfigTemp == 0x00 ) // no country code set
          {
            DEBUG("PSM Variant Country Code = Other / No Radio\r\n");    
            PgmState = PGM_PROGRAMVARIANTCOUNTRYCODESTART;
          }
          else
          {
            PgmState = PGM_GET_VARIANTEHUPRESENT1;
            DEBUG("PSM Variant Country Code Valid. Country = ");
            DEBUG((char*)Country[ConfigTemp].Name);
            DEBUG("\r\n");
          }
        }
        else
        {
          PgmState = PGM_FAILED;
          DEBUG("PSM Error. Variant Country Code read error\r\n");    
        }
      }
    }
    break;
//************************************************
  case PGM_PROGRAMVARIANTCOUNTRYCODESTART:
    ProgramISO.Buffer[0] = 0x3b;
    ProgramISO.Buffer[1] = 0x44;

    
    ProgramISO.Buffer[2] = Config[0];
    ProgramISO.Buffer[3] = Config[1];
    ProgramISO.Buffer[4] = Config[2];
    ProgramISO.Buffer[5] = 0x33;
    ProgramISO.Buffer[6] = 0x00;
    ProgramISO.Buffer[7] = 0x00;
    ProgramISO.Buffer[8] = 0x10; // we will set this to UK
    
    if ( ISO15765_Status(&ProgramISO.ChannelData) == ( ( (u16)ISO15765_GSTATE_IDLE << 8 ) | (u16)ISO15765_TSTATE_CONNOK ) )
    {
      // GET IDENTIFIER
      ISO15765_ChTx ( &ProgramISO.ChannelData,ProgramISO.Buffer, 9);
      PgmState = PGM_PROGRAMVARIANTCOUNTRYCODEWAIT1;
      DelayTimer = 2000;
    }
    break;
//************************************************
  case PGM_PROGRAMVARIANTCOUNTRYCODEWAIT1:
    if ( DelayTimer )
      DelayTimer--;
    else
    {
      PgmState = PGM_FAILED;
      DEBUG("PSM Error. No Response to Variant Country Code Program\r\n");
    }
    if ( RxLength ) // we have received a packet
    {
      if ( (ProgramISO.Buffer[0]!=0x7f) || (ProgramISO.Buffer[1]!=0x3b) || (ProgramISO.Buffer[2]!=0x78) )
      {
        DEBUG("PSM Error. Invalid Response to Variant Country Code Program\r\n");
        PgmState = PGM_FAILED;
      }
      else
      {
        PgmState = PGM_PROGRAMVARIANTCOUNTRYCODEWAIT2;
        DelayTimer = 2000;
      }
    }
    break;
//************************************************
  case PGM_PROGRAMVARIANTCOUNTRYCODEWAIT2:
    if ( DelayTimer )
      DelayTimer--;
    else
    {
      PgmState = PGM_FAILED;
      DEBUG("PSM Error. No Response 2 to Variant Country Code Program\r\n");
    }
    if ( RxLength ) // we have received a packet
    {
      if ( (ProgramISO.Buffer[0]!=0x7b) || (ProgramISO.Buffer[1]!=0x44) )
      {
        PgmState = PGM_FAILED;
        DEBUG("PSM Error. Variant Country Code Program Failed\r\n");    
      }
      else
      {
        PgmState = PGM_GET_VARIANTEHUPRESENT1;
        DEBUG("PSM Varaint Country Code Program Successful\r\n");    
      }
    }
    break;
#endif

    
//************************************************
  case PGM_GET_VARIANTEHUPRESENT1:
    ProgramISO.Buffer[0] = 0x1a;
    ProgramISO.Buffer[1] = 0x4c;
    if ( ISO15765_Status(&ProgramISO.ChannelData) == ( ( (u16)ISO15765_GSTATE_IDLE << 8 ) | (u16)ISO15765_TSTATE_CONNOK ) )
    {
      // GET IDENTIFIER
      ISO15765_ChTx ( &ProgramISO.ChannelData,ProgramISO.Buffer, 2);
      PgmState = PGM_WAIT_VARIANTEHUPRESENT1;
      DelayTimer = 2000;
    }
    break;
//************************************************
  case PGM_WAIT_VARIANTEHUPRESENT1:
    if ( DelayTimer )
      DelayTimer--;
    else
    {
      PgmState = PGM_FAILED;
      DEBUG("PSM Error. No Response to Variant EHU Present\r\n");
    }
    if ( RxLength ) // we have received a packet
    {
      if ( (ProgramISO.Buffer[0]!=0x5a) || (ProgramISO.Buffer[1]!=0x4c) )
      {
        PgmState = PGM_FAILED;
        DEBUG("PSM Error. Invalid Response to Variant EHU Present\r\n");    
      }
      else if ( RxLength == 6 )
      {
        // check the variant B Config
        memcpy(Config,&ProgramISO.Buffer[2],4);
        PgmState = PGM_GET_VARIANTEHUPRESENT2;
      }
      else
      {
        PgmState = PGM_FAILED;
        DEBUG("PSM Error. Invalid Data Length for Variant EHU Present\r\n");    
      }
    }
    break;
//************************************************
  case PGM_GET_VARIANTEHUPRESENT2:
    ProgramISO.Buffer[0] = 0x1a;
    ProgramISO.Buffer[1] = 0x4c;
    if ( ISO15765_Status(&ProgramISO.ChannelData) == ( ( (u16)ISO15765_GSTATE_IDLE << 8 ) | (u16)ISO15765_TSTATE_CONNOK ) )
    {
      // GET IDENTIFIER
      ISO15765_ChTx ( &ProgramISO.ChannelData,ProgramISO.Buffer, 2);
      PgmState = PGM_WAIT_VARIANTEHUPRESENT2;
      DelayTimer = 2000;
    }
    break;
//************************************************
  case PGM_WAIT_VARIANTEHUPRESENT2:
    if ( DelayTimer )
      DelayTimer--;
    else
    {
      PgmState = PGM_FAILED;
      DEBUG("PSM Error. No Response to Variant EHU Present 2\r\n");
    }
    if ( RxLength ) // we have received a packet
    {
      if ( (ProgramISO.Buffer[0]!=0x5a) || (ProgramISO.Buffer[1]!=0x4c) )
      {
        PgmState = PGM_FAILED;
        DEBUG("PSM Error. Invalid Response to Variant EHU Present 2\r\n");    
      }
      else
      {
        // check the mid can config
        if ( !memcmp(Config,&ProgramISO.Buffer[2],4) ) // the data is the same
        {
          ConfigTemp = ProgramISO.Buffer[4] & 0xc0;
          if ( (ConfigTemp != 0x80)  && (ConfigTemp != 0xc0 ) )
          {
            DEBUG("PSM Variant EHU Present Invalid\r\n");    
            PgmState = PGM_PROGRAMVARIANTEHUPRESENTSTART;
          }
          else if ( ConfigTemp & 0x40) 
          {
            PgmState = PGM_COMPLETE_OK;
            DEBUG("PSM Variant EHU Present Correct\r\n");    
          }
          else
          {
            DEBUG("PSM Variant EHU Present Incorrect\r\n");    
            PgmState = PGM_PROGRAMVARIANTEHUPRESENTSTART;
          }
        }
        else
        {
          PgmState = PGM_FAILED;
          DEBUG("PSM Error. Variant EHU Present read error\r\n");    
        }
      }
    }
    break;
//************************************************
  case PGM_PROGRAMVARIANTEHUPRESENTSTART:
    ProgramISO.Buffer[0] = 0x3b;
    ProgramISO.Buffer[1] = 0x4c;
    memcpy(&ProgramISO.Buffer[2],Config,4);
    ProgramISO.Buffer[4] |= 0xc0;
    
    if ( ISO15765_Status(&ProgramISO.ChannelData) == ( ( (u16)ISO15765_GSTATE_IDLE << 8 ) | (u16)ISO15765_TSTATE_CONNOK ) )
    {
      // GET IDENTIFIER
      ISO15765_ChTx ( &ProgramISO.ChannelData,ProgramISO.Buffer, 6);
      PgmState = PGM_PROGRAMVARIANTEHUPRESENTWAIT1;
      DelayTimer = 2000;
    }
    break;
//************************************************
  case PGM_PROGRAMVARIANTEHUPRESENTWAIT1:
    if ( DelayTimer )
      DelayTimer--;
    else
    {
      PgmState = PGM_FAILED;
      DEBUG("PSM Error. No Response to Variant EHU Present Program\r\n");
    }
    if ( RxLength ) // we have received a packet
    {
      if ( (ProgramISO.Buffer[0]!=0x7f) || (ProgramISO.Buffer[1]!=0x3b) || (ProgramISO.Buffer[2]!=0x78) )
      {
        DEBUG("PSM Error. Invalid Response to Variant EHU Present Program\r\n");
        PgmState = PGM_FAILED;
      }
      else
      {
        PgmState = PGM_PROGRAMVARIANTEHUPRESENTWAIT2;
        DelayTimer = 2000;
      }
    }
    break;
//************************************************
  case PGM_PROGRAMVARIANTEHUPRESENTWAIT2:
    if ( DelayTimer )
      DelayTimer--;
    else
    {
      PgmState = PGM_FAILED;
      DEBUG("PSM Error. No Response 2 to Variant EHU Present Program\r\n");
    }
    if ( RxLength ) // we have received a packet
    {
      if ( (ProgramISO.Buffer[0]!=0x7b) || (ProgramISO.Buffer[1]!=0x4c) )
      {
        PgmState = PGM_FAILED;
        DEBUG("PSM Error. Variant EHU Present Program Failed\r\n");    
      }
      else
      {
        PgmState = PGM_COMPLETE_OK;
        DEBUG("PSM Varaint EHU Present Program Successful\r\n");    
      }
    }
    break;
//************************************************
  case PGM_COMPLETE_OK:
    DisplayText.TextString = (char*)TextStringProgramOK;
    DisplayText.OverlayTimer = 5000;
    PgmState = PGM_FINISHED;
    break;
//************************************************
  case PGM_FAILED:
    DisplayText.TextString = (char*)TextStringProgramFailed;
    DisplayText.OverlayTimer = 5000;
    PgmState = PGM_FINISHED;
    break;
//************************************************
  case PGM_DELAY:
    if ( DelayTimer )
      DelayTimer--;
    else
      PgmState = DelayReturnState;
    break;
//************************************************
  case PGM_FINISHED:
    DEBUG("PSM Finished\r\n");
    ProgramISO.Enabled = false;
    PgmState = PGM_END;
    break;
//************************************************
  case PGM_END:
    break;
//************************************************
  default:
    // oops, do nowt.
    break;
//************************************************
  }
}
/******************************************************************************************/

#ifdef PROGRAM_COUNTRY_CODE
static u8 FindCountryCode (u8 * code)
{
  u8 loop;
  for ( loop = 0 ; loop < NUMCOUNTRYCODES ; loop++ )
  {
    if ( !memcmp(code,Country[loop].Code,4 ) )
    { // we have a winner
      return loop;
    }
  }
  return 0xff;
}
#endif
/******************************************************************************************/

static u8 CheckDisplayCompatible (u16 DisplayID)
{
  u8 loop;
  for ( loop = 0 ; loop < NUMCOMPATIBLEDISPLAYS ; loop++ )
  {
    if ( DisplayID == CompatibleDisplayIDs[loop] )
    { // we have a winner
      return loop;
    }
  }
  return 0xff;
}
/******************************************************************************************/

static void ForceCANWake(void)
{
  static CANWAKESTATE CWState = CW_Start;
  static u16 DelayCounter = 0;
  DelayCounter++;
  

  switch ( CWState )
  {
//********************************
  case CW_Start:
    if ( WakeByIgnitionToken == IGNITIONWAKETOKEN )
    {
      DEBUG("Woken by IGN wire\r\n");
      CWState = CW_Active;
      WakeByIgnitionToken = 0;
    }
    else
    {
      DEBUG("NOT Woken by IGN wire\r\n");
      CWState = CW_NotRequired;
    }
    break;
//********************************
  case CW_Active:
    // we have been woken up by the ignition wire rather than the CAN wake line
    // first we need to skip 5 seconds so that we dont try to wake the bus when we dont need to
    DelayCounter = 0;
    CWState = CW_Wait5Sec;
    break;
//********************************
  case CW_Wait5Sec:
    if ( DelayCounter > 5000 )
    {
      CWState = CW_CheckCanActive;
    }
    break;
//********************************
  case CW_CheckCanActive:
    if ( CANDataReceived > 10 ) // we have received at least 10 packets so the network must be up and running
    {
      CWState = CW_NotRequired;
    }
    else // got nowt so lets try to wake the bus up
    {
      SetNMData((u8*)NMDataWake);        
      vaux_nm_cmd(NMC_FORCEWAKE);
      DelayCounter = 0;
      CWState = CW_Wait2Sec;    
    }
    break;
//********************************
  case CW_Wait2Sec:
    if ( DelayCounter > 2000 )
    {
      if ( !global.ignition )
      {
        vaux_nm_cmd(NMC_SLEEP);
        SetNMData((u8*)NMDataOff);        
      }
      CWState = CW_NotRequired;
    }
    break;
//********************************
  case CW_NotRequired:
    // do nowt
    break;
//********************************
  default:
    CWState = CW_Start;
    break;
  }
}
/********************************************************************************************************************************/

