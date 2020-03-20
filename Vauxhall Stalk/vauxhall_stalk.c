#include "common.h"
#include "can.h"
#include <string.h>
#include "vauxhall_stalk_internal.h"
#include "vauxhall_stalk.h"
#include "Diags.h"
#include "global.h"

//#define STALK_DIAG

//******************************************************************************

void VauxhallStalkInit(void)
{
  DEBUG("Vauxhall stalk init OK\n\r");
}
//******************************************************************************

void VauxhallStalkSide (void)
{
  process_input_keys();
  process_keypresses();
  process_menunavi_buttons();
}
//******************************************************************************

void process_stalk_packet( TCANPacket * canpkt )
{
  // we now need to check the commands comming in and send them to the pogo unit if needed

  if ( canpkt->data[0] == 0x01 ) // is button being pressed
  {
    switch ( canpkt->data[1] ) // what key is it?
    {
    case 0x9f: case 0x82: // hangup button
      vx_stalk.button = BUTTON_HANGUP;
      break;
    case 0x90: case 0x81: // voice button
      vx_stalk.button = BUTTON_VOICE;
      break;
    case 0x91: case 0x8e: // track up
      vx_stalk.button = BUTTON_TRACKUP;
      break;
    case 0x92: case 0x8f: // track down
      vx_stalk.button = BUTTON_TRACKDOWN;
      break;
    case 0x9d:            // volume up
      vx_stalk.button = BUTTON_VOLUP;
      break;
    case 0x9e:           // volume down
      vx_stalk.button = BUTTON_VOLDOWN;
      break;
    }
  }
  else if ( ( canpkt->data[0] == 0x08) && ( canpkt->data[1] == 0x93 ) ) // new astra volume packet
  {
    if ( canpkt->data[2] == 0x01 ) // volume up
    {
      vx_stalk.button = BUTTON_VOLUP;
      vx_stalk.key_release_timer = KEY_RELEASE_TIMEOUT;
    }
    else if ( canpkt->data[2] == 0xff ) // volume down
    {
      vx_stalk.button = BUTTON_VOLDOWN;
      vx_stalk.key_release_timer = KEY_RELEASE_TIMEOUT;
    }
  }
  else
  {
    vx_stalk.button = BUTTON_NONE;
    vx_stalk.key_release_timer = 0;
  }
}
//******************************************************************************

static void process_input_keys(void)
{
  if ( vx_stalk.key_release_timer )
  {
    vx_stalk.key_release_timer--;
    if ( !vx_stalk.key_release_timer )
    {
      vx_stalk.button = BUTTON_NONE;
    }
  }

  if ( vx_stalk.button != vx_stalk.old_button )
  {
    vx_stalk.last_button = vx_stalk.old_button;
    vx_stalk.last_button_time = vx_stalk.button_held_timer;
    vx_stalk.button_held_timer = 0;
    vx_stalk.old_button = vx_stalk.button;
  }
  else
  {
    if ( vx_stalk.button_held_timer < 0xffff )
    {
      vx_stalk.button_held_timer++;
    }
  }

#ifdef STALK_DIAG
  // diags
  if ( vx_stalk.button_held_timer == 0 ) // just pressed
  {
    switch ( vx_stalk.button )
    {
    case BUTTON_NONE:
      SendDiag("Button NONE\n\r");
      break;
    case BUTTON_HANGUP:
      SendDiag("Button Hangup\n\r");
      break;
    case BUTTON_VOICE:
      SendDiag("Button Voice\n\r");
      break;
    case BUTTON_TRACKUP:
      SendDiag("Button Track Up\n\r");
      break;
    case BUTTON_TRACKDOWN:
      SendDiag("Button Track Down\n\r");
      break;
    case BUTTON_VOLUP:
      SendDiag("Button Volume Up\n\r");
      break;
    case BUTTON_VOLDOWN:
      SendDiag("Button Volume Down\n\r");
      break;
    }
  }
#endif
}
//******************************************************************************


static void process_menunavi_buttons(void)
{
  if ( vx_stalk.menunavi_button != MENUNAVI_NONE ) // there is a button pressed
  {
    if ( vx_stalk.old_menunavi_button != vx_stalk.menunavi_button ) // this is the first time we have seen it
    {
      vx_stalk.navibutton_timer = 100;
      vx_stalk.navibutton_counter = 0;
      switch ( vx_stalk.menunavi_button )
      {
      case MENUNAVI_MAIN:
        send_key_can_packet(0x01,0xe0,0);
        break;
      case MENUNAVI_SETTINGS:
        send_key_can_packet(0x01,0xff,0);
        break;
      case MENUNAVI_WHEELRIGHT:
        send_key_can_packet(0x08,0x6a,0x01);
        break;
      case MENUNAVI_WHEELLEFT:
        send_key_can_packet(0x08,0x6a,0xff);
        break;
      case MENUNAVI_LEFT_ARROW:
        send_key_can_packet(0x01,0x6d,0);
        break;
      case MENUNAVI_RIGHT_ARROW:
        send_key_can_packet(0x01,0x6c,0);
        break;
      case MENUNAVI_BC:
        send_key_can_packet(0x01,0x01,0);
        break;
      case MENUNAVI_OK:
        send_key_can_packet(0x01,0x6f,0);
        break;
      }
    }
    else if ( vx_stalk.menunavi_button == vx_stalk.old_menunavi_button )
    {
      // this is the repeat
      if ( vx_stalk.navibutton_timer )
      {
        if (!(--vx_stalk.navibutton_timer))
        {
          vx_stalk.navibutton_timer = 100;
          vx_stalk.navibutton_counter++;
          switch ( vx_stalk.menunavi_button )
          {
          case MENUNAVI_MAIN:
            send_key_can_packet(0x01,0xe0,vx_stalk.navibutton_counter);
            break;
          case MENUNAVI_SETTINGS:
            send_key_can_packet(0x01,0xff,vx_stalk.navibutton_counter);
            break;
          case MENUNAVI_LEFT_ARROW:
            send_key_can_packet(0x01,0x6d,vx_stalk.navibutton_counter);
            break;
          case MENUNAVI_RIGHT_ARROW:
            send_key_can_packet(0x01,0x6c,vx_stalk.navibutton_counter);
            break;
          case MENUNAVI_BC:
            send_key_can_packet(0x01,0x01,vx_stalk.navibutton_counter);
            break;
          case MENUNAVI_OK:
            send_key_can_packet(0x01,0x6f,vx_stalk.navibutton_counter);
            break;
          }
        }
      }
    }
  }
  else if ( vx_stalk.old_menunavi_button != MENUNAVI_NONE )
  {
    vx_stalk.navibutton_timer = 0;
    switch ( vx_stalk.old_menunavi_button )
    {
    case MENUNAVI_MAIN:
      send_key_can_packet(0x00,0xe0,vx_stalk.navibutton_counter);
      break;
    case MENUNAVI_SETTINGS:
      send_key_can_packet(0x00,0xff,vx_stalk.navibutton_counter);
      break;
    case MENUNAVI_LEFT_ARROW:
      send_key_can_packet(0x00,0x6d,vx_stalk.navibutton_counter);
      break;
    case MENUNAVI_RIGHT_ARROW:
      send_key_can_packet(0x00,0x6c,vx_stalk.navibutton_counter);
      break;
    case MENUNAVI_BC:
      send_key_can_packet(0x00,0x01,vx_stalk.navibutton_counter);
      break;
    case MENUNAVI_OK:
      send_key_can_packet(0x00,0x6f,vx_stalk.navibutton_counter);
      break;
    }
    vx_stalk.navibutton_counter = 0;

  }
  vx_stalk.old_menunavi_button = vx_stalk.menunavi_button;
}
//******************************************************************************

static void send_key_can_packet (u8 byte1,u8 byte2,u8 byte3)
{
  TCANPacket pkt;

  pkt.cplen = sizeof(TCANPacket);
  pkt.dlc = 3;
  pkt.id = 0x201;
  pkt.tag = 0;
  pkt.data[0] = byte1;
  pkt.data[1] = byte2;
  pkt.data[2] = byte3;

  CANTx(&pkt);
}
//******************************************************************************

static void process_keypresses(void)
{
  static TDISPLAY_MODE old_display_mode;
  static u8 wait_for_release = 0;
  static BUTTON old_vauxhall_button;
  
  if ( global.display_mode != old_display_mode )
  {
    old_display_mode = global.display_mode;
    wait_for_release = 1;
  }
  if ( vx_stalk.button == BUTTON_NONE )
  {
    wait_for_release = 0;
  }

  if( global.display_mode == DISPLAY_MODE_RADIO )
  {
    vx_stalk.menunavi_button = MENUNAVI_NONE;
    switch ( vx_stalk.button )
    {
    case BUTTON_HANGUP:
      if ( vx_stalk.button_held_timer > 2000 )
      {
        vx_stalk.menunavi_button = MENUNAVI_SETTINGS;
      }
      break;
    case BUTTON_VOICE:
      if ( ! (wait_for_release || global.phone_kit_present) )
      {
        VauxhallStalk.Button = BUTTON_VOICE;
        vx_stalk.extend_button_timer = 0;
      }
      break;
    case BUTTON_TRACKUP:
      if ( !wait_for_release )
      {
        VauxhallStalk.Button = BUTTON_TRACKUP;
        vx_stalk.extend_button_timer = 0;
      }
      break;
    case BUTTON_TRACKDOWN:
      if ( !wait_for_release )
      {
        VauxhallStalk.Button = BUTTON_TRACKDOWN;
        vx_stalk.extend_button_timer = 0;
      }
      break;
    case BUTTON_VOLUP:
      if ( !wait_for_release )
      {
        VauxhallStalk.Button = BUTTON_VOLUP;
        vx_stalk.extend_button_timer = 0;
      }
      break;
    case BUTTON_VOLDOWN:
      if ( !wait_for_release )
      {
        VauxhallStalk.Button = BUTTON_VOLDOWN;
        vx_stalk.extend_button_timer = 0;
      }
      break;
    case BUTTON_NONE: // no button pressed now, but we may need to look for a release
      switch ( vx_stalk.last_button )
      {
      case BUTTON_HANGUP:
#ifdef STALK_DIAG
        SendDiag("Button HANGUP\n\r");
#endif
        if ( ( vx_stalk.last_button_time < 2000 )  && ( !global.phone_kit_present ) )
        {
#ifdef STALK_DIAG
          SendDiag("Button HANGUP < 2000\n\r");
#endif
          VauxhallStalk.Button = BUTTON_HANGUP;
          vx_stalk.extend_button_timer = 200;
        }
        break;
      default:
        if ( vx_stalk.extend_button_timer )
        {
          vx_stalk.extend_button_timer--;
        }
        else
        {
          VauxhallStalk.Button = BUTTON_NONE;
        }
        break;
      }
      break;
    default:
     VauxhallStalk.Button = BUTTON_NONE;
      break;
    }
  }
  else // must be BC or settings
  {
    VauxhallStalk.Button = BUTTON_NONE;
    switch ( vx_stalk.button )
    {
    case BUTTON_HANGUP:
      if ( vx_stalk.button_held_timer > 2000 )
      {
        vx_stalk.menunavi_button = MENUNAVI_SETTINGS;
      }
      break;
    case BUTTON_TRACKUP:
      if ( !wait_for_release )
      {
        vx_stalk.menunavi_button = MENUNAVI_RIGHT_ARROW ;
      }
      break;
    case BUTTON_TRACKDOWN:
      if ( !wait_for_release )
      {
        vx_stalk.menunavi_button = MENUNAVI_LEFT_ARROW;
      }
      break;
    case BUTTON_VOLUP:
      if ( !wait_for_release )
      {
        vx_stalk.menunavi_button = MENUNAVI_OK;
      }
      break;
    case BUTTON_VOLDOWN:
      if ( !wait_for_release )
      {
        vx_stalk.menunavi_button = MENUNAVI_MAIN ;
      }
      break;
    default:
      vx_stalk.menunavi_button = MENUNAVI_NONE;
    }
  }
  vx_stalk.last_button = BUTTON_NONE;
  
  if ( VauxhallStalk.TimeHeld != 0xffff )
    VauxhallStalk.TimeHeld++;
  if ( old_vauxhall_button != VauxhallStalk.Button )
  {
    VauxhallStalk.TimeHeld = 0;
    old_vauxhall_button = VauxhallStalk.Button;
  }
}

//******************************************************************************

