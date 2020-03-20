#include <ior8c22_23.h>
#include <intrinsics.h>
#include "common.h"
#include "can.h"
#include "can_internal.h"
#include <string.h>
#include "global.h"

//#pragma diag_suppress=pe177,pe826

/*
	Revision history

        09 Feb 09 - changed code that tx queue and rx queue can be set to different sizes
                  - changed can init data to that it only stores a pointer rather than a copy of the data itself
                    this means that the calling function MUST keep the structure intact for all the time that the can routines are running

        23 Jun 08 - bug fix, changed can_int function so only reports bus error and bus off once each time the error occurs. this stops the RX channel from being
                    blocked by bus errors

        30 jan 07 - r8c version

	13 feb 06 - Added a method of retrieving back the packets in the queue that have succesfully completed transmission (there tags
				are returned to the caller upon called the CANRx routine).
				Reinitialising the can routines whilst there are packets attempting to be sent could cause a malfunction in both
				the sending and receiving or packets until the next time the can interface is reset, due to pending interrupts.

	30 aug 05 -	TCANInitData & TCANPacket sizes changed to accomodate new members. TCANInitData gets a new 'flags' member, and
					TCANPacket gets a new 'tag' member.
					If a TX fails due to timeout, the 'tag' from the packet that fails is provided in the received packet when
					CANERR_RX_TXTIMEOUT is received from the CANRx() routine.
					
	16 sep 05 - Added sleep functionality
*/

static TCANRXCircularBuffer InBuffer;					///< Incoming packet buffer - may be accessed by interrupt
static TCANTXCircularBuffer OutBuffer;				///< Outgoing packet buffer - may be accessed by interrupt
static TCANTagCircBuffer OutBuffer_Tags;
static uint8 Init_OK = 0;								///< Whether or not can initialisation has been a success
																/*!< If this variable remains unset, only CANInit() will function */
static uint8 TXInt_Finished = 1;						///< Whether or not the pending tx interrupt has executed & finished
																/*!< Whilst zero, outgoing packets will only be queued in OutBuffer */
static uint8 CANErrors = 0;							///< Bitmask of can errors that have occured. Parsed by CANRx() into CANERR_RX_*

static TCANInitData *LocalInitData;					///< Local copy of 'initdata' from parameter passed to CANInit()

static uint16 LastTXPacketTimer;						///< Keep track of how long ago the last packet was transmitted, to ensure txint happens.
static uint16 LastTXPacketTag;						///< Tag of the last packet that was sent (successfully or not)
static uint16 LastTXPacketTag_Fail;
static u8 bit_position_lookup_table[16];

// A global variable!
TExtCANInfo can;

// // // // // // // // // // // // //
// // // Routines for CAN Bus // // //
// // // // // // // // // // // // //

#define NextTXTag(PKT) \
	if (OutBuffer_Tags.in != OutBuffer_Tags.out) \
	{\
		PKT->tag = OutBuffer_Tags.buffer[OutBuffer_Tags.out];\
		OutBuffer_Tags.buffer[OutBuffer_Tags.out] = 0;\
		OutBuffer_Tags.out ++;\
		if (OutBuffer_Tags.out >= TXCACHE_SIZE) \
		{\
			OutBuffer_Tags.out = 0;\
		}\
	}

/// Retrieve a packet from the internal circular buffer. If an error is reported, the 'pkt' structure is NOT filled in.
/// Errors take priority over normal data to signify conditions like overrun/etc. All errors, apart from CANERR_RX_BUSOFF are
/// cleared when this function has reported them via the return value. To clear the CANERR_RX_BUSOFF error, a call to CANInit()
/// is required. It may take 1,408 (128*11) consecutive recessive bits before reception and transmit is possible again.
/// You can tell which packets transmit ok as the packet tags will be placed into the tag of RX_OK or RX_NODATA
/// \param pkt Where to put the received data, if any. cplen member must be valid.
/// \return One of CANERR_RX_*
/// \par Side effects
/// May modify file scope variables CANErrors, InBuffer
/// \note
/// If CANERR_RX_TXTIMEOUT occurs, tag of failed packet is placed in pkt->tag
CANErr CANRx (TCANPacket *pkt)
{
	CANErr err = CANERR_INTERNAL_ERROR;

	if (Init_OK)
	{
		if ((pkt) && (pkt->cplen == sizeof(TCANPacket)))
		{
			// Check for errors first
			if (CANErrors)
			{
				// Looks like we got a live one!
				if ((CANErrors & canerr_overrun) != 0)
				{
					// Clear the error, report the error.
					CANErrors &= (~canerr_overrun);
					err = CANERR_RX_OVRUN;
				}
				else if ((CANErrors & canerr_buserror) != 0)
				{
					// Clear the error, report the error.
					CANErrors &= (~canerr_buserror);		
					err = CANERR_RX_BUSERR;
				}
				else if ((CANErrors & canerr_busoff) != 0)
				{
					// Just report the error.
					err = CANERR_RX_BUSOFF;
				}
				else if ((CANErrors & canerr_txtimeout) != 0)
				{
					// Clear the error, report the error
					CANErrors &= (~canerr_txtimeout);
					pkt->tag = LastTXPacketTag_Fail;					
					LastTXPacketTag_Fail = 0;
					err = CANERR_RX_TXTIMEOUT;
				}
				else
				{
					// There's an error, but we don't understand what it is. Maybe a new flag was made up, but not added here.
					err = CANERR_INTERNAL_ERROR;
				}
			}
			else
			{
				// Check for incoming packets
				if (CB_RetrieveRX(&InBuffer, pkt))
				{
					err = CANERR_RX_OK;
				}
				else
				{
					err = CANERR_RX_NODATA;
					pkt->tag = 0;
				}
				NextTXTag(pkt);
			}
		} // if ((pkt) && (pkt->cplen == sizeof(TCANPacket)))
		else
		{
			err = CANERR_RX_INVALIDPKT;
		}
	} // if (Init_OK)
	else
	{
		err = CANERR_NOT_INITIALISED;
		// No point sending out debug info, as the can hasn't been initialised
	}
	return err;
}

/********************************************************************************************************************************/

/// Append the provided packet to the internal circular buffer for sending at the next available opportunity.
/// Once the packet has been placed into a slot for transmission, the 'tout' member of 'initdata' specifies how long
/// to wait before cancelling the packet and allowing the next in the buffer to be sent. 'tout' is reset after every successfull
/// packet transmission.
/// \param pkt The location of the packet to copy into the circular buffer. cplen member must be valid.
/// \return One of CANERR_TX_*
/// \par Side effects
/// May modify file scope variable OutBuffer, TXInt_Finished
CANErr CANTx (TCANPacket *pkt)
{
	CANErr err = CANERR_INTERNAL_ERROR;
//  TESTLED = 1;

	if (Init_OK)
	{
		if ((pkt) && (pkt->cplen == sizeof(TCANPacket)))
		{
			// Place packet into queue
			if (CB_AppendTX(&OutBuffer, pkt))
			{				
				// Packet queued ok. See if we need to kickstart it.
				err = CANERR_TX_OK;
				if (TXInt_Finished)
				{
					// Interrupt will not be called again until another packet has been sent
					TXNextPkt();
				}
			}
			else
			{
				err = CANERR_TX_BUFOVFLOW;
			}
		}
		else
		{
			err = CANERR_TX_INVALIDPKT;
		}
	}
	else
	{
		err = CANERR_NOT_INITIALISED;
	}
//  TESTLED = 0;
	return err;
}

/********************************************************************************************************************************/

/// Flush the receive or transmit circular buffer used for CAN transmission/reception.
/// \param cfg Which buffers to clear (See TCANFlushCfg)
/// \param abort Attempt to abort any current operation on the respective buffers also (eg. if a packet has already started transmission)
/// \return One of CANERR_FLUSH_*
/// \note 'abort' parameter not yet implemented
/// \par Side effects
/// May modify file scope variables InBuffer, OutBuffer
CANErr CANFlush (TCANFlushCfg cfg, uint8 abort)
{
	// TODO: CanFlush doesn't support 'abort' parameter yet.
	CANErr err = CANERR_INTERNAL_ERROR;
	
	if (Init_OK)
	{	
		if ((cfg == CANF_BOTH_BUFFERS) || (cfg == CANF_RX_BUFFER))
		{
			CB_InitialiseRX(&InBuffer);
		}
		
		if ((cfg == CANF_BOTH_BUFFERS) || (cfg == CANF_TX_BUFFER))
		{
			CB_InitialiseTX(&OutBuffer);
		}
		
		err = CANERR_FLUSH_OK;
	}
	else
	{
		err = CANERR_NOT_INITIALISED;
	}	

	return err;	
}

/********************************************************************************************************************************/

/// Place the CAN controller into sleep mode.
/// \return One of CANERR_SLEEP_*
CANErr CANSleep (void)
{
	// TODO: CanSleep
	CANErr err = CANERR_INTERNAL_ERROR;
	
	if (Init_OK)
	{	
		err = CANERR_SLEEP_OK;
		// Shutdown can controller and go into sleep mode. Wake up on incoming packet.
		CANInit_SelectMode(CIM_RESET);
		CANInit_SelectMode(CIM_SLEEP);
		
		// tja1054a standby
		// en = 0x40, stb = 0x20

		//P7 |= 0x20;

		// Normal running - stb high, en high
		// stb high, en high
		//P7 |= 0xC0;
		//P7 |= 0xC0;
		//P7 |= 0xC0;
		//P7 |= 0xC0;
		//P7 |= 0xC0;
		
		// Goto sleep - stb low, en high
		//P7 &= (~0x80);
		//P7 &= (~0x80);
		//P7 &= (~0x80);
		//P7 &= (~0x80);
		//P7 &= (~0x80);
				
		// Sleep - stb low, en low
		//P7 &= (~0x40);
	}
	else
	{
		err = CANERR_NOT_INITIALISED;
	}
	
	return err;
}

/********************************************************************************************************************************/

/// Setup the CAN controller to specification supplied in 'initdata'.
/// A single channel is configured to use as a transmit channel. \n
/// Upto 15 channels are available as receive channels, one per unique id. Fill in the ID's required in the \a initdata structure.
/// \param initdata Initialisation data. idlen field must be valid.
/// \return One of CANERR_INIT_*
/// \note
/// - All masks are hardcoded to FFF. \n
/// - Extended identifiers are not supported. \n
/// - Interrupts are disabled whilst this function executes
/// \par Side Effects
/// Overwrites file scope variables: LocalInitData, Init_OK, CANErrors
CANErr CANInit (TCANInitData *initdata)
{
	CANErr err = CANERR_INIT_FAIL;
  u8 object;

  // make sure the bit position lookup table is created in RAM
  object = 16;
  do
  {
    object--;
    bit_position_lookup_table[ ( ( (u16)0x09af << object ) >> 12 )] = object;
  } while ( object );
	
   if ((initdata) && (initdata->idlen == sizeof(TCANInitData)))
   {
		Init_OK = 0;
		LocalInitData = initdata;

		if ((CANInit_LocalInit()) && (CANInit_SelectCANType(initdata->cantype)))
		{
			// Reset can, and bring into initialisation mode, configure clock, control, masks, then go back into normal mode.

			if (	(CANInit_SelectMode(CIM_RESET)) &&
					(CANInit_ConfigureClock(initdata)) &&
					(CANInit_ConfigureControl()) &&
					(CANInit_ConfigureMasks(initdata)) &&
					(CANInit_SelectMode(CIM_NORMAL)) &&
					(CANInit_ConfigureIdentifiers(initdata)) &&
					(CANInit_ConfigureInterrupts()) )
			{
				Init_OK = 1;
				err = CANERR_INIT_OK;
				// Successfull CAN reset, so clear any pending BUSOFF error.
				CANErrors &= (~canerr_busoff);	

				// Ensure no can packet is attempting to be sent.
				while (C0MCTL0 & 2)
				{
					C0MCTL0 = 0;
				}
			}
		}
	}
   return err;
}

/********************************************************************************************************************************/

/// CAN Maintenance function. Checks for packets that have taken too long to transmit, schedules new packet transmissions, etc.
/// Must be called every 1ms (NOT from an interrupt)
/// \par Side effects
/// May modify file scope variables TXInt_Finished, LastTXPacketTimer, OutBuffer \n
/// \note
/// Call CANInit() before calling this function. (This code should only be run after car side has initialised anyway)
void CANSide (void)
{
	uint8 sleep_ok = 1;

#if 1
	// Check for packets that have taken too long to transmit.
	if (!TXInt_Finished)
	{
		sleep_ok = 0;
		if (LastTXPacketTimer > LocalInitData->tout)
		{
			// Kill the outgoing packet. We know the interrupt can clear the TXInt_Finished flag at any time, but if a packet has
			// been waiting for this long, then the chances are that it's not going to be sent anyway, and even if it did manage
			// to get sent, cancelling an empty buffer pretty much gets changed to a "no operation".
			
			C0MCTL0 = 0;
			if ((C0MCTL0 & 2) == 0)		// As soon as abort is actually carried out, transmitting bit changes back to zero.
			{									// If the kill was ignored, then keep trying each time we come through.
				TXInt_Finished = 1;
				CANErrors |= canerr_txtimeout;
				LastTXPacketTag_Fail = LastTXPacketTag;
				LastTXPacketTag = 0;
				// Do we need to purge the tx buffer?
				if (LocalInitData->flags & CIF_CLRBUFTXER)
				{
					CANFlush(CANF_TX_BUFFER, 0);
				}
			}
		}
		else
		{
			// Only increment the timer when it's less or equal to 'tout' in 'initdata' struct.
			LastTXPacketTimer ++;
		}
	}
	else
#endif
	{
		// If tag isn't invalid (0) then append it to the tag queue as the packet must have been a success
		if (LastTXPacketTag)
		{
			OutBuffer_Tags.buffer[OutBuffer_Tags.in++] = LastTXPacketTag;
			if (OutBuffer_Tags.in >= TXCACHE_SIZE)
			{
				OutBuffer_Tags.in = 0;
			}

			LastTXPacketTag = 0;
		}
	}
	
	// Check to see if we need to schedule a new packet transmission.
	if (OutBuffer.buffer[OutBuffer.out].cplen)
	{
		sleep_ok = 0;
		TXNextPkt();
	}
	
	if (sleep_ok)
	{
		can.system.flags |= CANSYS_SLEEPOK;
	}
	else
	{
		can.system.flags &= (~(CANSYS_SLEEPOK));
	}
}

/********************************************************************************************************************************/

/// \internal
/// Local (file) variable initialisation
/// \return 0 on error, 1 on success
static uint16 CANInit_LocalInit (void)
{
	CB_InitialiseRX(&InBuffer);
	CB_InitialiseTX(&OutBuffer);

	OutBuffer_Tags.in = 0;
	OutBuffer_Tags.out = 0;
	memset (OutBuffer_Tags.buffer, 0, sizeof(OutBuffer_Tags.buffer));
	return 1;
}

/********************************************************************************************************************************/

/// \internal
/// Select the type of CAN required.
/// \param cantype CIT_FAULT_TOLERANT or CIT_HIGH_SPEED
/// \return 0 on error, 1 on success
static uint16 CANInit_SelectCANType (uint8 cantype)
{
	uint16 err;
	
	if (cantype == CIT_FAULT_TOLERANT)
	{
		// Init FT CAN
		//P7 |= 12;			// Disable both can transceivers
		//P7 &= 0xFB;			// Enable just the FT one
		
		// Apply local wake up
		//P7 &= ~0x20;
		
		// Bring STB & EN high for normal operation mode
		//P7 |= 0xC0;

		err = 1;
	}
	else if (cantype == CIT_HIGH_SPEED)
	{
		// Init HS CAN
		//P7 |= 12;			// Disable both can transceivers
		//P7 &= 0xF7;			// Enable just the HS one
		
		//P8 &= 0xFE;			// Bring out of standby mode (STB pin)
		
		err = 1;
	}
	else
	{
		err = 0;				// Failure - unknown argument
	}
	
	return err;
}

/********************************************************************************************************************************/

/// \internal
/// Select CAN mode (Reset/Initialisation or normal operation)
/// \param mode CIM_SLEEP, CIM_RESET or CIM_NORMAL
/// \return 0 on error, 1 on success
/// \note
/// CIM_SLEEP must only be executed after a CIM_RESET.
static uint16 CANInit_SelectMode (uint8 mode)
{
	uint16 timer, modval, ok = 1;
	
	if (mode == CIM_RESET)
	{
		// Reset mode
		C0CTLR |= 1; // Reset can
		C0CTLR &= (~0x20); // Disable sleep
		modval = 0;
	}
	else if (mode == CIM_NORMAL)
	{
		// Normal mode
	   C0CTLR &= (~1); // exit reset mode
		modval = 0x100;	
	}
	else if (mode == CIM_SLEEP)
	{
		C0CTLR |= 0x20;	// Enable sleep
		C0CTLR &= (~1);	// Exit reset mode
	}
	else
	{
		ok = 0;	// Invalid parameter
	}

	if (ok)
	{
		timer = 0xFFFF;
		while (((C0STR & 0x100) == modval) && (timer > 0))
		{
			timer --;
			if (timer == 0)
			{
				ok = 0;
			}
		}		
	}
	
	return ok;
}

/********************************************************************************************************************************/

/// \internal
/// Configure the CAN baudrate as specified in the 'initdata' structure.
/// \param initdata Pointer to initialisation structure
/// \par Requirements
/// Must already be in reset/initialisation mode before calling this function (see CANInit_SelectMode)
/// \return 0 on error, 1 on success
static uint16 CANInit_ConfigureClock (TCANInitData *initdata)
{
   PRCR_bit.PRC0 = 1;            // Enable write to CAN clock select register
   CCLKR &= ~0x0F;               // Clear CAN clock select register
   CCLKR |= (initdata->brp0-1) & 7;  // Setup can clock select, ensuring that cpu interface remains active
   PRCR_bit.PRC0 = 0;            // disable write to CAN clock select register

   ///////////////////////////////
   // Setup CAN configuration register
   ///////////////////////////////

	if (initdata->sam > 2)
	{
		initdata->sam = 2;	// Allows to specify 3 sample points per bit and actually work :)
	}
		
   C0CONR = ((initdata->sjw-1) << 14) | ((initdata->pbs2-1) << 11) | ((initdata->pbs1-1) << 8) |
            ((initdata->pts-1) << 5)  | ((initdata->sam-1) << 4)    | ((initdata->brp1-1));

   return 1;
}

/********************************************************************************************************************************/

/// \internal
/// Configure the CAN control register C0CTLR bit-by-bit using logical operations. Currently the operations are hardcoded.
/// /par What is selected
/// Enable CAN, Disable bus error interrupt (so we only get the major errors, and not ever error that occurs on the bus),
/// Select normal operation, byte access to channel data registers (slots), no loopback, no basic CAN,
/// RetBusOff: Normal operation mode, RXOnly: Normal operation mode.
/// \par Requirements
/// Must already be in reset/initialisation mode before calling this function (see CANInit_SelectMode)
/// \return 0 on error, 1 on success
static uint16 CANInit_ConfigureControl (void)
{
   ///////////////////////////////
   // CAN Control register.   Enable CAN, Disable bus error interrupt, Select normal operation, byte access, no loopback.
   //                         No BASIC CAN, RetBusOff: Normal operation mode, RXOnly: Normal operation mode.
   ///////////////////////////////

   C0CTLR |= 0x40;      // enable can ports
   C0CTLR &= (~2);      // disable loopback
   C0CTLR |= 4;         // msgorder = byte
   C0CTLR &= (~8);      // basic can disabled
   C0CTLR &= (~0x10);   // bus error interrupt disabled
   C0CTLR &= (~0x300);  // time stamp prescaler disabled
   C0CTLR &= (~0x800);  // RetbusOff: Normal operation mode
   C0CTLR &= (~0x2000); // RXOnly: Normal operation mode.

   return 1;
}

/********************************************************************************************************************************/

/// \internal
/// Configure the local and global masks
/// \return 0 on error, 1 on success
static uint16 CANInit_ConfigureMasks (TCANInitData *initdata)
{
  ///////////////////////////////
  // Initialise CAN masks
  ///////////////////////////////

  // global mask
  C0GM0L = (uint8)(initdata->gmask & 0x3f);
  C0GM0H = (uint8)((initdata->gmask >> 6 ) & 0x1f);

  C0GM2H = (uint8)(initdata->gmaskext & 0x3f);
  C0GM1L = (uint8)(initdata->gmaskext >> 6 );
  C0GM1H = (uint8)((initdata->gmaskext >> 14 ) & 0x3f );

  // local A mask
  C0LMA0L = (uint8)(initdata->maska & 0x3f);
  C0LMA0H = (uint8)((initdata->maska >> 6 ) & 0x1f);

  C0LMA2H = (uint8)(initdata->maskaext & 0x3f);
  C0LMA1L = (uint8)(initdata->maskaext >> 6 );
  C0LMA1H = (uint8)((initdata->maskaext >> 14 ) & 0x3f );

  // local B mask
  C0LMB0L = (uint8)(initdata->maskb & 0x3f);
  C0LMB0H = (uint8)((initdata->maskb >> 6 ) & 0x1f);

  C0LMB2H = (uint8)(initdata->maskbext & 0x3f);
  C0LMB1L = (uint8)(initdata->maskbext >> 6 );
  C0LMB1H = (uint8)((initdata->maskbext >> 14 ) & 0x3f );

  return 1;
}

/********************************************************************************************************************************/

/// \internal
/// Configure the slots from 1 onwards to contain the identifiers present in the 'initdata' structure. Any identifier in the 'inidata'
/// structure which is set to zero will be ignored.
/// \par Requirements
/// Must not be in initialisation mode to use this function.
/// \note
/// Extended identifiers are not supported.
/// \return 0 on error, 1 on success
static uint16 CANInit_ConfigureIdentifiers (TCANInitData *initdata)
{
   uint8 slotid;
	uint16 ok = 1;

   for (slotid = 1; (slotid < 16) && ok; slotid ++)
   {
      volatile uint8 *slotaddr;									// Slot data buffer (ID, DLC, DATA0..7)
      volatile uint8 *slotctrl = (uint8 *)0x1300;		// Slot control register base
      sint16 id;														// Temporary used to hold id from initdata structure
      uint16 timer;													// 'Get out' timer just in case slot doesn't respond

		timer = 0xFFFF;
		while ((slotctrl[slotid] != 0) && (timer > 0))		// Ensure the channel is disabled
		{
			slotctrl[slotid] = 0;
			timer --;
			if (timer == 0)
			{
				ok = 0;
			}
		}
		
		id = initdata->ids[slotid-1];
		if (ok && (id!=-1))
		{
			slotaddr = (uint8 *)(0x1360 + (slotid * 16));		// Calculate address to slot data buffer
			slotaddr[0] = (uint8) (id >> 6);						// Setup standard identifier	6 .. 10
			slotaddr[1] = (uint8) (id & 0x3F);					//										0 .. 5
			slotaddr[2] = 0;											// Ensure entended identifier is zeroed
			slotaddr[3] = 0;
			slotaddr[4] = 0;
			slotctrl[slotid] = 0x40;								// Setup slot as receive data frame
		}
   }
   C0IDR = 0;	// No extended identifiers are used

   return ok;
}

/********************************************************************************************************************************/

/// \internal
/// Configure the allowed CAN interrupts. Setup the following: Wakeup (Pri 1), Receive (Pri 2), Transmit (Pri 1), Error (Pri 1).
/// Receive interrupt is given higher priority than the rest to help avoid data loss
/// \return 0 on error, 1 on success
static uint16 CANInit_ConfigureInterrupts (void)
{
   // For some reason, C0ICR (Interrupt control register) is missing from the system header file...
   volatile uint16 *c0icr = (uint16 *)0x1316;
   *c0icr = 0xFFFF;							// Interrupt disable on all channels

   C01WKIC = 0; // CAN wakeup interrupt = enabled (priority 1)
   C0RECIC = 0; // CAN receive takes priority over transmit
   C0TRMIC = 0; // CAN transmit ...
   C01ERRIC = 0; // CAN error ...

   // If we were waiting for CAN TX Completion (via interrupt), cancel it now.
   TXInt_Finished = 1;
   // Re-enable interrupts if they were enabled before (or leave them disabled if not)

   return 1;
}

/********************************************************************************************************************************/

/// \internal
/// Takes a packet from the outgoing circular buffer (if any) and requests transmission onto the can bus network. This routine
/// can fail for a number of reasons: If the transmission interrupt (specified by TXInt_Finished) is still active; if there is
/// already another packet being sent; if we can't disable the transmit channel.
/// \return 0 on error, 1 on success
static uint16 TXNextPkt(void)
{
	volatile uint8 *slot0 = (uint8 *)0x1360;	// Address of our sending channel (ch# 0)
	uint16 ok = 1;
	
	// Double check to make sure a packet is not already being sent.
	
	if ((TXInt_Finished) && ((C0MCTL0 & 2) == 0))	// Ensure tx interrupt has finished, and that a packet is not already being transmitted.
	{																
		TCANPacket xmit;
		if (CB_RetrieveTX(&OutBuffer, &xmit))
		{
			if (xmit.cplen == sizeof(TCANPacket))
			{
				// First thing to do is disable the transmit channel. This may not happen straight away, so we need to confirm it.
				uint16 timer = 0xFFFF;

				while ((C0MCTL0 != 0) && (timer > 0))
				{
					C0MCTL0 = 0;
					timer --;
					if (timer == 0)
					{
						ok = 0;
					}
				}

				if (ok)
				{
					// Controller should now accept the packet, so should be safe to set the TXInt_Finished flag.
					// This will cause us not to be called again until we have a successfull acknowledgement on this one.
					TXInt_Finished = 0;

					// Setup slot with ID and data.
					slot0[0] = (uint8)((xmit.id) >> 6);			// SID 6 - 10
					slot0[1] = (uint8)((xmit.id) & 0x3F);		// SID 0 - 5
					slot0[2] = 0;													// EID 14 - 17 (Unused)
					slot0[3] = 0;													// EID 6 - 13  (Unused)
					slot0[4] = 0;													// EID 5 - 0   (Unused)
					slot0[5] = xmit.dlc;											// DLC
				
					uint16 lp;
					for (lp = 0; lp < xmit.dlc; lp ++)
					{
						slot0[6+lp] = xmit.data[lp];
					}
						
					C0MCTL0 |= 0x80;												// Queue for transmission
					LastTXPacketTimer = 0;										// New packet has been sent
					LastTXPacketTag = xmit.tag;
				}
			}
			else
			{
				ok = 0; // Invalid 'cplen'
			}
		}
		else
		{
			ok = 0; // Nothing to send
		}			
	}
	else
	{
		ok = 0; // Interrupt isn't finished, or there is a packet already being sent.
	}
	
	return ok;
}

/********************************************************************************************************************************/

// // // // // // // // // // // // // // // // // // //
// // // Routines for cicular buffer management // // //
// // // // // // // // // // // // // // // // // // //

/// \internal
/// Initialise a circular buffer. Pointers are setup and buffers are initialised. The number of buffers to use is specified
/// in the can_internal.h file. Any existing data in the buffers is lost.
/// \param cb Circular buffer to initialise
/// \return 0 on failure, 1 on success.
static uint16 CB_InitialiseRX (TCANRXCircularBuffer *cb)
{
	uint16 ok = 1;
	
	if (cb)
	{
		cb->in = 0;
		cb->out = 0;
		
		uint16 lp;
		for (lp = 0; lp < RXCACHE_SIZE; lp++)
		{
			cb->buffer[lp].cplen = 0;						// Valid packets have a valid 'cplen', so this marks all packets as invalid
		}
	}
	else
	{
		ok = 0;
	}
	
	return ok;
}
static uint16 CB_InitialiseTX (TCANTXCircularBuffer *cb)
{
	uint16 ok = 1;
	
	if (cb)
	{
		cb->in = 0;
		cb->out = 0;
		
		uint16 lp;
		for (lp = 0; lp < TXCACHE_SIZE; lp++)
		{
			cb->buffer[lp].cplen = 0;						// Valid packets have a valid 'cplen', so this marks all packets as invalid
		}
	}
	else
	{
		ok = 0;
	}
	
	return ok;
}


/********************************************************************************************************************************/

/// \internal
/// Append a packet to a circular buffer.
/// \param cb Circular buffer to append the packet to
/// \param srcpkt The packet to append
/// \return 0 on failure (eg. no room to store packet), 1 on success.
/// \note The packet is not checked for correctness.
static uint16 CB_AppendTX (TCANTXCircularBuffer *cb, TCANPacket *srcpkt)
{
	uint16 ok = 1;
	
	if (cb && srcpkt)
	{
		// If the current can packet referenced by the in pointer is usable, stick the data there, otherwise return failure.
		if (!cb->buffer[cb->in].cplen)
		{
			memcpy (&cb->buffer[cb->in], srcpkt, sizeof(TCANPacket));
			cb->in ++;
			if (cb->in == TXCACHE_SIZE)
			{
				cb->in = 0;
			}
		}
		else
		{
			ok = 0;	// No room to store packet
		}
	}
	else
	{
		ok = 0;	// A 'NULL' was passed as a parameter
	}
	return ok;
}

//static uint16 CB_AppendRX (TCANRXCircularBuffer *cb, TCANPacket *srcpkt)
//{
//	uint16 ok = 1;
//	
//	if (cb && srcpkt)
//	{
//		// If the current can packet referenced by the in pointer is usable, stick the data there, otherwise return failure.
//		if (!cb->buffer[cb->in].cplen)
//		{
//			memcpy (&cb->buffer[cb->in], srcpkt, sizeof(TCANPacket));
//			cb->in ++;
//			if (cb->in == RXCACHE_SIZE)
//			{
//				cb->in = 0;
//			}
//		}
//		else
//		{
//			ok = 0;	// No room to store packet
//		}
//	}
//	else
//	{
//		ok = 0;	// A 'NULL' was passed as a parameter
//	}
//	return ok;
//}

/********************************************************************************************************************************/

/// \internal
/// Attempt to retrieve a packet from a circular buffer.
/// \param cb Circular buffer to check for new data
/// \param destpkt Where to store the data, if any is found
/// \return 0 on failure (eg. no data was found), 1 on success.
static uint16 CB_RetrieveTX (TCANTXCircularBuffer *cb, TCANPacket *destpkt)
{
	uint16 ok = 1;
	
	if (cb && destpkt)
	{
		if (cb->buffer[cb->out].cplen)	// Anything there?
		{
			// copy the packet to the destination
			memcpy (destpkt, &cb->buffer[cb->out], sizeof(TCANPacket));
			cb->buffer[cb->out].cplen = 0;	// Allow the packet buffer to be reused
			cb->out ++;
			if (cb->out == TXCACHE_SIZE)
			{
				cb->out = 0;
			}
		}
		else
		{
			ok = 0; // Nothing there
		}
	}
	else
	{
		ok = 0;	// A 'NULL' was passed as a parameter
	}
	
	return ok;
}
static uint16 CB_RetrieveRX (TCANRXCircularBuffer *cb, TCANPacket *destpkt)
{
	uint16 ok = 1;
	
	if (cb && destpkt)
	{
		if (cb->buffer[cb->out].cplen)	// Anything there?
		{
			// copy the packet to the destination
			memcpy (destpkt, &cb->buffer[cb->out], sizeof(TCANPacket));
			cb->buffer[cb->out].cplen = 0;	// Allow the packet buffer to be reused
			cb->out ++;
			if (cb->out == RXCACHE_SIZE)
			{
				cb->out = 0;
			}
		}
		else
		{
			ok = 0; // Nothing there
		}
	}
	else
	{
		ok = 0;	// A 'NULL' was passed as a parameter
	}
	
	return ok;
}

/********************************************************************************************************************************/

//#define TOGGLE_LED {static uint16 p=0;if(p){p=0;P0_bit.P0_0=0;}else{p=1;P0_bit.P0_0=1;}}

void can_int (void)
{
  uint16 canstat = C0STR;
  uint16 mailbox;

  static u8 CANErrStat;

  if ((canstat & 0x2000) != 0)
  {
    if ( ! (CANErrStat && canerr_buserror) )
    {
      CANErrors |= canerr_buserror;
      CANErrStat |= canerr_buserror;
    }
  }
  else
  {
    CANErrStat &= (~canerr_buserror);
  }

  if ((canstat & 0x4000) != 0)	
  {
    if ( ! (CANErrStat && canerr_busoff) )
    {
      CANErrors |= canerr_busoff;
      CANErrStat |= canerr_busoff;
    }
  }
  else
  {
    CANErrStat &= (~canerr_busoff);
  }



  // check for xmit and set TXInt_Finished if so
  if (C0SSTR & 0x0001) // tx slot 1 completed
  {
    C0MCTL0 = 0;										// Transmission finished, kill the tx channel (nb: may be ignored by controller)
    TXInt_Finished = 1;								// Pending transmission interrupt has been called, so no more packet queuing please
    //TOGGLE_LED;
  }


  while ( mailbox = ( C0SSTR & 0xFFFE) )
  {
//    TESTLED = 1;

    uint8 mbox = bit_position_lookup_table[(((mailbox & (-mailbox )) * (u16)0x09af ) >> 12 )];

    volatile uint8 *slotaddr = (uint8 *) (0x1360 + (mbox * 16));
    volatile uint8 *slotctrl = (uint8 *) (0x1300 + mbox);

    //TOGGLE_LED;

    // CAN controller received a meessage successfully.
    TCANPacket *pkt = &InBuffer.buffer[InBuffer.in]; // Basically a quick way of doing CB_Append

    if (!pkt->cplen)	// Check to see if there is room
    {
      pkt->id = slotaddr[0]; pkt->id <<= 6;		// Standard identifier (TODO: Read from our initdata structure instead below)
      pkt->id |= (slotaddr[1] & 0x3F);
      pkt->dlc = slotaddr[5];							// Data length
      pkt->data[0] = slotaddr[6];					// Pull out all 8 bytes regardless of length (usually quicker than conditional reads)
      pkt->data[1] = slotaddr[7];
      pkt->data[2] = slotaddr[8];
      pkt->data[3] = slotaddr[9];
      pkt->data[4] = slotaddr[10];
      pkt->data[5] = slotaddr[11];
      pkt->data[6] = slotaddr[12];
      pkt->data[7] = slotaddr[13];
      (*slotctrl) &= 0xFE;								// Mark slot as read.

      if (((*slotctrl) & 6) != 0)					// Check to see if another packet has arrived whilst we were processing this one
      {
        // Yup, it's been overwritten whilst we were reading it. Don't place into buffer as it could be corrupt.
        CANErrors |= canerr_overrun;	
        (*slotctrl) &= 0xFB;							// Clear overwrite flag
      }
      else
      {
        pkt->cplen = sizeof (TCANPacket);		// Mark packet as valid
        pkt->tag = 0;
        InBuffer.in ++;
        if (InBuffer.in == RXCACHE_SIZE)
        {
          InBuffer.in = 0;
        }
      }				
    }
    else
    {
      // No room to store the packet, so we need to loose it.
      (*slotctrl) &= 0xFA;								// Mark slot as read, and clear any possible overwrite flag
      CANErrors |= canerr_overrun;						
    }
  }
//  TESTLED = 0;
}
