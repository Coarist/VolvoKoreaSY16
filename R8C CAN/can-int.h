#ifndef CAN_CAN_H
#define CAN_CAN_H

#include "common.h"

/// Initialisation structure for CANInit routine.
/// Note that masks are all set to FFF, but you can fill in the identifiers you are interested in by using the 'ids' field. \n
/// Only 11-bit standard identifiers are supported, and "Request To Return" style packets are not supported. \n\n
/// The configuration values are used to configure the can bus as follows: \n
/// \image html ssptspbs.jpg
/// \n The formula to calculate the baud rate from this structure is as follows: \n
/// \image html baudcalc.jpg
/// \n Where 'numtq' is the sum of SS(1), PTS, PBS1 and PBS2.\n\n
/// For more information on bit-timing, see the M16C29 pdf page 316 (page 296 of the hardware manual).
/// \note This structure uses "real values" (1 onwards), not "register values" (0 onwards).
typedef struct
{
   uint16 idlen;	    ///< Length of this structure, in bytes
   uint8 cantype;     ///< Can type    (--)                 (0 = Hispeed can, 1 = Full tolerant can)
   uint8 brp0;        ///< Prescaler 0 (CCLKR bits 0 - 2)   (Provides fCAN from crystal - divide factor is (2^brp0) - 5+ prohibited)
   uint8 brp1;        ///< Prescaler 1 (C0CONR bits 0 - 3)  (fCAN divided by this value, 1 to 16)
   uint8 sam;         ///< Sample Ctrl (C0CONR bit 4)       (1 = One time sampling, 2 or 3 = Three times sampling)
   uint8 pts;         ///< PropTimeSeg (C0CONR bits 6 - 7)  (1 to 8 tq) Propogation time segment
   uint8 pbs1;        ///< PhaseBufSeg (C0CONR bits 8 - 10) (1 to 8 tq) Phase buffer segment 1
   uint8 pbs2;        ///< PhaseBufSeg (C0CONR bits 11 - 13)(1 to 8 tq) Phase buffer segment 2
   uint8 sjw;         ///< SyncJump    (C0CONR bits 14 - 15)(1 to 4 tq) Synchronisation jump width
   uint16 tout;	      ///< Timeout     (--)                 (1 - 65535) Packet timeout. Packets not sent after this timeout are cancelled.
   sint32 ids[15];    ///< Identifiers (C0S0MI0)            (array of 11 bit identifiers - use zero for unused entries)
   uint16 flags;	    ///< Flags                            (flags for use inside the can routines)
   uint16 gmask;      ///< global mask for 11 bit ID's
   uint16 gmaskext;   ///< top 18 bits for global mask for 29 bit ID's
   uint16 maska;      ///< mask A for 11 bit ID's
   uint16 maskaext;   ///< top 18 bits for mask A for 29 bit ID's
   uint16 maskb;      ///< mask B for 11 bit ID's
   uint16 maskbext;   ///< top 18 bits for mask B for 29 bit ID's
} TCANInitData;

/// Incoming or outgoing CAN packet. Expected by CANTx and CANRx routines.
typedef struct
{
	uint16	cplen;	///< Length of this structure, in bytes
   uint16   id;      ///< 11-bit can identifier to use/received
   uint8		dlc;		///< Datalength - 1 to 8 bytes. Invalid packet lengths will cause an error.
   uint8    data[8]; ///< Data to send/received
   uint16	tag;		///< 16-bit value to identify this packet (only used internally, not presented onto the bus)
} TCANPacket;

/// External CAN variables - accessible by other files (eg. main.c)
typedef struct
{
	struct
	{
		volatile uint16 flags;	///< External flags available (see CANSYS_*)
	} system;
} TExtCANInfo;

/// Definitions for the value of 'flags' in the TExtCANInfo structure.
enum
{
	CANSYS_SLEEPOK = 1					/*!<	CAN system can enter sleep mode. Note that this will not be set when any of the following
														are true: The TX or RX buffers are not empty; a transmit or receive is in progress;
														there are unread error codes. */
};

/// Definitions for the CANType in the 'initdata' structure
typedef enum
{
	CIT_FAULT_TOLERANT = 1,
	CIT_HIGH_SPEED = 0
} CANType;

/// Definitions for the 'flags' in the 'initdata' structure
enum
{
	CIF_CLRBUFTXER = 1						///< When a CAN TX Timeout event occurs, request the can module to purge the TX buffer.
};

/// CAN Status information returned from the various CAN routines
typedef enum
{
		CANERR_NOT_INITIALISED = 100,	///< (Global)-		CANRx() CANTx() etc called without first successfully initialising via CANInit()		
		CANERR_INTERNAL_ERROR = 101,	///< (Global)-		Something nasty happened, probably a bug

		CANERR_RX_NODATA = 0, 			///< CANRx() -		There are currently no packets or errors waiting for retrieval
		CANERR_RX_OK = 1, 				///< CANRx() -		A packet was received successfully and stored in the provided buffer
		CANERR_RX_OVRUN = 2, 			///< CANRx() -		Receive buffer has exceeded it's maximum capacity, the last received data was lost.
		CANERR_RX_BUSERR = 3, 			///< CANRx() -		A large number (ie. more than 127) of receive or transmit errors have occured
		CANERR_RX_BUSOFF = 4,			/*!< CANRx() -		Too many (ie. more than 256) errors have occured on the CAN bus, and the controller has
																		entered bus-off state */
		CANERR_RX_INVALIDPKT = 5,		///< CANRx() -		CANRx() was called with an NULL parameter, or the TCANPacket structure was invalid
		CANERR_RX_TXTIMEOUT = 6,		///< CANRx() -		Notification of a failed can transmission due to timeout

		CANERR_TX_BUFOVFLOW = 0, 		///< CANTx() -		An attempt has been made to send a CAN packet when there is no buffer space available
		CANERR_TX_INVALIDPKT = 2,		///< CANTx() -		CANTx() was called with an NULL parameter, or the TCANPacket structure was invalid
		CANERR_TX_OK = 1,					/*!< CANTx() -		The packet was succesfully allocated and copied into an outgoing buffer. It will be
																		sent asap. */

		CANERR_INIT_FAIL = 0, 			/*!< CANInit() -	Either the CAN controller failed to acknowledge a "reset", "normal operation" or
																		channel disable request (in which case, try again later), or the contents of the
																		TCANInit structure was invalid. */
		CANERR_INIT_OK = 1,				///< CANInit() -	The CAN controller was successfully configured with the provided values.

		CANERR_FLUSH_FAIL = 0, 			///< CANFlush() - Failed to flush the selected buffers, or the passed TCANFlushCfg value was invalid
		CANERR_FLUSH_OK = 1,				///< CANFlush() - The selected buffers flushed successfully

		CANERR_SLEEP_FAIL = 0, 			/*!< CANSleep() - Failed to enter sleep mode, or failed to exit sleep mode. The CAN controller may be in
																		an undetermined state. */
		CANERR_SLEEP_OK = 1				///< CANSleep() - Sleep mode was exited properly
} CANErr;

/// Parameter to CANFlush()
typedef enum
{
		CANF_RX_BUFFER,					///< Flush the RX buffer
		CANF_TX_BUFFER,					///< Flush the TX buffer
		CANF_BOTH_BUFFERS					///< Flush both buffers
} TCANFlushCfg;

extern TExtCANInfo can;

//// Prototypes ////

/// Retrieve a packet from the internal circular buffer. If an error is reported, the 'pkt' structure is NOT filled in.
/// Errors take priority over normal data to signify conditions like overrun/etc. All errors, apart from CANERR_RX_BUSOFF are
/// cleared when this function has reported them via the return value. To clear the CANERR_RX_BUSOFF error, a call to CANInit()
/// is required. It may take 1,408 (128*11) consecutive recessive bits before reception and transmit is possible again.
/// \param pkt Where to put the received data, if any. cplen member must be valid.
/// \return One of CANERR_RX_*
/// \par Side effects
/// May modify file scope variables CANErrors, InBuffer
/// \note
/// This function may change to allow for can transmit failure errors codes to be received.
CANErr CANRx (TCANPacket *pkt);

/// Append the provided packet to the internal circular buffer for sending at the next available opportunity.
/// Once the packet has been placed into a slot for transmission, the 'tout' member of 'initdata' specifies how long
/// to wait before cancelling the packet and allowing the next in the buffer to be sent. 'tout' is reset after every successfull
/// packet transmission.
/// \param pkt The location of the packet to copy into the circular buffer. cplen member must be valid.
/// \return One of CANERR_TX_*
/// \par Side effects
/// May modify file scope variable OutBuffer, TXInt_Finished
/// \note
/// This function may change to allow for can transmit failure errors codes to be received.
CANErr CANTx (TCANPacket *pkt);

/// Flush the receive or transmit circular buffer used for CAN transmission/reception.
/// \param cfg Which buffers to clear (See TCANFlushCfg)
/// \param abort Attempt to abort any current operation on the respective buffers also (eg. if a packet has already started transmission)
/// \return One of CANERR_FLUSH_*
/// \note 'abort' parameter not yet implemented
/// \par Side effects
/// May modify file scope variables InBuffer, OutBuffer
__monitor CANErr CANFlush (TCANFlushCfg cfg, uint8 abort);

/// Place the CAN controller into sleep mode. Will not return until sleep mode has exited.
/// \return One of CANERR_SLEEP_*
/// \note Not yet implemented
CANErr CANSleep (void);

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
__monitor CANErr CANInit (TCANInitData *initdata);

/// CAN Maintenance function. Checks for packets that have taken too long to transmit, schedules new packet transmissions, etc.
/// Must be called every 1ms (NOT from an interrupt)
/// \par Side effects
/// May modify file scope variables TXInt_Finished, LastTXPacketTimer, OutBuffer \n
/// \note
/// Call CANInit() before calling this function. (This code should only be run after car side has initialised anyway)
void CANSide (void);

#endif
