#ifndef CAN_CAN_INTERNAL_H
#define CAN_CAN_INTERNAL_H

/// See TCANCircularBuffer
enum {CACHE_SIZE = 10};

/// Holds CACHE_SIZE amount of TCANPacket structures
typedef struct
{
	volatile uint16 in;					///< Pointer for storing into the buffer
	uint16 out;								///< Pointer for retrieving out of the buffer
	TCANPacket buffer[CACHE_SIZE];	///< Actual buffer storage
} TCANCircularBuffer;

typedef struct  
{
	uint16 in;
	volatile uint16 out;
	uint16 buffer[CACHE_SIZE];
} TCANTagCircBuffer;

/// Enumerations/Bits for the 'canerrors' variable.
/// NB: Some bits may be set by interrupt. Ensure to disable interrupts before clearing. */
enum
{
	canerr_busoff = 1,									/*!<	Bus off error from controller.
																		Reported by CANRx(). Cleared by CANInit(). */
	canerr_buserror = 2,									/*!<	If set, > 127 errors have occured in either transmission or reception.
																		Reported and cleared by a call to CANRx(). */
	canerr_overrun = 4,									/*!<	If set, a data overrun has occured, causing data to be overwritten.
																		Reported and cleared by a call to CANRx(). */
	canerr_txtimeout = 8									/*!<	If set, can packet failed to transmit during the timeout specified
																		in the 'initdata' structure. Reported and cleared by a call to CANRx(). */
};

/// Enumation for CANInit_SelectMode()
enum
{
	CIM_RESET = 1,
	CIM_NORMAL = 0,
	CIM_SLEEP = 2
};

// For internal use in can.c only.

/// \internal
/// Local (file) variable initialisation
/// \return 0 on error, 1 on success
static uint16 CANInit_LocalInit (void);

/// \internal
/// Select the type of CAN required.
/// \param cantype CIT_FAULT_TOLERANT or CIT_HIGH_SPEED
/// \return 0 on error, 1 on success
static uint16 CANInit_SelectCANType (uint8 cantype);

/// \internal
/// Select CAN mode (Reset/Initialisation or normal operation)
/// \param mode CIM_RESET or CIM_NORMAL
/// \return 0 on error, 1 on success
static uint16 CANInit_SelectMode (uint8 mode);

/// \internal
/// Configure the CAN baudrate as specified in the 'initdata' structure.
/// \param initdata Pointer to initialisation structure
/// \par Requirements
/// Must already be in reset/initialisation mode before calling this function (see CANInit_SelectMode)
/// \return 0 on error, 1 on success
static uint16 CANInit_ConfigureClock (TCANInitData *initdata);

/// \internal
/// Configure the CAN control register C0CTLR bit-by-bit using logical operations. Currently the operations are hardcoded.
/// /par What is selected
/// Enable CAN, Disable bus error interrupt (so we only get the major errors, and not ever error that occurs on the bus),
/// Select normal operation, byte access to channel data registers (slots), no loopback, no basic CAN,
/// RetBusOff: Normal operation mode, RXOnly: Normal operation mode.
/// \par Requirements
/// Must already be in reset/initialisation mode before calling this function (see CANInit_SelectMode)
/// \return 0 on error, 1 on success
static uint16 CANInit_ConfigureControl (void);

/// \internal
/// Configure the local and global masks to use all bits (0xFFFF).
/// \return 0 on error, 1 on success
static uint16 CANInit_ConfigureMasks (TCANInitData *initdata);

/// \internal
/// Configure the slots from 1 onwards to contain the identifiers present in the 'initdata' structure. Any identifier in the 'inidata'
/// structure which is set to zero will be ignored.
/// \par Requirements
/// Must not be in initialisation mode to use this function.
/// \note
/// Extended identifiers are not supported.
/// \return 0 on error, 1 on success
static uint16 CANInit_ConfigureIdentifiers (TCANInitData *initdata);

/// \internal
/// Configure the allowed CAN interrupts. Setup the following: Wakeup (Pri 1), Receive (Pri 2), Transmit (Pri 1), Error (Pri 1).
/// Receive interrupt is given higher priority than the rest to help avoid data loss
/// \return 0 on error, 1 on success
static uint16 CANInit_ConfigureInterrupts (void);

/// \internal
/// Initialise a circular buffer. Pointers are setup and buffers are initialised. The number of buffers to use is specified
/// in the can_internal.h file. Any existing data in the buffers is lost.
/// \param cb Circular buffer to initialise
/// \return 0 on failure, 1 on success.
static uint16 CB_Initialise (TCANCircularBuffer *cb);

/// \internal
/// Append a packet to a circular buffer.
/// \param cb Circular buffer to append the packet to
/// \param srcpkt The packet to append
/// \return 0 on failure (eg. no room to store packet), 1 on success.
/// \note The packet is not checked for correctness.
static uint16 CB_Append (TCANCircularBuffer *cb, TCANPacket *srcpkt);

/// \internal
/// Attempt to retrieve a packet from a circular buffer.
/// \param cb Circular buffer to check for new data
/// \param destpkt Where to store the data, if any is found
/// \return 0 on failure (eg. no data was found), 1 on success.
static uint16 CB_Retrieve (TCANCircularBuffer *cb, TCANPacket *destpkt);

/// \internal
/// Takes a packet from the outgoing circular buffer (if any) and requests transmission onto the can bus network. This routine
/// can fail for a number of reasons: If the transmission interrupt (specified by TXInt_Finished) is still active; if there is
/// already another packet being sent; if we can't disable the transmit channel.
/// \return 0 on error, 1 on success
static uint16 TXNextPkt(void);

#endif
