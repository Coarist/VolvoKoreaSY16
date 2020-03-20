#ifndef ISO15765_H
#define ISO15765_H

typedef enum
{
  ISODIR_RX, // Receive only
  ISODIR_TX, // Transmit only
  ISODIR_BI, // Receive & Transmit (One at a time only, or packets will be prematurely terminated!)
} ISO15765_Dir;
/// Everything about an ISO15765 channel is listed here.
/// Since we may receive multiple, segmented, ISO15765 packets, it's a good idea to have the buffer local to each ISO15765 channel too.
/// Maximum value for timers (tstmin, pkttimer, tsttimer) is 32767ms.
typedef struct
{
  uint16 chid;                  ///< Channel ID, specifies position in channel array, used when dereferencing a channel pointer
  uint16 xmitid;                  ///< Channel for sending or receiving data on
  uint16 rcvid;
  uint16 gstate;                    ///< State of the channel, such as idle
  uint16 tstate;                    ///< Connection state of the channel
  uint8  *buffer;                 ///< Buffer storage for packet
  uint16 buffer_length;
  uint16 buffer_pos;                ///< How many bytes we have placed into the buffer so far (upto pkt_length)
  uint16 pkt_length;                ///< Actual length of ISO15765 packet
  uint16 next_seq;                  ///< For segmented messages, indicates sequence number
  uint16 completed;                 ///< Indicates whether or not the packet has completed transmission
  uint16 tstmin;                    ///< Minimum time gap between transmission of consecutive data frames
  uint16 tbs;                     ///< Transmit block size (number of packets to send before waiting for next FC)
  sint16 pkttimer;                  ///< Maximum time gap between FCs or CFs before considering an error
  sint16 tsttimer;                  ///< tstmin timer, when zero, sends out another packet if bs != 0
  uint16 retries;                 ///< Number of resends so far of a certain packet type
  uint16 flags;                   ///< Flags, reset to zero when a packet is resent from the beginning
  uint16 fp_bs;                   ///< 'BS' value from the first FC packet
  uint16 fp_st;                   ///< 'ST' value from the first FC packet
  ISO15765_Dir dir;
} ISO15765_Channel;

extern uint16 ISO15765_Initialise (void);
extern uint16 ISO15765_Connect (ISO15765_Channel *chan, uint16 chid, uint16 xmitid, uint16 rcvid, 
                                 uint8 *buffer, uint16 buffer_length, ISO15765_Dir dir);
extern uint16 ISO15765_ChTx (ISO15765_Channel *chan, uint8 *pkt, uint16 length);
extern uint16 ISO15765_Status (ISO15765_Channel *chan);
extern uint16 ISO15765_RunCycle (ISO15765_Channel *chan);
extern uint16 ISO15765_ProcessPkt (ISO15765_Channel *chan, TCANPacket *pkt);
extern uint16 ISO15765_IsPacketWaiting (ISO15765_Channel *chan);
extern uint16 ISO15765_Rx (ISO15765_Channel *chan, uint16 *id, uint8 *pkt, uint16 *length);
extern void ISO15765_ReportFailure (ISO15765_Channel *chan, uint16 tag);
extern void ISO15765_ReportSuccess (ISO15765_Channel *chan, uint16 tag);

enum
{
  ISO15765_INVALID_CONN_ID = 0xffff,        ///< Invalid connection id

  ISO15765_GSTATE_AWAITING  = 0x10,         ///< ISO15765 channel is waiting for one of the ISO15765_GSTATE_AWAIT_*
  ISO15765_GSTATE_AWAIT_CF = 0x12,          ///< ISO15765 channel is waiting for the next packet of a multi-segmented packet (RX)
  ISO15765_GSTATE_AWAIT_FC = 0x13,          ///< ISO15765 channel is waiting for flow control (TX)
  ISO15765_GSTATE_INVALID = 0x21,         ///< ISO15765 channel is not valid
  ISO15765_GSTATE_IDLE    = 0x31,         ///< ISO15765 channel not transmitting, receiving, or waiting for an event

  ISO15765_TSTATE_CONNOK  = 0x12,         ///< ISO15765 channel is connected
  ISO15765_TSTATE_INVALID = 0x21          ///< ISO15765 channel is not valid
};

enum
{
  ISO15765S_PACKET_WAITING = 1,
  ISO15765S_TX_OK = 2,
  ISO15765S_TX_ERROR_MAJOR = 3,           // ECU falls into degraded mode and attempts to reconnect
  ISO15765S_TX_ERROR_MINOR = 6,           // Message is ignored
  ISO15765S_IDLE = 4,
  ISO15765S_CONNECTION_ERROR = 5
};

/// Flow Control Flow Status (FCFS)
enum
{
  FCFS_CTS = 0,                   ///< Continue to send
  FCFS_WAIT = 1,                    ///< Wait - Do not send any more packets yet
  FCFS_OVERFLOW = 2                 ///< Buffer overflow, abort transmission
};
#endif
