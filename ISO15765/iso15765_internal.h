#ifndef ISO15765_INTERNAL_H
#define ISO15765_INTERNAL_H

#define ISO15765CHANNELBUFFERSIZE 127

#define MAXIMUM_ISO15765_RETRIES 6

/// Protocol Control Information (PCI) values
/// The ISO spec only ever uses the upper nibble for the PCI type, an the lower nibble for data.
enum
{
  PCI_SF = 0x00,                    ///< Single frame
  PCI_FF = 0x10,                    ///< First frame of multi segmented packet
  PCI_CF = 0x20,                    ///< Consecutive frame of multi segmented packet
  PCI_FC = 0x30                   ///< Flow control information
};

// ISO15765 flags
enum
{
  ISO15765F_INVALIDPKT = 1,             ///< Packet is invalid, don't process any further
  ISO15765F_MINOR_ERROR = 2,              ///< Don't reset the bus when the retry counter exceeds it's maximum
  ISO15765F_RECEIVED_FCCTS = 4,           ///< We have received the CTS information for the packet
  ISO15765F_RETRY_DELAY = 8,              ///< We are in the retry stage of a packet resend, we ignore incoming packets trying to be from the previous packet
  ISO15765F_WAITING_TXOK = 16             ///< Waiting for the last packet to transmit ok before sending another
};

enum
{
  N_Bs = 250,                   
  TL_A = 250,
  TL_B = 70,
  CF_TIME_MAX = 250,                ///< Time until reception of next CF N_PDU expires
  TIMER_RESOLUTION = 0
};

enum
{
  TAG_CF = 128,
  TAG_FF = 129,
  TAG_FC = 130,
  TAG_SF = 131
};

static void InternalProcessPkt (ISO15765_Channel *chan, TCANPacket *pkt);
static uint16 SendFC (ISO15765_Channel *chan, uint16 fs, uint16 bs, uint16 st);
static void ReceiveSingleFrame (ISO15765_Channel *chan, TCANPacket *pkt);
static void ReceiveMultiFrame (ISO15765_Channel *chan, TCANPacket *pkt);
static void TransmitFrame (ISO15765_Channel *chan, TCANPacket *pkt);
static uint16 ISO15765_ChTxChunk (ISO15765_Channel *chan);
static uint16 ISO15765_ChRx (ISO15765_Channel *chan, uint8 *pkt, uint16 *length);

#endif
