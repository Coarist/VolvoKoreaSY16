#include "can.h"
#include "common.h"
#include "iso15765.h"
#include "iso15765_internal.h"
#include <string.h>

/////////////////////////////////////////
// Provides ISO15765 service
/////////////////////////////////////////

// TEMP
#pragma diag_suppress=pe826
// TEMP

// Unused parts of iso transmission always sent as zeros (defined) or random crap (undefined)
#define NULLIFY_UNUSED_DATA

// Helper macros
#define ISO15765_CreateTagFromChanPtr(CHAN, TAG) ((CHAN->chid << 8) | (TAG))

static uint16 UUDT_Tx (uint16 id, uint16 length, uint8 *data, uint16 tag)
{
  uint16 res = 0;

  if ((id > 0) && (length > 0) && (data) && (length < 9))
  {
    TCANPacket pkt;
    uint16 lp = 0;

    pkt.cplen = sizeof(TCANPacket);
    pkt.dlc = 8;
    pkt.id = id;
    pkt.tag = tag;
#ifdef NULLIFY_UNUSED_DATA
    pkt.data[1] = 0;
    pkt.data[2] = 0;
    pkt.data[3] = 0;
    pkt.data[4] = 0;
    pkt.data[5] = 0;
    pkt.data[6] = 0;
    pkt.data[7] = 0;
#endif
    while (length)
    {
      length --;
      pkt.data[lp] = data[lp];
      lp++;
    }

    if (CANTx(&pkt) == CANERR_TX_OK)
    {
      res = 1;
    }
  }

  return res;
}


static uint16 ISO15765_ChTxChunk (ISO15765_Channel *chan)
{
  uint16 res = 0;

  //  DEBUGUARTSTR("<iso-chunk>");
  if (chan)
  {
    uint8 outpkt[8];

    if ( (chan->next_seq == 0) && (chan->buffer_pos == 0) )

    {
      // Do first frame (FF)

      outpkt[0] = (0x10 | ((chan->pkt_length >> 8) & 0xF));     // This is the FF frame, upper 4 bits of length is in lower nibble
      outpkt[1] = (chan->pkt_length & 0xff);                    // rest of length is in the second byte
      memcpy (&outpkt[2], chan->buffer, 6);                     // which leaves 6 bytes for the start of the packet
      res = UUDT_Tx(chan->xmitid, 8, outpkt, ISO15765_CreateTagFromChanPtr(chan, TAG_FF));
      chan->pkttimer = N_Bs - TIMER_RESOLUTION;
      chan->buffer_pos += 6;
      chan->next_seq ++;
    }
    else
    {
      // Do consecutive frame (CF)
      uint16 size = chan->pkt_length - chan->buffer_pos;

      if (size > 7)
        size = 7;

      if ((size + chan->buffer_pos) < chan->buffer_length)                        // Ensure we don't overflow the buffer!
      {
        outpkt[0] = (0x20 | (chan->next_seq));
        memcpy (&outpkt[1], &chan->buffer[chan->buffer_pos], size);
        res = UUDT_Tx(chan->xmitid, size + 1, outpkt, ISO15765_CreateTagFromChanPtr(chan, TAG_CF));
        chan->buffer_pos += size;
        chan->next_seq ++;
        chan->pkttimer = N_Bs - TIMER_RESOLUTION;
        if (chan->next_seq > 0xF)
        {
          chan->next_seq = 0;
        }
        // If we have completed sending everything, switch status to 'awaiting_ak'
        if (chan->buffer_pos >= chan->pkt_length)
        {
          chan->tbs = 0;                // Send no more packets
          chan->gstate = ISO15765_GSTATE_IDLE;
          chan->pkttimer = 0x7FFF;
        }
      }
    }
  }

  return res;
}

// TODO: What do we do if the final packet of an ISO15765 transmission doesn't get received? We should resend the entire packet a limited
// number of times before giving up and dropping the connection.

/// Attempt to retrieve a ISO15765 packet from connection 'connid'. All appropriate headings/footing are stripped, and segmented
/// packets are concatenated together. Packet is received as per ISO 15765-2 and RDS V1.3.
/// \param pkt Where to store the packet (can be null if caller is not interested in the actual data)
/// \param length Where to store the length of the packet. On entry, this should contain the length of the buffer (pkt).
/// \return 0 on failure, 1 on success.
/// \note
/// If the length of the buffer specified by 'length' is smaller than the data size, failure will result. This can easily be
/// checked, as the new length (written by this function into the variable) will be larger than the original length. This
/// allows the calling function to enlarge it's buffer to an appropriate size for a successfull receive. \n\n
///
/// Identifier is not returned, as the caller of this function should know where the packet is coming from anyway via the
/// connection id (connid) \n\n
///
/// Calling this function when no new data is available will cause it to repeat the last data until such time that it is
/// overwritten by a new packet - failure is not returned in this case.
static uint16 ISO15765_ChRx (ISO15765_Channel *chan, uint8 *pkt, uint16 *length)
{
  uint16 res = 0;

  if (length)
  {
    // We only function on a properly setup channel...
    if (chan->tstate != ISO15765_TSTATE_INVALID)
    {
      // copy as many bytes as possible into the buffer provided.
      if ((chan->buffer) && (pkt))
        memcpy (pkt, chan->buffer, *length > chan->pkt_length ? chan->pkt_length : *length);
      res = 1;
      chan->completed = 0;
      // Report actual length
      *length = chan->pkt_length;
      // Zeroise packet length so we need a SF or FF to start again.
      chan->pkt_length = 0;
    }
  }

  return res;
}

static void ReceiveSingleFrame (ISO15765_Channel *chan, TCANPacket *pkt)
{
  // Single frame. If the data length is zero, or the data length is greater than 7, then the packet will be ignored.
  // The DLC of the CAN packet must be greater than the length specified in the PCI, otherwise the packet will be ignored.
  // If the packet has already been used and has unread data in it, don't overwrite the data, but ignore. It'll be resent later.
  // (Ignore = No ACK will be emitted, and the packet will not be passed to the higher layers)
  if ((pkt) && (chan->tstate != ISO15765_TSTATE_INVALID))
  {
    uint16 len = pkt->data[0] & 0x0F;

    if ((!chan->completed) && (len > 0) && (len < 8) && (pkt->dlc > len) && (chan->buffer) && (chan->buffer_length >= len))
    {
      // Move the packet into our own buffers
      memcpy (chan->buffer, &pkt->data[1], len);
      chan->buffer_pos = len;
      chan->pkt_length = len;
      chan->completed = 1;
    }
    else if ((chan->buffer == NULL) && (chan->buffer_length))
    {
      // don't store the data, but do store the length and the completion flag. 
      chan->buffer_pos = len;
      chan->pkt_length = len;
      chan->completed = 1;      
    }
  }
}

static void ReceiveMultiFrame (ISO15765_Channel *chan, TCANPacket *pkt)
{
  if ((pkt) && (chan->tstate != ISO15765_TSTATE_INVALID) && ((chan->flags & ISO15765F_INVALIDPKT) == 0))
  {
    uint16 PCI = pkt->data[0];

    switch (PCI & 0xF0)
    {
    case PCI_FF:
      // First frame of a multi-segmented packet
      // If we receive another FF in the middle of receiving another packet, we abort the current reception and start again.
      if (!chan->completed)
      {
        uint16 length;

        // FF packets mean data more than 7 bytes wants to come in, so the CAN DLC must be 8.
        if (pkt->dlc == 8)
        {
          length = (((PCI & 0x0F) << 8) | pkt->data[1]);

          // Check the internal length. We must be able to process packets larger than our buffer (upto 4095 bytes)
          if (length > 7)
          {
            // Setup the channel for receiving a multi-segmented packet
            chan->buffer_pos = 6; // Next packet will start being received here.
            chan->gstate = ISO15765_GSTATE_AWAIT_CF;
            chan->pkttimer = TL_A - TIMER_RESOLUTION;
            chan->next_seq = 1;
            chan->pkt_length = length;  // Actual length of packet
            chan->flags |= ISO15765F_RETRY_DELAY; // Don't add 100ms retry delay to our timer

            // store the data from this packet, but don't mark the packet as received until we have got all the CF's
            if (chan->buffer)
              memcpy (chan->buffer, &pkt->data[2], 6);

            // acknowledge reception of the packet, and request the next 1
            SendFC (chan, FCFS_CTS, 1, 0);
          }
          // Otherwise length is 7 or less, and will be ignored, as it should have been sent using a single frame.
        }
      }
      break;

    case PCI_CF:
      // Consecutive frame of a multi-segmented packet
      if (!chan->completed)
      {
        uint16 seqnum = (PCI & 0x0F);

        // Ensure we were expecting this - ignore if we were not.
        if (chan->gstate == ISO15765_GSTATE_AWAIT_CF)
        {
          // And that the sequence number is correct
          if (chan->next_seq == seqnum)
          {
            uint8 plen = pkt->dlc - 1;
            // check for null packet (PCI and nothing else)
            if (plen)
            {
              // and just make a quick check to ensure we are not going to overrun our buffer
              // if a buffer overflow is attempted, we still acknowledge the packet (Renault requirement), but
              // actually ignore all the data in it :)
              if ((chan->buffer) && ((chan->buffer_pos + plen) < chan->buffer_length))
              {
                memcpy (&chan->buffer[chan->buffer_pos], &pkt->data[1], plen);
              }
              // We have to increment the buffer position regardless of whether we would overflow the buffer or not
              // as it's the only way we know whether or not we have received all the bytes
              chan->buffer_pos += plen;
              chan->next_seq ++;
              if (chan->next_seq > 0xF)
              {
                chan->next_seq = 0;
              }
            }
            // if buffer pos is now greater than the original packet length specified in the FF packet, then we have a
            // complete packet, otherwise we need to ask for more.
            if (chan->buffer_pos >= chan->pkt_length)
            {
              // Packet complete, send ACK.
              if (chan->buffer_length)     // If buffer length is zero, caller isn't interested in knowing when packets are received.
                chan->completed = 1;
              chan->gstate = ISO15765_GSTATE_IDLE;
            }
            else
            {
              // Packet still incomplete, send FC.
              chan->pkttimer = TL_A - TIMER_RESOLUTION;
              chan->flags |= ISO15765F_RETRY_DELAY; // Don't add 100ms retry delay to our timer
              SendFC (chan, FCFS_CTS, 1, 0);
            }
          }
          else // Sequence number is incorrect.
          {
            chan->flags |= (ISO15765F_INVALIDPKT | ISO15765F_MINOR_ERROR | ISO15765F_RETRY_DELAY);
            chan->pkttimer = 1;
          }
        }
      }
      break;

    default:
      break;
    }
  } // if (pkt)
}

static void TransmitFrame (ISO15765_Channel *chan, TCANPacket *pkt)
{
  if ((pkt) && (chan->tstate != ISO15765_TSTATE_INVALID) &&
      ((chan->flags & ISO15765F_INVALIDPKT) == 0))
  {
    uint16 PCI = pkt->data[0];

    switch (PCI & 0xF0)
    {
    case PCI_FC:
      // Channel state must be in AWAIT_FC state, and the length must be at least 3 bytes. (Otherwise it is ignored)
      // Also, if there are packets left to send from a previous FC, ignore the FC.
      if ((chan->gstate == ISO15765_GSTATE_AWAIT_FC) && (pkt->dlc >= 3) && (chan->tbs == 0))
      {
        uint16 fs,bs,st;

        fs = pkt->data[0] & 0xF;        // Flow status
        bs = pkt->data[1];              // Block size
        st = pkt->data[2];              // Separation time
        switch (fs)
        {
        case FCFS_CTS:
          // Reset our FC timer
          chan->pkttimer = N_Bs - TIMER_RESOLUTION;

          // Only take the info from the first FC received per segmented packet
          if ((chan->flags & ISO15765F_RECEIVED_FCCTS) == 0)
          {
            chan->flags |= ISO15765F_RECEIVED_FCCTS;

            // Continue to send 'bs' number of packets of 'sp' interval. Don't actually send the packet here - wait for
            // the st period to expire.
            if (bs > 0)
            {
              // Send this many packets before waiting for next FC
              chan->tbs = bs;
            }
            else
            {
              // A 'bs' of zero means send all packets
              chan->tbs = 0xFFFF;
            }
            chan->fp_bs = chan->tbs;

            // Separation time
            if (st < 0x80)
            {
              chan->fp_st = st;
            }
            else if (st < 0xF1)
            {
              // Invalid reserved range, use the largest value specified by ISO 15765.
              chan->fp_st = 127;
            }
            else if (st < 0xFA)
            {
              // 100us - 900us minimum time gap. Lets round it up to 1ms.
              chan->fp_st = 1;
            }
            else
            {
              // Invalid reserved range, use the largest value specified by ISO 15765.
              chan->fp_st = 127;
            }
            chan->tstmin = chan->fp_st;

            // Valid FC, send out a CF ASAP.
            chan->tsttimer = 1;
          }
          else
          {
            // Not the first FC frame, so use the values from last time, regardless of the ones in the frame
            chan->tbs = chan->fp_bs;
            chan->tstmin = chan->fp_st;

            // Valid FC, send out a CF ASAP.
            chan->tsttimer = 1;
          }
          break;
        case FCFS_WAIT:
        case FCFS_OVERFLOW:
        default:
          // Invalid packet, mark as such and retry
          chan->flags |= (ISO15765F_INVALIDPKT | ISO15765F_RETRY_DELAY);
          chan->pkttimer = TL_B - TIMER_RESOLUTION;
          break;
        }
      }
      break;

    default:
      break;
    }
  }
}

/// Process an incoming CAN packet as an ISO15765 packet. Performs content stripping, extraction, prepares for acknowledgement, etc.
static void InternalProcessPkt (ISO15765_Channel *chan, TCANPacket *pkt)
{
  if ((pkt) && (chan->tstate != ISO15765_TSTATE_INVALID))
  {
    uint8 PCI;

    if (pkt->dlc)       // At least 1 byte is required
    {
      PCI = pkt->data[0];
      // If channel is in CONNOK state, accept all packets apart from CS
      // If channel is in CONNREQ state, only accept AK for transmit channels and CS for receive channels
      // Otherwise ignore the packet. Unknown packets are also ignored.
      if (chan->tstate == ISO15765_TSTATE_CONNOK)
      {
        // We can handle anything apart from CS frames

        switch (PCI & 0xF0)
        {
          // Receiving
        case PCI_SF:
          if (chan->dir != ISODIR_TX)
            ReceiveSingleFrame(chan, pkt);
          break;
        case PCI_FF:
        case PCI_CF:
          if (chan->dir != ISODIR_TX)
            ReceiveMultiFrame(chan, pkt);
          break;
          // Transmitting packets
        case PCI_FC:
          if (chan->dir != ISODIR_RX)
            TransmitFrame (chan, pkt);
          break;
        default:
          break;
        }
      }
    } // if (pkt)
  }
  return;
}

/// Send a flow control packet over the CAN
/// \param fs FCFS_CTS (Continue To Send), FCFS_WAIT (Do not send any more packets yet), FCFS_OVERFLOW (Buffer overflow, abort send)
/// \param bs Block size - the absolute number of CF N_PDU's per block
/// \param st Separation time gap between packets - see Table 15 of ISO 15765
static uint16 SendFC (ISO15765_Channel *chan, uint16 fs, uint16 bs, uint16 st)
{
  uint8 pkt[3];
  uint16 res = 0;

  if (fs < 3)
  {
    pkt[0] = (0x30 | fs);
    pkt[1] = bs;
    pkt[2] = st;
    res = UUDT_Tx(chan->xmitid, 3, pkt, ISO15765_CreateTagFromChanPtr (chan, TAG_FC));
  }
  return res;
}

/*****************************************************************************************************/
/* Public functions                                                                                  */
/*****************************************************************************************************/

// Magic features: if 'buffer' is null, packets will be received as normal, but not stored. 
// If 'buffer' is null, but 'buffer_length' is > 0, caller will be told when a packet has been received
// (and will have to call ISO15765_Rx to clear the flag), but will obviously receive no data other than id and length.
// If 'buffer_length' is zero and 'buffer' is null, packets will be received as normal, but not stored, and the
// caller will NOT be notified of any incoming packets. 

uint16 ISO15765_Connect (ISO15765_Channel *chan, uint16 chid, uint16 xmitid, uint16 rcvid, 
                          uint8 *buffer, uint16 buffer_length, ISO15765_Dir dir)
{
  if ((xmitid) && (rcvid))                                                                      // Both ID's are valid?
  {
    memset(chan,0,sizeof(ISO15765_Channel) );
    chan->xmitid = xmitid;
    chan->rcvid = rcvid;
    chan->chid = chid;
    chan->buffer = buffer;
    chan->buffer_length = buffer_length;
    chan->dir = dir;
    chan->tstate = ISO15765_TSTATE_CONNOK;
    chan->gstate = ISO15765_GSTATE_IDLE;
    chan->pkttimer = 0x7FFF;
    return 0;
  }
  return (uint16)-1;
}

/// Queue a transmit request for 'pkt' of length 'length'. All appropriate headings/footings etc are handled here, as well as
/// splitting the packet up into segments and handling the flow control. Packet is transmitted as per ISO 15765-2 and RDS V1.3.
/// \param connid Connection ID returned from ISO15765_Connect()
/// \param pkt The packet to be transmitted
/// \param length The length of the packet
/// \return 0 on failure, 1 on success
uint16 ISO15765_ChTx (ISO15765_Channel *chan, uint8 *pkt, uint16 length)
{
  uint16 res = 0;

  if (chan->dir == ISODIR_RX)
    return 0;

  // Valid 'connid' ?
  if (chan->tstate != ISO15765_TSTATE_INVALID)
  {
    // Is this really a transmit channel? And if so, one that is not in use?
    if (chan->gstate == ISO15765_GSTATE_IDLE)
    {
      uint8 outpkt[8];
      // How big is the packet? For 7 or less we can use a SF-style packet, but for anything bigger we need the FF/CF combo
      // with flow control.
      if (length < 8)
      {
        chan->gstate = ISO15765_GSTATE_IDLE;
        chan->pkttimer = 0x7FFF;
        chan->retries = 0;
        chan->buffer_pos = length;
        chan->pkt_length = length;
        chan->flags = 0;
        chan->tbs = 0;

        outpkt[0] = length; // PCI = 0, SingleFrame
        memcpy (chan->buffer, pkt, length); // Store a copy just in case we need to retry
        memcpy (&outpkt[1], pkt, length); // And also send a copy out
        res = UUDT_Tx(chan->xmitid, length+1, outpkt, ISO15765_CreateTagFromChanPtr (chan, TAG_SF));
      }
      else
      {
        // 8 or more bytes. Ensure it's less than our buffer.
        if (length <= chan->buffer_length)
        {
          memcpy (chan->buffer, pkt, length);                           // Copy to our buffer
          chan->buffer_pos = 0;                                         // Will start transmitting at this position in the buffer
          chan->gstate = ISO15765_GSTATE_AWAIT_FC;                      // We expect a flow control frame after this packet has been sent
          chan->retries = 0;
          chan->flags = 0;
          chan->next_seq = 0;                                                                   // The sequence number we shall use next will be '1'
          chan->pkt_length = length;                                                            // Total packet length
          chan->tbs = 0;

          res = ISO15765_ChTxChunk(chan);
        }
      }
    }
  }
  return res;
}

/// Return the first connection id that has data in it ready for retrieval.
/// \param id Where to store the received identifier (must not be null)
/// \param pkt Where to store the packet (can be null if not interested in the packet data)
/// \param length Where to store the length of the packet. On entry, this should contain the length of the buffer (pkt).
/// \return 0 on failure, 1 on success.
/// \note
/// See ISO15765_ChRx() for further details, as that function is called internally using the supplied parameters.
uint16 ISO15765_Rx (ISO15765_Channel *chan, uint16 *id, uint8 *pkt, uint16 *length)
{
  uint16 res = 0;

  if ((chan->completed) && (chan->dir != ISODIR_TX))
  {
    // Make sure the parameters are actually valid
    if ((id) && (length))
    {
      // We know there's at least one packet waiting for reception - lets go and find it.
      *id = chan->rcvid;
      res = ISO15765_ChRx(chan, pkt, length);
    }
  }
  return res;
}

/// Get the status of the connection 'connid'. Should be used after a ISO15765_Connect() to ensure the connection has been setup correctly.
/// \param connid Connection ID returned from ISO15765_Connect()
/// \return Connection status (GSTATE in upper 8-bits, TSTATE in lower 8-bits)
uint16 ISO15765_Status (ISO15765_Channel *chan)
{
  uint16 res;

  if (chan->tstate != ISO15765_TSTATE_INVALID)
  {
    res = (chan->gstate << 8);
    res |= (chan->tstate);
  }
  else
  {
    res = ((ISO15765_GSTATE_INVALID) << 8);
    res |= (ISO15765_TSTATE_INVALID);
  }

  return res;
}

/// Runs the ISO15765 main loop. Timed events are done here. The bulk of sending a receiving is done on an event-driven basis.
/// (See ISO15765_ProcessPkt())
/// \return ISO15765 status: eg. ISO15765S_PACKET_WAITING, ISO15765S_TX_OK, ...
uint16 ISO15765_RunCycle (ISO15765_Channel *chan)
{
  uint16 res = ISO15765S_IDLE;

  // If there is a packet waiting, default to that, otherwise change to error code.
  if (chan->completed)
  {
    res = ISO15765S_PACKET_WAITING;
  }

  // We need to check if any of the channels timers have expired, which includes CF's, AK's, etc, along with scheduling
  // CF's to be sent.

  // Check if channel needs it's timers checking

  if (chan->tstate != ISO15765_TSTATE_INVALID)
  {
    if (((chan->gstate & 0xF0) == ISO15765_GSTATE_AWAITING) && (chan->pkttimer != 0x7FFF))
    {
      chan->pkttimer --;
      if (chan->pkttimer <= 0)
      {
        // Timer triggered. Find out what we were waiting for and retry if possible.
        if ((chan->flags & ISO15765F_RETRY_DELAY) == 0)
        {
          // Not yet done the retry delay, mark the packet as invalid just in case someone tries to talk about the packet we
          // have just canceled.
          chan->flags |= (ISO15765F_RETRY_DELAY | ISO15765F_INVALIDPKT);
          chan->pkttimer = TL_B - TIMER_RESOLUTION;
        }
        else // ISO15765F_RETRY_DELAY is set
        {
          switch (chan->gstate)
          {
          case ISO15765_GSTATE_AWAIT_CF:
            // We were expecting a consecutive frame from the display, but didn't get it. Abort the packet receive.
            chan->pkttimer = 0;
            chan->flags = 0;
            chan->gstate = ISO15765_GSTATE_IDLE;
            break;
          case ISO15765_GSTATE_AWAIT_FC:
            chan->retries ++;
            if (chan->retries < MAXIMUM_ISO15765_RETRIES)
            {
              chan->flags = 0;
              // Quick sanity check
              if (chan->pkt_length >= 8)
              {
                chan->buffer_pos = 0;                                                           // Resend from the beginning
                chan->gstate = ISO15765_GSTATE_AWAIT_FC;                                        // We expect a flow control frame after this packet has been sent
                chan->next_seq = 0;                                                             // The sequence number we shall use next will be '1'
                ISO15765_ChTxChunk(chan);
              }
              else
              {
                chan->gstate = ISO15765_GSTATE_IDLE;
              }
            }
            else // chan->retries is >= MAXIMUM_ISO15765_RETRIES
            {
              chan->gstate = ISO15765_GSTATE_IDLE;                                      // Allow channel to be reused (no longer in use)
              if (chan->flags & ISO15765F_MINOR_ERROR)
              {
                res = ISO15765S_TX_ERROR_MINOR;
              }
              else
              {
                res = ISO15765S_TX_ERROR_MAJOR;                                         // Fall into degraded mode and attempt to reconnect
                                                                                        //DEBUGUARTSTR("<MajISO15765errFC>");
              }
              chan->flags = 0;
            }
            break;
          default:
            // Don't know what brought us here, must be bogus.
            chan->pkttimer = 0;
            // We must clear the gstate now the timer is no more, otherwise we will constantly come back in here
            chan->gstate = ISO15765_GSTATE_IDLE;
            break;
          } // switch
        }   // ISO15765F_RETRY_DELAY if
      }
      else
      {
        // Packet timer hasn't expired, See if this channel is transmitting a large packet
        if ((chan->gstate == ISO15765_GSTATE_AWAIT_FC) && ((chan->flags & (ISO15765F_INVALIDPKT | ISO15765F_WAITING_TXOK)) == 0))
        {
          // see if we need to send another portion of it (check block size & timer)
          if (chan->tbs)
          {
            chan->tsttimer --;
            if (chan->tsttimer <= 0)
            {
              chan->tsttimer = chan->tstmin;
              chan->tbs --;
              // Comms looks ok - reset our retry counter.
              ISO15765_ChTxChunk(chan);
              // Don't start decrementing the timer for the next packet until this one has successfully been transmitted
              if (chan->tbs)
              {
                // We need to autosend the next CF, wait until this has cleared first.
                chan->flags |= ISO15765F_WAITING_TXOK;
              }
            }
          }
        }
      }
    }
  }

  return res;
}

/// Accept a packet, and see if it's one that we are interested in.
/// \return 0 on failure (packet was invalid, or wasn't for us), 1 on success (we have taken the packet and used it)
uint16 ISO15765_ProcessPkt (ISO15765_Channel *chan, TCANPacket *pkt)
{
  uint16 res = 0;

  if (pkt)
  {
    if (pkt->cplen == sizeof(TCANPacket))
    {
      uint16 id = pkt->id;
      if ((chan->tstate != ISO15765_TSTATE_INVALID) && (chan->rcvid == id))
      {
        InternalProcessPkt (chan, pkt);
        res = 1;
      }
    }
  }

  return res;
}

/// Return the possibility of a new packet.
uint16 ISO15765_IsPacketWaiting (ISO15765_Channel *chan)
{
  return ((chan->completed) && (chan->dir != ISODIR_TX));
}

void ISO15765_ReportFailure (ISO15765_Channel *chan, uint16 tag)
{
}

void ISO15765_ReportSuccess (ISO15765_Channel *chan, uint16 tag)
{
  // Remove ISO15765F_WAITING_TXOK flag from channel if needed
  uint8 tagtype = tag & 0xFF;

  if (tagtype == TAG_CF)
  {
    if (chan->tstate != ISO15765_TSTATE_INVALID)
    {
      chan->flags &= ~ISO15765F_WAITING_TXOK;
    }
  }
}
