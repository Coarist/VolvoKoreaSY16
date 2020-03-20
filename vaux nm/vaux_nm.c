#include "vaux_nm.h"
#include <string.h>
#include "diags.h"

// HISTORY

// 26-01-09
// added ability to set the extra 4 data bytes sent out with every NM frame

#define OURADDR 1

// When we are sleeping, we still sniff the bus and grab the netlist. It expires after we don't see any NM
// for this amount of time. This allows for speedier wakeup, as we already know who to talk to.
#define RXNETHOLDTIME 200

// Delay between being told to go sleep and actually doing it. Delay for display and radio is 10 seconds,
// so we copy that here.
#define SLEEPTIME 10000

// NM Status packet delays. Singlenode = Only us on the network. Multinode = at least one other node talking to us.
// (Multinode should never expire if the network is running properly, if it does, we lost our succ)
#define TXCHECKTIME_SINGLENODE 100
#define TXCHECKTIME_NORMAL 100
#define TXCHECKTIME_MULTINODE 150

// By default we talk 100ms after we have been addressed. However, if we have been skipped, then we talk ASAP
#define TXSTATUSDELAY_NORMAL 100
#define TXSTATUSDELAY_NOW 1

// If no network traffic for this period with an active network list, we reinit the network.
#define DEADNETWORKDELAY 1700

static NMInternalState CurrentState = NMI_SLEEPING, WantedState = NMI_SLEEPING;
static NMExternalState ExtCurrentState = NME_SLEEPING;
static u16 NetList = 1 << OURADDR;        // By default, there's only us on the network
static u16 FaultyNetList = 0;             // List of faulty nodes on the net (Inverted, 0 = Faulty)
static const u8 OurAddr = OURADDR;
static u8 Succ = OURADDR;                 // and our succ will be ourselves until we find out differently
static u8 NextSucc = 0;

static u16 SleepTimer = 0;                // Used for transition from "ready to sleep" to "sleep" and off-bus
static u16 TXCheckTimer = 0;              // Ensures our packets are replied to
static u16 TXStatusDelay = 0;             // How long to wait before sending out our status
static u16 RXNetHoldDelay = 0;            // We still sniff bus traffic when sleeping - this is how long we hold onto the info for.
static u16 DeadNetwork = 0;

static u8 CurrentStatusByte = 0x02;       // Our node status
static u8 NMData[4] = {0,0,0,0};

static u8 FindSucc (u8 Addr);
static void CalcNetList (u8 *NewList);
static u8 vaux_nm_canwake (TCANPacket *msg);
static bool ForceNetworkWake = false;

// Local functions //

// Find succ to addr in global NetList
static u8 FindSucc (u8 Addr)
{
  // We will always be in the list ourselves, so no need for any getout clauses.
  for (;;)
  {
    Addr ++;
    Addr &= 15;
    if (NetList & (1 << Addr))
      return Addr;
  }
}

static void CalcNetList (u8 *NewList)
{
  // Rebuild netlist
  NetList = NewList[0] << 8;
  NetList |= NewList[1];
  NetList |= ( 1 << OurAddr); // Ensure we are in the list ourself.

  // Calculate new successor (previously, we would calculate the predecessor too for the purpose of checking whether or not we
  // have been missed out of the loop. It can cause problems however if our pred is intermittant, so we just check succ now)
  Succ = FindSucc (OurAddr);
}

// Global functions //

void vaux_nm_1ms (void)
{
  if (DeadNetwork)
  {
    DeadNetwork --;
    if (!DeadNetwork)
    {
      // No nodes talking at all? Attempt to restart the network.
      Succ = OurAddr;
      NextSucc = Succ;
      CurrentState = NMI_STARTUP;
      CurrentStatusByte = 0x22;
      NetList |= ( 1 << OurAddr );
      FaultyNetList = NetList;
      DeadNetwork = DEADNETWORKDELAY;
      TXCheckTimer = 0;
      TXStatusDelay = TXSTATUSDELAY_NOW;
    }
  }

  if (RXNetHoldDelay)
  {
    RXNetHoldDelay --;
    if (!RXNetHoldDelay)
    {
      if (CurrentState == NMI_SLEEPING)
      {
        Succ = OurAddr;
        NetList = 1 << OurAddr;
      }
    }
  }

  if (SleepTimer)
  {
    SleepTimer --;
    if (!SleepTimer)
    {
      if (CurrentState == NMI_READYFORSLEEP)
      {
        CurrentState = NMI_GOINGTOSLEEP;
        CurrentStatusByte |= 8;
      }
    }
  }

  if (TXCheckTimer)
  {
    TXCheckTimer --;
    if (!TXCheckTimer)
    {
      if ((Succ == OurAddr) && (CurrentState != NMI_SLEEPING))
      {
        // We are talking to ourselves
        TXStatusDelay = TXSTATUSDELAY_NOW;
        ExtCurrentState = NME_ACTIVE;
      }
      else
      {
        u16 SuccBit = 1 << NextSucc;
        // We talked to someone, but they didn't.
        // Skip them for now, but give them another chance next time around the ring
        NextSucc = FindSucc (NextSucc);
        if (FaultyNetList & SuccBit)  // Marked as a known good node?
          FaultyNetList &= ~SuccBit;  // Then mark as an intermittant node
        else
        {
          NetList &= ~SuccBit;        // Two attempts, no response, your outa here
          Succ = NextSucc;            // and we have a new succ.
        }
        CurrentStatusByte |= 0x20;
        TXStatusDelay = TXSTATUSDELAY_NOW;
      }
    }
  }

  if (CurrentState != WantedState)
  {
    switch (WantedState)
    {
      case NMI_READYFORSLEEP:
        if ((CurrentState == NMI_RUNNING) || (CurrentState == NMI_STARTUP))
        {
          CurrentStatusByte &= ~2;
          CurrentStatusByte |= 0x10;
          SleepTimer = SLEEPTIME;
          CurrentState = NMI_READYFORSLEEP;
        }
        break;
      case NMI_STARTUP:
        if (CurrentState == NMI_SLEEPING)
        {
          // If we are talking to ourselves, then talk now, else wait for our position in the network.
          if (Succ == OurAddr)
          {
            TXCheckTimer = TXCHECKTIME_SINGLENODE;
            TXStatusDelay = TXSTATUSDELAY_NOW;
          }
          CurrentStatusByte = 0x02;
          if (ForceNetworkWake)
          {
            ForceNetworkWake = false;
            CurrentStatusByte = 0x01;
          } 
          CurrentState = NMI_STARTUP;
          ExtCurrentState = NME_AWAKE;
          NetList |= ( 1 << OurAddr );
          FaultyNetList = NetList;
          NextSucc = Succ;
          DeadNetwork = DEADNETWORKDELAY;
        }
        else if (CurrentState == NMI_READYFORSLEEP)
        {
          // No need to setup any timers, as we are already talking.
          CurrentStatusByte = 0x02;
          CurrentState = NMI_STARTUP;
          ExtCurrentState = NME_AWAKE;
        }
        break;
    }
  }

  // Do we need to send a status packet out?
  if (TXStatusDelay)
  {
    TXStatusDelay --;
    if (!TXStatusDelay)
    {
      TCANPacket statuspkt;

      if (CurrentState == NMI_GOINGTOSLEEP)
      {
        // Remove ourselves from the network
        DeadNetwork = 0; // Don't try to recover from this. We don't care if the network dies from now on.
        CurrentState = NMI_SLEEPING;
        ExtCurrentState = NME_SLEEPING;
        NetList &= ~(1 << OurAddr);
      }
      else
      {
        TXCheckTimer = ((CurrentStatusByte & 0x20 ? NextSucc : Succ) != OurAddr ? TXCHECKTIME_MULTINODE : TXCHECKTIME_SINGLENODE);
        DeadNetwork = DEADNETWORKDELAY;
      }

      statuspkt.cplen = sizeof(TCANPacket);
      statuspkt.id = 0x500 + OurAddr;
      statuspkt.tag = (u16)-1;
      statuspkt.dlc = 8;
      statuspkt.data[0] = ((CurrentStatusByte & 0x20 ? NextSucc : Succ) << 4) | OurAddr;
      statuspkt.data[1] = NetList >> 8;
      statuspkt.data[2] = NetList & 0xFF;
      statuspkt.data[3] = CurrentStatusByte;
      statuspkt.data[4] = NMData[0];
      statuspkt.data[5] = NMData[1];
      statuspkt.data[6] = NMData[2];
      statuspkt.data[7] = NMData[3];
      CANTx(&statuspkt);
      CurrentStatusByte &= (~0x20);
      if ( CurrentStatusByte & 0x01 )
      {
        CurrentStatusByte &= ~0x01;
        CurrentStatusByte |= 0x02;
      }
    }
  }
}

int vaux_nm_can (TCANPacket *msg)
{
  if ((msg->id & 0x7F0) == 0x500)
  {
    u8 msgsucc = (msg->data[0] >> 4);
    u8 msgpred = (msg->data[0] & 0xF);

    CalcNetList(&msg->data[1]);
    RXNetHoldDelay = RXNETHOLDTIME;
    if ((CurrentState == NMI_SLEEPING) && (msg->dlc >= 4) && vaux_nm_canwake(msg))
    {
      CurrentState = NMI_RUNNING;
      WantedState = NMI_READYFORSLEEP;
      // We leave WantedState alone so we can sleep again in SLEEPTIME ms. 
      // Our netlist should have already been rebuilt by this point, so we are sending to the right person and
      // no need to do anything else here apart from setting the CurrentStatusByte. 
      CurrentStatusByte = 0x02; // Not sure whether this should be 0x02 or 0x10. 
      DEBUG("NM Wake\r\n");
    }

    if (msgsucc == OurAddr)
    {
      // We have been addressed.
      if (CurrentState != NMI_SLEEPING)
      {
        ExtCurrentState = NME_ACTIVE;
        DeadNetwork = DEADNETWORKDELAY;
        TXStatusDelay = TXSTATUSDELAY_NORMAL;
        TXCheckTimer = 0; // No longer waiting for anything
        NextSucc = Succ;  // Reset potentially failing succ (but don't reset any possible faults)
        CurrentStatusByte &= (~0x20);
        if (CurrentState == NMI_STARTUP)
        {
          CurrentState = NMI_RUNNING;
          CurrentStatusByte |= 0x10;
        }
      }
    }
    else if (msgsucc == Succ)
    {
      // Someone is talking to our successor, shout up that we should be doing this and not them.
      if (CurrentState != NMI_SLEEPING)
        TXStatusDelay = TXSTATUSDELAY_NOW;
    }

    if (CurrentStatusByte & 0x20)
    {
      // We are skipping someone out, so succ may not actually be our succ at this time (but we need to keep them in the netlist for now)
      if (msgpred == NextSucc)
      {
        TXCheckTimer = 0; // The node was decided to speak to has spoken, everything ok.
        NextSucc = Succ;  // Reset failing succ (but don't reset fault)
      }
    }
    else
    {
      if (msgpred == Succ)
      {
        // Our successor is talking
        FaultyNetList = NetList;
        NextSucc = Succ;
        TXCheckTimer = 0;
      }
    }
    return 1;
  }
  else
    return 0;
}

void vaux_nm_cmd (NMCommand cmd)
{
  switch (cmd)
  {
    case NMC_WAKE:
      WantedState = NMI_STARTUP;
      break;
    case NMC_SLEEP:
      WantedState = NMI_READYFORSLEEP;
      break;
    case NMC_FORCEWAKE:
      WantedState = NMI_STARTUP;
      ForceNetworkWake = true;
      break;
  }
}

NMExternalState vaux_nm_status (void)
{
  return ExtCurrentState;
}

// Check if node is available on network.
// Network must be active before this will work correctly!
// (It is possible that this will return true even in sleeping state, as the network list is kept for a limited time in sleep)
u8 vaux_node_avail (u8 node)
{
  return !!(NetList & (1 << node));
}

void SetNMData( u8 * data_array )
{
  memcpy ( NMData , data_array , 4 );
}

u8 vaux_nm_canwake (TCANPacket *msg)
{
  if ( ( ( msg->data[4] & 0x0f ) == 0x1 ) && ( msg->data[6] & 0x02) )
    return 1;


  return 0;
}
