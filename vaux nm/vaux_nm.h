#ifndef VAUX_NM_INCLUDED
#define VAUX_NM_INCLUDED

#include "can.h"

// Internal use only
typedef enum {NMI_SLEEPING, NMI_GOINGTOSLEEP, NMI_READYFORSLEEP, NMI_RUNNING, NMI_STARTUP} NMInternalState;
// NME_SLEEPING - Not participating on the bus, but monitoring bus traffic
// NME_AWAKE - Not sleeping, but might not have sent or received any packets yet
// NME_ACTIVE - Fully participating on the bus. Set when in single node mode, or another node talks to us.
typedef enum {NME_SLEEPING = 0, NME_AWAKE = 1, NME_ACTIVE = 2 } NMExternalState;
// NMC_SLEEP - Go off bus, can take upto 10 secs before off-bus status finally happens
// NMC_WAKE - Wake up
typedef enum {NMC_SLEEP, NMC_WAKE, NMC_FORCEWAKE} NMCommand;

// Call this every 1ms
void vaux_nm_1ms (void);
// Call this with every can packet received. Returns 1 if can packet eaten.
int vaux_nm_can (TCANPacket *msg);
// To control on/off bus status.
void vaux_nm_cmd (NMCommand cmd);
// Get internal status
NMExternalState vaux_nm_status (void);
// Check if node is available on network. 
// Returns 1 if node was marked as active in the last 200ms. (Netlist only, so the actual node might not have been seen on the network for upto 3.2s)
u8 vaux_node_avail (u8 node);
// use this to set the extra 4 data bytes attached to the end of every NM packet sent out
// create the 4 bytes of data in a 4 byte array of u8 then pass as a pointer
void SetNMData( u8 * data_array );

#endif
