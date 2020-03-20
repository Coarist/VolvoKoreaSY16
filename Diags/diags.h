#ifndef DIAGS_H
#define DIAGS_H
#include "common.h"



extern void InitDiags(void);
extern void DiagsProcessing(void);
#ifdef DIAGS_ENABLED
#define DEBUG(x)  SendDiag(x)
extern void SendDiag( char * );
#else
#define DEBUG(x) 
#endif

#endif
