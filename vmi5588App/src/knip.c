
/* knip.c */

/* vxWorks #includes */
#include    <vxWorks.h>
#include    <vme.h>
#include    <rebootLib.h>
#include    <iv.h>
#include    <intLib.h>
#include    <taskLib.h>
#include    <sysLib.h>
#include    <symLib.h>
#include    <vxLib.h>
#include    <logLib.h>
#include    <stdlib.h>
#include    <stdio.h>

/* EPICS #includes */
#include    <devLib.h>

#include    <string.h>

#include "vmi5588.h"
#define INT1 1
#define INT2 2
#define INT3 3

#define BLKLEN 256                                /* in unsigned long */
#define SCS 0
#define CEM 1
#define PWFS1 2
#define PWFS2 3
#define ALTAIR 4


SEM_ID updateNow = NULL;
static unsigned long bad_data_count = 0;
/*static unsigned long bad_echo_count = 0;*/

static int reset_node_flag = 0;
static int int1_flag = 0;
static int int2_flag = 0;
static int int3_flag = 0;
static int rmTaskId = 0;

int nodeISR2 = 0;
int nodeISR3 = 0;

int intRate = 200;
int sysClkRateTicks = 200;
int rmQuit = 0;
int echoback = 0;
int blocklen = 256;
int currentNode = CEM;
int intVector = 3;
int p2Sendflag = 0;

unsigned long int2_count = 0;
unsigned long int3_count = 0;

/**IMPORT start**/

/* Board addressing */
#define RM_DEBUG 1

#define RM_SYM_HASHSIZE    6    /* Hash size for symbol table (log2) */
#define RM_INPUTLINESIZE 256    /* Input line buffer length */
#define RM_INPUTTYPESIZE  20    /* type keyword buffer length */
#define RM_INPUTNAMESIZE 128    /* name buffer length */

/* Storage to allocate for each shared rm_data structure */
#define RM_SIZE_ALOG 16
#define RM_SIZE_LONG 12
#define RM_SIZE_STRG 48
#define RM_SIZE_ARRY 16 /* Plus size of array */


enum {
   GOOD = 0, 
   BAD,
   BAD_CANNOT_PROBE,
   BAD_BOARDID 
};

LOCAL void      vmi5588_reboot(int startType);
LOCAL void      rmTalkCEM(void);
LOCAL void      fireInterrupts();


/* Driver variables */
LOCAL VOIDFUNCPTR pisr[4];
int rmMaxAttempts;


LOCAL void fireInterrupts() {
   static int tickCounts = 0;

   if (tickCounts++ >= intRate) {
      tickCounts = 0;
      p2Sendflag = -1;
      semGive(updateNow);
   }
}


/*****************************************************************************
*
* rmStatus - return current RM status information
*
* This routine reads various  status bits from the 5588 card and returns a
* composite of the interesting bits.  The input parameter is a bit mask
* which is used to reset those status bits which are writable -- if rmStatus
* is called with the parameters RM_RESYNC, RM_BADXFER or RM_NORING then the
* relevant status bit or bits will be cleared if they are currently set.  
* To clear all status bits, use a parameter value of 0xffff.  The returned
* status value is not affected by the input parameter.
*
* The RM_NORING bit functions slightly differently to the others. Whenever 
* rmStatus is called with the the bit RM_NORING set the current value of
* the status bit is returned and the bit cleared as normal, but the test
* location in the reflected memory is also written which results in a data
* packet being transmitted around the ring.  Any calls to rmStatus which 
* occur before this packet (or any other packets which originated at this
* node) has been received will return RM_NORING, but the ring transition 
* time should only be a few microseconds at worst.
*
* RETURNS:
* Status from the VMIC5588 card (a value in the range 0 to 0xffff), or 
* S_dev_NoInit.
*
* The following symbols are defined in the header file vmi5588.h and 
* provide a bit mask for the return state.  The bit will be set if the 
* condition described below is true, so zero means Good status.
*
* .CS
*    RM_IRQ1     Interrupt 1 Pending
*    RM_IRQ2     Interrupt 2 Pending
*    RM_IRQ3     Interrupt 3 Pending
*    RM_NOSIG    No Input Signal
*    RM_NOSYNC   Input PLL unsynchronized
*    RM_RESYNC   PLL Recently unsynchronized
*    RM_NORING   Fibre Ring broken
*    RM_BADXFR   Single Transfer Error
*    RM_TXHALF   Transmit FIFO Half-full
*    RM_RXHALF   Receive FIFO Half-full
* .CE
*
* EXAMPLE:
* .CS
*   -> rmStatus 0x40
*   value = 64 = 0x44 = '@'
*   -> rmStatus
*   value = 4 = 0x4
* .CE
*
*/

long rmStatus
(
 long reset      /* Status bits to reset */
 )
{
   long            stat;
   static unsigned char testCounter;

   if (prm == NULL)
      return BAD;

   /*
    * The following combines the two status registers, masks out the bits
    * we're not interested in and makes everything active high, so 0=good
    * status. Note 0x40 = RM_RESYNC is probably good status, but indicates
    * that we lost sync recently, ie someone got turned off & on again.
    */
   stat = ((prm->intRxStatus | prm->boardCsr << 8) &
         (RM_RESYNC | RM_NOSYNC | RM_NOSIG  | RM_IRQ3 | RM_IRQ2 | RM_IRQ1 |
          RM_RXHALF | RM_TXHALF | RM_BADXFR | RM_NORING)) ^
      (RM_TXHALF | RM_RXHALF | RM_NORING);

   /* Clear the R/W status bits */
   if (stat & reset & RM_RESYNC)
      prm->intRxStatus &= ~RM_IRS_LATCHED;
   if (stat & reset & RM_BADXFR)
      prm->boardCsr &= ~RM_CSR_BAD_DATA;
   if (reset & RM_NORING) {
      prm->boardCsr &= ~RM_CSR_OWN_DATA;
      prm->test = (prm->nodeId << 8) + testCounter++;
   };

   return stat;
}



/*****************************************************************************
 *
 * vmi5588_intr - interrupt handler
 *
 * All RM interrupts pass through this routine, which reads the RM source ID
 * and calls the connected routine for this channel.
 *
 * RETURNS: N/A
 *
 * NOMANUAL
 */

void vmi5588_intr
(
 int irqNumber       /* RM interrupt channel number */
 )
{
   if (prm == NULL || irqNumber < 0 || irqNumber > 3) {
      logMsg("%s: Bad RM Interrupt, parameter = 0x%x", 
            (int) __FILE__, irqNumber, 0, 0, 0, 0);
      return;
   };

   if (pisr[irqNumber] != NULL) {
      if (irqNumber > 0)
         (*pisr[irqNumber]) (prm->interrupt[irqNumber].senderId);
      else
         (*pisr[irqNumber]) (0);

      /* finally re-initialise the interrupt hardware */
      prm->interrupt[irqNumber].control = RM_CR_INT_LEVEL6 | 
         RM_CR_INT_ENABLE | 
         RM_CR_INT_AUTOCLR;
   }
   else
      logMsg("%s: RM Interrupt #%d while disconnected", 
            (int) __FILE__, irqNumber, 0, 0, 0, 0);
}

/*****************************************************************************
*
* rmIntConnect - Connect a C routine up to an RM Interrupt
*
* This is used to register an interrupt routine with the RM driver layer,
* and initialises the BIM registers on the 5588 card to enable interrupts
* for the given RM interrupt channel number.
*
* The interrupt routine should expect a single integer parameter, which 
* will contain the node number of the RM interrupt source.
*
* NOTE
* Channel 0 is sent by the 5588 card to indicate transfer errors, or that
* the transmit FIFO is more than half-full. This interrupt is not used by
* the EPICS driver.
* .br
* Channel 1 is used by the device support layer for EPICS RM record types.
* .br
* Channels 2 and 3 are available for general C subroutines.
*
* RETURNS:
* OK, or S_dev_??? errors.
* 
* EXAMPLE:
* .CS
*   void myIsr(int nodeFrom);
*   int status;
*
*   status = rmIntConnect(2, myIsr);
*   if(status != 0)
*       printf("Can't connect RM interrupt: %d\\n", status);
* .CE
*
* SEE ALSO: rmIntDisconnect(), rmIntSend()
*/

long rmIntConnect
(
 int irqNumber,      /* RM interrupt channel */
 VOIDFUNCPTR proutine    /* routine to call on int */
 )
{
   long status;

   if (prm == NULL)
      return BAD;
   if (irqNumber < 0 || irqNumber > 3)
      return BAD;
   if (pisr[irqNumber] != NULL)
      return BAD;

   status = intConnect( (void *)INUM_TO_IVEC(RM_INT_VECTOR + irqNumber),
                           (VOIDFUNCPTR) vmi5588_intr, irqNumber);
   if (status != OK)
      return status;

   /* save the routine pointer */
   pisr[irqNumber] = proutine;

   /* clear out card interrupt FIFOs */
   if (irqNumber > 0)
      prm->interrupt[irqNumber].senderId = 0;

   /* finally enable the hardware */
   prm->interrupt[irqNumber].control = RM_CR_INT_LEVEL6 | 
      RM_CR_INT_ENABLE | 
      RM_CR_INT_AUTOCLR;
   return OK;
}


/********************
 * rmIntSend -- 
 *
 * This causes an interrupt to be sent out on the RM bus, using the given
 * channel number.  The interrupt will be broadcast to all other RM nodes
 * if the node ID given is -1, otherwise it will be sent only to the node
 * specified.
 *
 * You can't send an interrupt to yourself.  The hardware will ignore it.
 *
 * RETURNS:
 * OK, or BAD 
 *
 *
 */
long rmIntSend
(
    int irqNumber,      /* RM interrupt channel */
    int nodeId          /* node number, or -1 for broadcast */
)
{
    if (prm == NULL)
    return BAD;
    if (irqNumber < 1 || irqNumber > 3 || nodeId == prm->nodeId)
    return BAD;

    if (nodeId == -1)
    /* Broadcast to all nodes */
    prm->cmd_Node = irqNumber << 8 | RM_CMD_BROADCAST;
    else
    /* Node specific */
    prm->cmd_Node = irqNumber << 8 | (nodeId & 0xff);

    return OK;
}

/*******************************************************************/
void RM_reset_node_handler(void) {
   reset_node_flag = -1;
}

/*******************************************************************/
void sendrand( ) {
   static unsigned int i;

   long cksum;
   long *pdata;

   pdata = (long *) prm->mem;
   cksum=0;

   /*printf("starting at %p\n", pdata);*/

   for (i=0; i<blocklen-1; i++) {

      *pdata = rand();
      cksum += *pdata;
      
      /*printf("val[%p] = %ld\n", pdata, *pdata );*/
      pdata++;
   }

   *pdata = cksum;

   /*
    printf("ending at %p with cksum val= %ld\n", pdata, *pdata );
    printf("cksum = %ld\n", cksum);
   */
   rmIntSend( intVector, currentNode );
}

/*******************************************************************/
void rmISR2(int node) {
   nodeISR2 = node;
   int2_flag = -1;
   int2_count++;

   currentNode = CEM;
}

/*******************************************************************/
void rmISR3(int node) {
   nodeISR3 = node;
   int3_flag = -1;
   int3_count++;
   
   intVector = INT2;
   currentNode = CEM;
}

/*******************************************************************/
void RM_bad_data_handler(void) {
   bad_data_count++;
}


/* rmTalkCEM
 * Main driver task for the communication
 *
 */
void rmTalkCEM(void) {

   sendrand();
   FOREVER {
      
      if (rmQuit) 
         break;

      semTake(updateNow, sysClkRateTicks); 

      if (p2Sendflag || int3_flag) {
         p2Sendflag = 0;
         int3_flag=0;
         sendrand();
      }
      
    }

   taskDelete(rmTaskId);
   sysAuxClkConnect((FUNCPTR) fireInterrupts, 0);
   sysAuxClkDisable();

}

void rmDiagnose(void) {

   /* Start pushing the data and interrupting CEM */
   rmTaskId = taskSpawn ("RMcom", 7, VX_FP_TASK, 20000, (FUNCPTR) rmTalkCEM, 
                  0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
   if (rmTaskId == ERROR) {
      logMsg ("unable to spawn rmTalkCEM task\n", 0, 0, 0, 0, 0, 0);
   }


   sysAuxClkRateSet(sysClkRateTicks);
   sysAuxClkConnect((FUNCPTR) fireInterrupts, 0);
   sysAuxClkEnable();
}

