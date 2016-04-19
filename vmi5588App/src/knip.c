
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

#define RM_VME_BASE 0xA00000    /* A24 Address space */
#define RM_VME_SIZE 0x040000    /* 256 Kbytes long */

/* Interrupt addressing */
#define RM_INT_VECTOR   0xb0    /* 4 vectors used, b0 to b3 */
#define RM_INT_LEVEL    6   /* interrupts are urgent */

/* Internal driver sizes */
#define RM_PAGE_SIZE 0x0400 /* 1024 bytes per page */
#define RM_NUM_PAGE  255    /* max number of pages */

#define RM_SYM_HASHSIZE    6    /* Hash size for symbol table (log2) */
#define RM_INPUTLINESIZE 256    /* Input line buffer length */
#define RM_INPUTTYPESIZE  20    /* type keyword buffer length */
#define RM_INPUTNAMESIZE 128    /* name buffer length */

/* Storage to allocate for each shared rm_data structure */
#define RM_SIZE_ALOG 16
#define RM_SIZE_LONG 12
#define RM_SIZE_STRG 48
#define RM_SIZE_ARRY 16 /* Plus size of array */

/* Board identification */
#define RM_BOARD_ID     0x42

/* intRxStatus bits */
#define RM_IRS_INT1     0x01
#define RM_IRS_INT2     0x02
#define RM_IRS_INT3     0x04
#define RM_IRS_RX_SIG       0x10
#define RM_IRS_VIOLATION    0x20
#define RM_IRS_LATCHED      0x40
#define RM_IRS_RX_SYNC      0x80

/* boardCsr bits */
#define RM_CSR_FAST     0x01
#define RM_CSR_MASK     0x02
#define RM_CSR_OWN_DATA     0x04
#define RM_CSR_BAD_DATA     0x08
#define RM_CSR_TX_EMPTY     0x10
#define RM_CSR_TX_HALF      0x20
#define RM_CSR_RX_HALF      0x40
#define RM_CSR_FAIL     0x80

/* cmd_Node bits */
#define RM_CMD_RESET        0x0000
#define RM_CMD_INT1     0x0100
#define RM_CMD_INT2     0x0200
#define RM_CMD_INT3     0x0300
#define RM_CMD_BROADCAST    0x4000

/* interrupt[n].control bits */
#define RM_CR_INT_LEVEL6    0x06
#define RM_CR_INT_AUTOCLR   0x08
#define RM_CR_INT_ENABLE    0x10

/* Exported forward references */
long            vmi5588_report();
long            vmi5588_init();

enum {
   GOOD = 0, 
   BAD,
   BAD_CANNOT_PROBE,
   BAD_BOARDID 
};

/* Local forward references */
LOCAL void      vmi5588_reboot(int startType);
LOCAL void      rmTalkCEM(void);

LOCAL void fireInterrupts();


/* Driver variables */
LOCAL VOIDFUNCPTR pisr[4];
int rmMaxAttempts;

/* Memory layout of the reflective memory board */
struct {                                        /* VMIVME5588    */
    char                     pad1;              /* unused    */
    char                     boardId;           /* BID       */
    volatile unsigned char   intRxStatus;       /* IRS       */
    char                     pad2;              /* unused    */
    unsigned char            nodeId;            /* NID       */
    volatile unsigned char   boardCsr;          /* CSR       */
    volatile unsigned short  cmd_Node;          /* CMD & CMDN    */
    char                     pad3[24];          /* unused    */
    struct {
    char                     pad4[2];           /* unused    */
    volatile unsigned char   senderId;          /* SIDn      */
    volatile unsigned char   control;           /* CRn       */
    }                        interrupt[4];      /* for n=0 to 3  */
    struct {
    char                     pad5[3];           /* unused    */
    volatile unsigned char   number;            /* VRn       */
    }                        vector[4];         /* for n=0 to 3  */
                                                /* Below here the card is just reflected RAM */
    volatile int             test;              /* to check ring */
    char                     pad6[0x100-0x44];  /* align to xx00 */
    volatile short           pageFlag[RM_NUM_PAGE];     /* Update flags  */
    short                    pad7[256-RM_NUM_PAGE];     /* align to xx00 */
    char                     pad8[0x100];               /* align to x400 */
    unsigned char   mem[RM_NUM_PAGE*RM_PAGE_SIZE];      /* data storage */
} *prm;


LOCAL void fireInterrupts() {
   static int tickCounts = 0;

   if (tickCounts++ >= intRate) {
      tickCounts = 0;
      p2Sendflag = -1;
      semGive(updateNow);
   }
}

/*****************************************
 * rmSIE -- This enables the VME interrupt level
 *
 */
int rmSIE (int level)
{
   int status;

   if ( level<1 || level>7 ) {
       printf ("argument out of range: level 1..7\n");
       return ERROR;
   }
   if ((status = sysIntEnable(level)) == ERROR) {
       printf ("error attempting to enable bus interrupt level %d\n", level);
       return ERROR;
   }
   return OK;
}

/*****************************************************************************
 * rmSID -- This disables the VME interrupt level
 *
 *
 *
 */
int rmSID (int level)
{
    int status;

    if ( level<1 || level>7 ) {
        printf ("argument out of range: level 1..7\n");
        return ERROR;
    }
    if ((status = sysIntDisable(level)) == ERROR) {
        printf ("error attempting to disable bus interrupt level %d\n", level);
        return ERROR;
    }
    return OK;
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
* vmi5588_init - DRVET Init function
*
* This routine initialises the vmi5588 driver.
*
* RETURNS:
* OK, or S_dev_??? errors.
*/

long vmi5588_init
(
    void
)
{
   int             i;
   int             error = 0;
   long            status;
   char            test;
   const char      vmi5588[] = "vmi5588";

   if ( (updateNow = semBCreate(SEM_Q_PRIORITY, SEM_EMPTY)) == NULL) {
      printf("error creating updateNow semaphore\n");
   }

    /* register the 5588 card in the A24 address space */
    status = devRegisterAddress(vmi5588, atVMEA24, (void *) RM_VME_BASE,
                                RM_VME_SIZE, (void **) &prm);
    if (status != OK)
    return status;

   /* make sure something is out there */
   if (vxMemProbe(&prm->boardId, READ, sizeof(char), &test) != OK) {
      prm   = NULL;
      error = BAD_CANNOT_PROBE;
   };

   if(!error)
   {
      /* is this the right card? */
      if (prm->boardId != RM_BOARD_ID) {
         printf("boardId %d read from addr: %x\n", test, prm->boardId);
         prm = NULL;
         return BAD_BOARDID;
      };

      printf("%s: Found RM card at addr %x\n",
            __FILE__, (unsigned int)prm);

      /* enable VMEbus Level 6 interrupts onto the card */
      status = rmSIE(RM_INT_LEVEL);
      if (status != OK)
         return status;

      /* initialise the card interrupters */
      for (i = 0; i <= 3; i++) {
         pisr[i] = NULL;
         prm->vector[i].number = RM_INT_VECTOR + i;
      };

      /* turn any interrupts off if we do a reboot */
      rebootHookAdd((FUNCPTR) vmi5588_reboot);

      rmMaxAttempts = 0;

      /* Finally we turn off the FAIL LED */
      prm->boardCsr &= ~RM_CSR_FAIL;
   }


   return OK;
}


/*****************************************************************************
*
* vmi5588_report - DRVET Report function
*
* This is the driver status report function, which displays the current
* status of the hardware when called, usually from dbior().
*
* RETURNS:
* OK, or S_dev_NoInit if driver not initialised.
*
* EXAMPLE:
* .CS
*   -> vmi5588_report
*   vmi5588: RM node 0x1, status 0x40, max 0 retries
*   value = 0 = 0x0
* .CE
*/

long vmi5588_report
(
    void
)
{
    if (prm == NULL)
    return BAD;

    printf("vmi5588: RM node 0x%x, status 0x%x, max %d retries\n",
           prm->nodeId, (unsigned int)rmStatus, rmMaxAttempts);

#ifdef RM_DEBUG
    printf("test address = 0x%x, mem starts at 0x%x, Card starts at: 0x%x\n", 
            prm->test, (unsigned int) prm->mem, (unsigned int) prm);
    {
    unsigned char   irs, csr, icr;
    int             i;

    /* read status */
    irs = prm->intRxStatus;
    csr = prm->boardCsr;

    printf("    Receiver: %s, PLL %s%s%s\n",
           irs & RM_IRS_RX_SIG ? "NO INPUT SIGNAL" : "Input signal good",
           irs & RM_IRS_VIOLATION ? "RESYNC NEEDED" : "Locked",
           irs & RM_IRS_LATCHED ? ", Recent Sync loss" : "",
           irs & RM_IRS_RX_SYNC ? ", SYNC BIT HIGH" : "");

    printf("    Jumpers:  %s mode, Transfer Error Interrupt %s\n",
           csr & RM_CSR_FAST ? "Fast" : "Slow",
           csr & RM_CSR_MASK ? "Disabled" : "Enabled");

    printf("    Status:   Fibre ring %s, %sFail LED %s\n",
           csr & RM_CSR_OWN_DATA ? "Intact" : "BROKEN",
           csr & RM_CSR_BAD_DATA ? "TRANSFER ERROR, " : "",
           csr & RM_CSR_FAIL ? "ON" : "Off");

    printf("    FIFOs:    Transmitter %s, Receiver %s\n",
           csr & RM_CSR_TX_EMPTY ? csr & RM_CSR_TX_HALF ?
           "<50% Full" : ">50% FULL" : "Empty",
           csr & RM_CSR_RX_HALF ? "<50% Full" : ">50% FULL");

    printf("    Int's:    %s%s%s\n",
           irs & RM_IRS_INT1 ? "Irq 1 pending " : "",
           irs & RM_IRS_INT2 ? "Irq 2 pending " : "",
           irs & RM_IRS_INT3 ? "Irq 3 pending " : "");

    for (i = 0; i <= 3; i++) {
        icr = prm->interrupt[i].control;

        printf("       Int %d: %s, Level %d %s, %svector %x\n", i,
           pisr[i] != 0 ? "Allocated" : "Not in use",
           icr & 7,
           icr & RM_CR_INT_ENABLE ? "enabled" : "disabled",
           icr & RM_CR_INT_AUTOCLR ? "Auto clear, " : "",
           prm->vector[i].number);
    };
    };
#endif

    return OK;
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

/*****************************************************************************
*
* vmi5588_reboot - rebootHook routine
*
* This routine is connected to the vxWorks reboot Hook by vmi5588_init,
* and is responsible for turning off RM interrupts. This is essential
* because other nodes may continue to send interrupts after we have
* restarted.
*
* RETURNS: N/A
*
* NOMANUAL
*/

LOCAL void vmi5588_reboot
(
    int startType       /* vxWorks reboot type */
)
{
    int             i;

    logMsg("Reboot: Disabling RM interrupts\n", 0, 0, 0, 0, 0, 0);

    for (i = 0; i <= 3; i++)
    prm->interrupt[i].control &= ~RM_CR_INT_ENABLE;
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

/******************************************************************
void rmConnectCEM(void) {
   time_t tmo;
   int err,row,col;
   int isrps_x, isrps_y, bdtps_x, bdtps_y, becps_x, becps_y;
   int isr_x,   isr_y,   bdt_x,   bdt_y,   bec_x,   bec_y;
   static unsigned long old_int1_count,new_int1_count;
   static unsigned long old_bad_count,new_bad_count;
   static unsigned long old_bad_echo_count,new_bad_echo_count;
   static unsigned long patadr,echadr,pat,ech,i;

   randomize();

   err = init_RM();
   if (err) { printf("\nCan't init RM: %d", err ); quit(); }

   RM_send_loopback();
   tmo = time(NULL)+2;
   while ( !RM_loopback_arrived() ) {
      if ( time(NULL) > tmo ) {
         printf("\nRM loopback failed");
         quit();
      }

   }

   if ( RM_addr_base == 0 ) {
      printf("\RM base address not set");
      quit();
   }

   row = 8;
   col = 5;

   gotoxy(col,row);
   cprintf("INT1 count per sec: ");
   isrps_x = wherex()+2;
   isrps_y = wherey();

   row++;

   gotoxy(col,row);
   cprintf("Total INT1 count: ");
   isr_x = wherex()+2;
   isr_y = wherey();

   row+=2;

   gotoxy(col,row);
   cprintf("Bad data xfers per sec: ");
   bdtps_x = wherex()+2;
   bdtps_y = wherey();

   row++;

   gotoxy(col,row);
   cprintf("Total bad data xfers: ");
   bdt_x = wherex()+2;
   bdt_y = wherey();

   row+=2;

   gotoxy(col,row);
   cprintf("Bad echos per sec: ");
   becps_x = wherex()+2;
   becps_y = wherey();

   row++;

   gotoxy(col,row);
   cprintf("Total bad echos: ");
   bec_x = wherex()+2;
   bec_y = wherey();

   old_int1_count = int1_count;
   old_bad_count = bad_data_count;
   old_bad_echo_count = bad_echo_count;

   tmo = time(NULL);

   int1_flag = 0;
   sendrand(RM_addr_base+4*BLKLEN);

   while (!kbhit()) {
      if ( int1_flag ) {
         int1_flag = 0;
         patadr = RM_addr_base + 4*BLKLEN;
         echadr= patadr + 4*BLKLEN;

         for (i=0;i<BLKLEN;i++) {
            pat = big_read32(patadr);
            ech = big_read32(echadr);
            if ( pat != ech ) {
               bad_echo_count++;
               break;
            }
            patadr+=4;
            echadr+=4;
         }

         sendrand(RM_addr_base+4*BLKLEN);
      }

      if ( time(NULL) > tmo ) {
         gotoxy(isrps_x,isrps_y);
         clreol();
         new_int1_count=int1_count;
         cprintf("%lu",new_int1_count-old_int1_count);
         old_int1_count=new_int1_count;

         gotoxy(isr_x,isr_y);
         clreol();
         cprintf("%lu",int1_count);

         gotoxy(bdtps_x,bdtps_y);
         clreol();
         new_bad_count=bad_data_count;
         cprintf("%lu",new_bad_count-old_bad_count);
         old_bad_count=new_bad_count;

         gotoxy(bdt_x,bdt_y);
         clreol();
         cprintf("%u",bad_data_count);

         gotoxy(becps_x,becps_y);
         clreol();
         new_bad_echo_count=bad_echo_count;
         cprintf("%lu",new_bad_echo_count-old_bad_echo_count);
         old_bad_echo_count=new_bad_echo_count;

         gotoxy(bec_x,bec_y);
         clreol();
         cprintf("%u",bad_echo_count);

         tmo+=1;
      }
   }

   if (getch()==0) (void)getch();

   quit();
}
*/
void end() {

}
