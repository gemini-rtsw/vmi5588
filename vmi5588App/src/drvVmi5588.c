/* drvVmi5588.c -  Device driver for VMIC VMIVME5588 
*
*   Author:     Andrew Johnson
*   Date:       10-10-94
*
* Experimental Physics and Industrial Control System (EPICS)
*
* Developed at the Royal Greenwich Observatory for the Gemini
* 8M Telescopes Project.
*
* This driver incorporates some code from Bob Daly's VmiVme5576
* driver drvRM.c, dated 23 Oct 93, but is mostly new.
*/

/*
modification history
--------------------
1994-10-10   anj   Original release 

DESCRIPTION
This driver contains support routines for the VMIC VMIVME5588 Reflective
Memory card, for use both from EPICS and from a C subroutine.

INCLUDE FILES: vmi5588.h
*/


/* EPICS #includes */
#include    <epicsStdioRedirect.h>
#include    <epicsString.h>
#include    <epicsExit.h>
#include    <epicsExport.h>
#include    <epicsPrint.h>
#include    <cantProceed.h>
#include    <errlog.h>
#include    <dbDefs.h>
#include    <dbScan.h>
#include    <drvSup.h>
#include    <devSup.h>
#include    <devLib.h>
#include    <iocsh.h>
#include    "vmi5588.h"



/* These used to be defined by vxWorks */
#define OK 0
#define ERROR (-1)


/* Board addressing */
#define RM_DEBUG 1

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

/* Hardcoded board identification (This is set by jumpers) */
#define RM_BOARD_ID     0x4b

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


/* default base addr, mem size, interrupt vector and level */
#define RM_VME_BASE 0xA00000    /* A24 Address space */
#define RM_VME_SIZE 0x040000    /* 256 Kbytes long */
#define RM_INT_VECTOR   0xb0    /* 4 vectors used, b0 to b3 */
#define RM_INT_LEVEL       6    /* interrupts are urgent */

/* default values */
static unsigned int vmi5588_baseAddr = RM_VME_BASE;
static unsigned int vmi5588_memSize  = RM_VME_SIZE;
static unsigned int vmi5588_intVec   = RM_INT_VECTOR;
static unsigned int vmi5588_intLvl   = RM_INT_LEVEL;

 
/* Exported forward references */
long            vmi5588_report();
long            vmi5588_init();

/* Local forward references */
LOCAL void      vmi5588_reboot(void *p);

/* Driver support DRVET */
struct {
    long            number;
    DRVSUPFUN       report;
    DRVSUPFUN       init;
} drvVmi5588 = {
    2,
    vmi5588_report,
    vmi5588_init
};
epicsExportAddress(drvet, drvVmi5588);

/* Driver variables */
LOCAL void (*pisr[4])(int);



/* LOCAL SYMTAB_ID rmSymTbl; */
int rmMaxAttempts;

struct {
    long            number;
    struct {
    short           page;
    short           lastPageFlag;
    IOSCANPVT       ioscanpvt;
    } p[RM_NUM_PAGE];
} pageIo;


/* Memory layout of the reflective memory board */
struct {                    /* VMIVME5588    */
    char                     pad1;      /* unused    */
    char                     boardId;       /* BID       */
    volatile unsigned char   intRxStatus;   /* IRS       */
    char                     pad2;      /* unused    */
    unsigned char            nodeId;        /* NID       */
    volatile unsigned char   boardCsr;      /* CSR       */
    volatile unsigned short  cmd_Node;      /* CMD & CMDN    */
    char                     pad3[24];      /* unused    */
    struct {
    char                     pad4[2];   /* unused    */
    volatile unsigned char   senderId;  /* SIDn      */
    volatile unsigned char   control;   /* CRn       */
    }                        interrupt[4];  /* for n=0 to 3  */
    struct {
    char                     pad5[3];   /* unused    */
    volatile unsigned char   number;    /* VRn       */
    }                        vector[4];     /* for n=0 to 3  */
    /* Below here the card is just reflected RAM */
    volatile int             test;      /* to check ring */
    char                     pad6[0x100-0x44];  /* align to xx00 */
    volatile short           pageFlag[RM_NUM_PAGE]; /* Update flags  */
    short                    pad7[256-RM_NUM_PAGE]; /* align to xx00 */
    char                     pad8[0x100];   /* align to x400 */
    unsigned char   mem[RM_NUM_PAGE*RM_PAGE_SIZE];  /* data storage */
} *prm;

/*****************************************************************************
*
* vmi5588_init - DRVET Init function
*
* This routine initialises the vmi5588 driver.  It is called by EPICS in
* iocInit() as part of the driver initialisation process.
*
* It registers the card with the devLib address routines, and makes sure
* that a VMIC5588 card is present at the given location. 
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
    long            status;
    short           test;
    const char      vmi5588[] = "vmi5588";

    /* register the 5588 card in the A24 address space */
    status = devRegisterAddress(vmi5588, atVMEA24,  vmi5588_baseAddr,
                                vmi5588_memSize, (void *)&prm);
    if (status != OK) {
       errlogPrintf("vmi5588: Failed to register A24 Base Address\n"); 
       return status;
    }

    /* make sure something is out there */
    if (devReadProbe(sizeof(short), prm, &test) != OK) {
       devUnregisterAddress(atVMEA24, vmi5588_baseAddr, vmi5588);
       prm = NULL;
       errlogPrintf("vmi5588: device not present\n"); 
       return S_dev_noDevice;
    }
    
    /* is this the right card? */
    if (prm->boardId != RM_BOARD_ID) {
       errlogPrintf("vmi5588: wrong device ID, expected $%02x, found $%02x\n",
                     RM_BOARD_ID, prm->boardId); 
       devUnregisterAddress(atVMEA24, vmi5588_baseAddr, vmi5588);
       prm = NULL;
       return S_dev_wrongDevice;
    }

#ifdef RM_DEBUG
      printf("%s: RM card at addr %p, Board ID=$%02x, Node ID=$%02x\n", __FILE__, prm, prm->boardId, prm->nodeId);
#endif

      /* enable VMEbus Level 6 interrupts onto the card */
      status = devEnableInterruptLevel(intVME, vmi5588_intLvl);
      if (status != OK)
      return status;

      /* initialise the card interrupters */
      for (i = 0; i <= 3; i++) {
      pisr[i] = NULL;
      prm->vector[i].number = vmi5588_intVec + i;
      };

      /* turn any interrupts off if we do a reboot */
      /* rebootHookAdd((FUNCPTR) vmi5588_reboot); */
      epicsAtExit(vmi5588_reboot, NULL);

      rmMaxAttempts = 0;

      /* Finally we turn off the FAIL LED */
      prm->boardCsr &= ~RM_CSR_FAIL;
    
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

long vmi5588_report (void)
{
    unsigned char   irs, csr, icr;
    int             i;

    if (prm == NULL)
    return S_dev_NoInit;

    printf("vmi5588: RM node 0x%02x, status 0x%lx, max %d retries\n",
           prm->nodeId, rmStatus(0), rmMaxAttempts);

    printf("test address = 0x%x, mem starts at 0x%x\n", prm->test, *prm->mem);
    
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
    }

    return OK;
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

LOCAL void vmi5588_reboot (void *p)
{
    int             i;

    for (i = 0; i <= 3; i++)
    prm->interrupt[i].control &= ~RM_CR_INT_ENABLE;
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

void vmi5588_intr(void *p)
{
    int irqNumber = *(int *)p;
  /* RM interrupt channel number */
    if (prm == NULL || irqNumber < 0 || irqNumber > 3) {
    errlogPrintf("%s: Bad RM Interrupt, parameter = 0x%x", 
         __FILE__, irqNumber);
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
    errlogPrintf("%s: RM Interrupt #%d while disconnected", 
        __FILE__, irqNumber);
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
    void (*proutine)(int)    /* routine to call on int */
)
{
    long status;

    if (prm == NULL)
       return S_dev_NoInit;
    if (irqNumber < 0 || irqNumber > 3)
       return S_dev_vecInstlFail;
    if (pisr[irqNumber] != NULL)
       return S_dev_vectorInUse;

    /* plug in our wrapper routine */
    status = devConnectInterrupt(intVME, vmi5588_intVec + irqNumber,
                                 vmi5588_intr, (void *)&irqNumber);
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

/*****************************************************************************
*
* rmIntDisconnect - Disconnect an RM Interrupt routine
*
* This disables RM interrupts on the given channel number and marks the
* channel as unused.
*
* RETURNS:
* OK, or S_dev_???
*
* EXAMPLE:
* .CS
*   int status;
*
*   status = rmIntDisconnect(2);
*   if (status)
*       printf("No ISR Connected\\n");
* .CE
*
* SEE ALSO: rmIntConnect()
*/

long rmIntDisconnect
(
    int irqNumber       /* RM interrupt channel */
)
{
    if (prm == NULL)
    return S_dev_NoInit;
   if (irqNumber < 0 || irqNumber > 3 || pisr[irqNumber] == NULL)
    return S_dev_vectorNotInUse;

    /* disable the hardware */
    prm->interrupt[irqNumber].control &= ~RM_CR_INT_ENABLE;

    /* flag interrupt unused */
    pisr[irqNumber] = NULL;

    /* disconnect the software */
    return devDisconnectInterrupt(intVME, vmi5588_intVec + irqNumber,
                                  vmi5588_intr);
}

/*****************************************************************************
* rmIntSend - send an RM interrupt
*
* This causes an interrupt to be sent out on the RM bus, using the given
* channel number.  The interrupt will be broadcast to all other RM nodes
* if the node ID given is -1, otherwise it will be sent only to the node
* specified.
*
* You can't send an interrupt to yourself.  The hardware will ignore it.
*
* RETURNS:
* OK, or S_dev_???
*
* EXAMPLE:
* .CS
*   #define BROADCAST -1
*
*   rmIntSend(2, BROADCAST);
* .CE
*
* SEE ALSO: rmIntConnect(), rmIntDisconnect()
*/

long rmIntSend
(
    int irqNumber,      /* RM interrupt channel */
    int nodeId          /* node number, or -1 for broadcast */
)
{
    if (prm == NULL)
    return S_dev_NoInit;
    if (irqNumber < 1 || irqNumber > 3 || nodeId == prm->nodeId)
    return S_dev_badRequest;

    if (nodeId == -1)
    /* Broadcast to all nodes */
    prm->cmd_Node = irqNumber << 8 | RM_CMD_BROADCAST;
    else
    /* Node specific */
    prm->cmd_Node = irqNumber << 8 | (nodeId & 0xff);

    return OK;
}

/*****************************************************************************
*
* rmNodeId - return my RM Node number
*
* Used to find the value of the RM node ID setting on the VMIC5588 card.
*
* RETURNS:
* vmi5588 Node number (in the range 0 to 255), or S_dev_NoInit.
*
* EXAMPLE:
* .CS
*   long me;
*
*   me = rmNodeId();
*   if (me > 255L)
*       printf("RM error %d\\n", me);
* .CE
*
*/

long rmNodeId
(
    void
)
{
    if (prm == NULL)
    return S_dev_NoInit;

    return prm->nodeId;
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

unsigned long rmStatus
(
    long reset      /* Status bits to reset */
)
{
    unsigned long        stat;
    static unsigned char testCounter;

    if (prm == NULL)
    return S_dev_NoInit;

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
* vmi5588_pageISR - page trigger interrupt service routine
*
* Called by an incoming I/O Interrupt on channel 1, causes records
* in modified pages which are set to I/O Interrupt scanning to be
* processed.
*
* RETURNS: N/A
*
* NOMANUAL
*/

void vmi5588_pageISR
(
    int rmNodeId        /* source Node Id */
)
{
    short i;

#ifdef RM_DEBUG
    errlogPrintf("Interrupt: vmi5588_pageISR, from Node <%x>\n", rmNodeId);
#endif

    /* check for new data in page */
    for (i = 0; i < pageIo.number; i++)
    if (prm->pageFlag[pageIo.p[i].page] != pageIo.p[i].lastPageFlag) {
        pageIo.p[i].lastPageFlag = prm->pageFlag[pageIo.p[i].page];

#ifdef RM_DEBUG
        errlogPrintf("Triggering page <%d>\n", pageIo.p[i].page);
#endif

        scanIoRequest(pageIo.p[i].ioscanpvt);
    };
}

/*****************************************************************************
*
* vmi5588_pageInit - initialise page data structure
*
* Initialises the page structure used for incoming I/O Interrupts.
* No action if page has already been initialised.
*
* RETURNS:
* OK, or S_dev_??? if can't connect interrupt
*
* NOMANUAL
*/

long vmi5588_pageInit
(
    short rmPage        /* rm page number */
)
{
    short pageIndex;
    long status = OK;

    for (pageIndex = 0; pageIndex < pageIo.number; pageIndex++)
    if (rmPage == pageIo.p[pageIndex].page)
        return OK;

    if (pageIndex == 0)
    if ((status = rmIntConnect(1, vmi5588_pageISR)) != OK)
        errMessage(status, "Can't connect RM irq #1");

    pageIo.p[pageIndex].page = rmPage;
    scanIoInit(&pageIo.p[pageIndex].ioscanpvt);
    pageIo.number++;

#ifdef RM_DEBUG
    printf("Init pageIo index <%hd> page <%ld>\n",
           pageIndex, pageIo.number);
#endif

    return status;
}

/*****************************************************************************
*
* vmi5588_pvtInit - initialise private data structure
*
* Allocates storage for and initialises private record data, returning
* a pointer to the initialised structure in the first parameter.
*
* RETURNS: N/A
*
* NOMANUAL
*/

void vmi5588_pvtInit
(
    struct rmpvt **ppdpvt,  /* location for private pointer */
    short rmPage,       /* rm page number */
    short rmOffset,     /* offset into rm page */
    struct rm_data *prmData /* pointer to shared memory */
)
{
    struct rmpvt   *prmpvt;

    prmpvt = (struct rmpvt *)callocMustSucceed(1, sizeof(struct rmpvt),
                  "vmi5588_pvtInit: Can't allocate memory");
    prmpvt->page = rmPage;
    prmpvt->offset = rmOffset;
    prmpvt->address = prmData;

#ifdef RM_DEBUG
   epicsPrintf("page<%x> offset<%x> address<%p>\n", 
           rmPage, rmOffset, prmData);
#endif

    *ppdpvt = prmpvt;
}

/*****************************************************************************
*
* vmi5588_getIoscanpvt - look up I/O interrupt data structure
*
* Returns the IOSCANPVT token for a record given its dpvt structure.
*
* RETURNS: OK, or S_dev_internal if no entry found.
*
* NOMANUAL
*/

long vmi5588_getIoscanpvt
(
    struct rmpvt *pdpvt,    /* record private data structure */
    IOSCANPVT *pscanpvt     /* location for interrupt data */
)
{
    short i, page;

    page = pdpvt->page;
    for (i = 0; i < pageIo.number; i++)
    if (page == pageIo.p[i].page) {
        *pscanpvt = pageIo.p[i].ioscanpvt;
        return OK;
    };

/* Question: could I call vmi5588_pageInit at this point, since it
   obviously hasn't been done yet?  This would allow a record to be
   set to I/O Interrupt Scanning after initialisation.
   It may be illegal to call ScanIoInit now.

   Answer: It's legal; add this when I get the chance.
*/

    return S_dev_internal;
}

/*****************************************************************************
*
* vmi5588_trigger - trigger a remote I/O interrupt
*
* Sends a broadcast interrupt to trigger I/O interrupt processing for all
* RM pages on given page number.
*
* RETURNS: OK, or S_dev_???
*
* NOMANUAL
*/

long vmi5588_trigger
(
    short rmPage        /* rm page number */
)
{
    static unsigned char nodeIDcounter;

    prm->pageFlag[rmPage] = (prm->nodeId << 8) + nodeIDcounter++;

#ifdef RM_DEBUG
    printf("page <%hd>  pageFlag[page] <%hd>\n",
            rmPage, prm->pageFlag[rmPage]);
    errlogPrintf("sending interrupt to all RMs\n");
#endif

    return rmIntSend(1, -1);
}



int drvVmi5588Config(unsigned baseAddr, unsigned memSize, unsigned intVec, unsigned intLvl)
{
   vmi5588_baseAddr = baseAddr;
   vmi5588_memSize  = memSize;
   vmi5588_intVec   = intVec;
   vmi5588_intLvl   = intLvl;
   return 0;
}

static const iocshArg drvVmi5588Arg0 = {"baseAddr", iocshArgInt};
static const iocshArg drvVmi5588Arg1 = {"memSize", iocshArgInt};
static const iocshArg drvVmi5588Arg2 = {"intVec", iocshArgInt};
static const iocshArg drvVmi5588Arg3 = {"intLvl", iocshArgInt};
static const iocshArg *drvVmi5588ConfigArgs[] = 
         {&drvVmi5588Arg0,&drvVmi5588Arg1,&drvVmi5588Arg2,&drvVmi5588Arg3};
static const iocshFuncDef drvVmi5588ConfigFuncDef = 
         {"drvVmi5588Config", 4, drvVmi5588ConfigArgs};       
static void drvVmi5588ConfigCallFunc(const iocshArgBuf *args)
{
   drvVmi5588Config(args[0].ival, args[1].ival,args[2].ival,args[3].ival);
}

static void drvVmi5588RegisterCommands(void)
{
  static int firstTime = 1;   
  if(firstTime) {
     iocshRegister(&drvVmi5588ConfigFuncDef, drvVmi5588ConfigCallFunc);

     firstTime = 0;
   }
}
epicsExportRegistrar(drvVmi5588RegisterCommands);
