#include <string.h>
#include <epicsThread.h>
#include <epicsEvent.h>
#include <iocsh.h>
#include <epicsExport.h>
#include <epicsPrint.h>
#include <math.h>
#include <stdlib.h>


#include "vmi5588.h"

#define INT0 0
#define INT1 1
#define INT2 2
#define INT3 3
#define OK 0
#define ERROR (-1)
unsigned int pingpongISR_count = 0;
double waittime = 2.0;
int stop = 0;
int isrnode = -1;
int prepared = 0;
epicsEventId intFlag;
epicsThreadId tid = 0;

typedef struct
{
    long            checksum;
    float           testfloat;
    long            testlong;
    float           testfloat2;
    long            testlong2;
    long            pad[251]; 
}commandBlock;

typedef struct
{
    commandBlock    page0;
}memMap;


/*******************************************************************/
void pingpongISR(int node) {
   isrnode = node; 
   pingpongISR_count++;
   epicsEventSignal(intFlag);
}
#define BLKLEN 256
#define WORDSIZE sizeof(unsigned long) /*4 bytes*/
#define oneK (WORDSIZE * BLKLEN )
#define sixteenthMeg (oneK * 64)  /*64 k*/
#define eighthMeg (oneK * 128)  /*128 k*/
#define quarterMeg (oneK * BLKLEN) /*256 k*/
#define END 128*WORDSIZE*BLKLEN-(3*WORDSIZE)

enum machines {VME_0, VME_1, VME_2, CEM=1};
int _pEnd = 4;

// Assumes 0 <= max <= RAND_MAX
// Returns in the closed interval [0, max]
// From http://stackoverflow.com/questions/2509679/how-to-generate-a-random-number-from-within-a-range
long random_at_most(long max) {
  unsigned long
    // max <= RAND_MAX < ULONG_MAX, so this is okay.
    num_bins = (unsigned long) max + 1,
    num_rand = (unsigned long) RAND_MAX + 1,
    bin_size = num_rand / num_bins,
    defect   = num_rand % num_bins;

  long x;
  do {
   x = rand();
  }
  // This is carefully written not to overflow
  while (num_rand - defect <= (unsigned long)x);

  // Truncated division is intentional
  return x/bin_size;
}

void checkEcho(void) {

    unsigned long i, cksum;
    volatile unsigned long *pdata;
    volatile unsigned long *pEchoData;

    pdata = (volatile unsigned long*)rmPageMemBase();
    pEchoData = pdata+(BLKLEN * 4); /*offset into next block to see echo area*/
    cksum=0;

    for (i=0; i<_pEnd; i++)  {
        cksum += *pEchoData;
        if ( i<4|| i== END) 
            errlogPrintf("echoval[%p] = %lu\n\n", pEchoData, *pEchoData );
        pEchoData++;
    }

    errlogPrintf("checkEcho ending at %p, cksum =%lu, received cksum val= %lu\n",
            pEchoData, cksum, *pEchoData );
}


void clearMemory(volatile unsigned long *p) {
    int i;

    errlogPrintf("\n\nClearing all RM from %p to %p \n", p, p+(sizeof(p)*_pEnd) );
    for (i=0; i<_pEnd+1; i++)  {
        if(i<4)
            errlogPrintf("Clearing %p \n", p );
       *(p++) = 0; 
    }
    errlogPrintf("Clear ending location -1 at %p with = %lu\n", p-1, *(p-1) );
    errlogPrintf("Clear ending location at %p with = %lu\n", p, *p );
}

int firstpass = 1;

/* pingPong()
 * Main driver task for the communication
 *
 */
void pingPong(void *p) {
//   unsigned char nodeId = rmNodeId();
//   volatile unsigned long *pdata = (volatile unsigned long*)rmPageMemBase();
//   float *ptest = (float *) (pdata+1);
//   volatile unsigned long *psave;
//   unsigned long i, cksum;
//   float testfloat = M_PI;

   memMap *scsBase;
   commandBlock *localPtr;
   
   scsBase = (memMap *) rmPageMemBase();
   localPtr = (commandBlock *) &(scsBase->page0);

   errlogPrintf("scsBase %p localPtr %p \n",scsBase, localPtr);
   errlogPrintf("lp->testfloat  address: %p \n", &localPtr->testfloat);
   errlogPrintf("lp->testlong   address: %p \n", &localPtr->testlong);
   errlogPrintf("lp->testfloat2 address: %p \n", &localPtr->testfloat2);
   errlogPrintf("lp->checksum   address: %p \n", &localPtr->checksum);


   localPtr->testfloat =  M_PI;
   localPtr->testlong  =  random_at_most(65535);
   localPtr->testfloat2 =  M_PI + 10;
   localPtr->testlong2  =  random_at_most(65535);
   localPtr->checksum = localPtr->testfloat+
                        localPtr->testlong +
                        localPtr->testfloat2 +
                        localPtr->testlong2;

   errlogPrintf("testFloat %f, testLong = %ld\n", localPtr->testfloat, localPtr->testlong);
   errlogPrintf("ending at with cksum val= %lu\n\n", localPtr->checksum );

   rmIntSend(INT2, CEM); /*Interrupt Int2 on Node 2 which equals CEM or VME_2 */

  // psave = pdata;

/*     if (firstpass) {
        firstpass = 0;
        clearMemory(pdata);

        pdata = psave; 
        errlogPrintf("Check Echo region at %p with = %lu\n\n", pdata, *pdata );
        errlogPrintf("Check Echo region at %p with = %lu\n\n", pdata+2*BLKLEN-1, *(pdata + 2*BLKLEN-1) );

    }
 */
    /*VME_0 VME_1 VME_2 and CEM*/
/*     if(nodeId == VME_0 ) {
        errlogPrintf("starting at %p\n", pdata);
        cksum=0;
        for (i=0; i<_pEnd; i++)  {
          if (i == 0){
              *pdata = _pEnd;
          }
          else if (i==1) {
              *ptest = testfloat;
          }
          else{
              *pdata = random_at_most(65535);
          }
            //pdata = i + 10;
            if ( i<4 || i==END){ 
              if (i==1)
              errlogPrintf("val[%p] = %f\n", ptest, *ptest );
              else
              errlogPrintf("val[%p] = %lu\n", pdata, *pdata );
            }
            cksum += *pdata;
            pdata++;
        }
        *pdata = cksum;

        errlogPrintf("ending at %p with cksum val= %lu\n", pdata, *pdata );

        // if we're node 1, we kick things off 
        //errlogPrintf("Initial serve...\n");
        // tell other system that new data is available 
        // rmIntSend(INT2, CEM); Interrupt Int2 on Node 2 which equals CEM or VME_2 
    } */
 
   while(1) {
      if (stop) epicsThreadSuspendSelf();

      //errlogPrintf("Waiting ... isrnode=%d\n",isrnode);
      /* wait for interrupt */
      epicsEventMustWait(intFlag);
      
      /*reset pointer*/
//      pdata = psave; 

      /* wait another 2 seconds */
      epicsThreadSleep(waittime); 
      
   //   checkEcho();

   localPtr->testfloat =  M_PI;
   localPtr->testlong  =  random_at_most(65535);
   localPtr->testfloat2 =  M_PI + 10;
   localPtr->testlong2  =  random_at_most(65535);
   localPtr->checksum = localPtr->testfloat+localPtr->testlong+localPtr->testfloat2+localPtr->testlong2;
   
   errlogPrintf("testFloat %f testLong = %ld\n", localPtr->testfloat, localPtr->testlong);
   errlogPrintf("testFloat2 %f testLong2 = %ld\n", localPtr->testfloat2, localPtr->testlong2);
   errlogPrintf("ending at with cksum val= %lu\n", localPtr->checksum );

   rmIntSend(INT2, CEM); /*Interrupt Int2 on Node 2 which equals CEM or VME_2 */


/*       cksum=0;

      for (i=0; i<_pEnd; i++)  {
          if (i == 0){
              *pdata = _pEnd;
          }
          else if (i==1) {
              *ptest = testfloat;
          }
          else{
              *pdata = random_at_most(65535);
          }
          cksum += *pdata;
          if ( i<4|| i== END) { 
              
              if (i==1)
              errlogPrintf("val[%p] = %f\n", ptest, *ptest );
              else
              errlogPrintf("val[%p] = %lu\n", pdata, *pdata );
              
          
          }

          pdata++;
      }
      *pdata = cksum;
 */
      //errlogPrintf("ending at %p with cksum val= %lu\n", pdata, *pdata );

      /* tell other system (isrnode) that new data is available */
      //rmIntSend(INT2, isrnode); /*isrnode is the caller that just interrupted us!*/
   }
}

void prep(int irq) {
   long status;

   if ((status = rmIntConnect(irq, pingpongISR)) != OK) {
       perror("rmIntConnect(INT2, pingpong)");
       errlogPrintf("rmIntConnect fail with %ld. Pingpong task not started\n", status);
   }
   errlogPrintf("Prep finished\n");
   prepared = 1;

}

void ppStart(void) {

    if (!prepared)
        prep(2);

    if (!tid) {
        tid = epicsThreadCreate("Ping Pong", 
                epicsThreadPriorityMedium,
                epicsThreadGetStackSize(epicsThreadStackMedium),
                pingPong,
                NULL);

        intFlag = epicsEventMustCreate(epicsEventEmpty);
    }
    else if(epicsThreadIsSuspended(tid)) 
    {
        epicsThreadResume(tid);
        stop = 0;
    }
}
static const iocshFuncDef ppStartFuncDef =
           {"ppStart", 0, NULL};

static void ppStartCallFunc(const iocshArgBuf *args)
{
   ppStart();
}

void ppStop(void)
{
   stop = 1;
}
static const iocshFuncDef ppStopFuncDef =
           {"ppStop", 0, NULL};

static void ppStopCallFunc(const iocshArgBuf *args)
{
   ppStop();
}

static const iocshArg prepArg0 = {"irq", iocshArgInt};
static const iocshArg *prepArgs[] = {&prepArg0};
static const iocshFuncDef prepFuncDef = {"prep", 1, prepArgs};       

static void prepCallFunc(const iocshArgBuf *args)
{
   prep(args[0].ival);
}

static void ppStartRegisterCommands(void)
{
   static int firstTime = 1;
   if (firstTime) {
      iocshRegister(&ppStartFuncDef, ppStartCallFunc);
      iocshRegister(&ppStopFuncDef,  ppStopCallFunc);
      iocshRegister(&prepFuncDef,    prepCallFunc);
      firstTime = 0;
   }
}
epicsExportRegistrar(ppStartRegisterCommands);
epicsExportAddress(int, pingpongISR_count);
epicsExportAddress(int, _pEnd);
epicsExportAddress(double, waittime);


