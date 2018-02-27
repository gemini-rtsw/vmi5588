#include <string.h>
#include <epicsThread.h>
#include <epicsEvent.h>
#include <iocsh.h>
#include <epicsExport.h>
#include <epicsPrint.h>
#include <inttypes.h>
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
    unsigned long     length;
    float             testfloat;
    long     testlong;
    float             testfloat2;
    long     checksum;
    unsigned long     testlong2;
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

/* mrippa Feb. 2018. Use the random_at_most(long) routine
 * above to return an signed long, possibly negative, number.
 * 
 * With 16-bits, a signed value is in the range [-32768, 32767].
 * That is, [-(2^(16-1)), 2^(16-1)-1]. So, just get the random
 * integer in the range [0, 65535], then always subtract the
 * largest negative value which is 32768. Check the end cases:
 *
 * Case 1: Highest possible positive value.
 *         The random_at_most(65535) actually returns 65535.
 *         This case returns 65535 - 32768 = 32767.
 *
 * Case 2: Lowest possible value is negative.
 *         The random_at_most(65535) actually returns 0.
 *         This case returns 0 - 32768 = -32768.
 *
 */
long random_maybe_negative(void) {
    return (random_at_most(65537) - 32768);
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
            errlogPrintf("echoval[%p] = %lu\n", pEchoData, *pEchoData );
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

long checkSum_long (void *ptr, int numLongs)
{
    long *checkPtr = (long *) ptr;
    long sum = 0;
    int  n;

    for (n = 0; n < numLongs; n++)
    {
        sum += *checkPtr++;
    }

    return (sum);
}


uint32_t checkSum (void *ptr, int numLongs)
{
    uint32_t   *checkPtr = (uint32_t *) ptr;
    uint32_t sum = 0;
    int     n;

    for (n = 0; n < numLongs; n++)
    {
        sum += *checkPtr++;
    }

    return (sum);
}

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

   rmIntSend(INT2, CEM); //Interrupt Int2 on Node 2 which equals CEM or VME_2 
 
   while(1) {
       if (stop) epicsThreadSuspendSelf();

       /* wait for interrupt */
       epicsEventMustWait(intFlag);

       /* wait another delay to slowthings */
       epicsThreadSleep(waittime); 
       checkEcho();

       localPtr->length =  _pEnd;
       localPtr->testfloat =  M_PI;
       localPtr->testlong  =  random_maybe_negative(); 
       localPtr->testfloat2 =  -M_PI + 10;
       localPtr->testlong2  =  random_maybe_negative(); 
       //localPtr->checksum = localPtr->length  + localPtr->testfloat;
       //localPtr->checksum = checkSum((void *) &localPtr->length, _pEnd);
       localPtr->checksum = checkSum_long((void *) &localPtr->length, _pEnd);

       //localPtr->testlong   +
       //localPtr->testfloat2 +
       //localPtr->testlong2;

       errlogPrintf("testfloat %f\n", localPtr->testfloat);
       errlogPrintf("testlong = %ld\n",  localPtr->testlong);
       errlogPrintf("testfloat2 = %f\n",  localPtr->testfloat2);
       //errlogPrintf("testlong2 = %" PRIu32"\n",  localPtr->testlong2);
       errlogPrintf("ending at with cksum val= %ld\n\n", localPtr->checksum );

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


