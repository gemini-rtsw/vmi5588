#include <string.h>
#include <epicsThread.h>
#include <epicsEvent.h>
#include <iocsh.h>
#include <epicsExport.h>
#include <epicsPrint.h>
#include <stdlib.h>


#include "vmi5588.h"

#define INT0 0
#define INT1 1
#define INT2 2
#define INT3 3
#define OK 0
#define ERROR (-1)
unsigned int pingpongISR_count = 0;
epicsExportAddress(int, pingpongISR_count);
int stop = 0;
epicsEventId intFlag;
epicsThreadId tid = 0;

/*******************************************************************/
void pingpongISR(int node) {
   pingpongISR_count++;
   epicsEventSignal(intFlag);
}
#define BLKLEN 256
#define oneK (sizeof(unsigned long) * BLKLEN )
#define sixteenthMeg (oneK * 64)  /*64 k*/
#define eighthMeg (oneK * 128)  /*128 k*/
#define quarterMeg (oneK * BLKLEN) /*256 k*/

#define _pEnd (oneK - 2)

void clearMemory(volatile unsigned long *p) {
    int i;

    printf("\n\nClearing all RM from %p to %p \n", p, p+(sizeof(p)*_pEnd) );
    for (i=0; i<_pEnd+1; i+=4)  {
        if(i<16)
            printf("Clearing %p \n", p );
       *(p++) = 0; 
    }
    printf("Clear ending location -1 at %p with = %lu\n", p-1, *(p-1) );
    printf("Clear ending location at %p with = %lu\n", p, *p );
}

int firstpass = 1;

/* pingPong()
 * Main driver task for the communication
 *
 */
void pingPong(void *p) {
   unsigned char nodeId = rmNodeId();
   volatile unsigned long *pdata = (volatile unsigned long*)rmPageMemBase();
   volatile unsigned long *psave;
   unsigned long i, cksum;
   psave = pdata;

    if (firstpass) {
        firstpass = 0;
        clearMemory(pdata);

        pdata = psave; /*reset pointer*/
        printf("Check Echo region at %p with = %lu\n\n", pdata, *pdata );
        printf("Check Echo region at %p with = %lu\n\n", pdata+2*BLKLEN-1, *(pdata + 2*BLKLEN-1) );

    }

    printf("starting at %p\n", pdata);
    cksum=0;

    for (i=0; i<_pEnd; i+=4)  {

        *pdata = i;
        if ( i<16 || i== _pEnd) 
            printf("val[%p] = %lu\n", pdata, *pdata );

        cksum += *pdata;
        pdata++;
    }
    *pdata = cksum;

    printf("ending at %p with cksum val= %lu\n\n", pdata, *pdata );

   /* if we're node 0, we kick things off */
   if(nodeId == 0) {
      //errlogPrintf("Initial serve...\n");
      /* tell other system that new data is available */
      rmIntSend(INT2, -1);
   }

   while(1) {
      if (stop) epicsThreadSuspendSelf();

      //errlogPrintf("Waiting ...");
      /* wait for interrupt */
      epicsEventMustWait(intFlag);

      pdata = psave; /*reset pointer*/
      //errlogPrintf("Got it!\n");
      //errlogPrintf("Check Echo region at %p with = %lu\n\n", pData, *pData );
      //
      /* wait another 2 seconds */
      epicsThreadSleep(1.0); 
      
      /* increment RM data storage */

      cksum=0;

      for (i=0; i<_pEnd; i+=4)  {

          *pdata = i;
          cksum += *pdata;
          if ( i<16 || i==sizeof(pdata)*(BLKLEN-2)) 
              printf("val[%p] = %lu\n", pdata, *pdata );
          pdata++;
      }
      *pdata = cksum;

      printf("ending at %p with cksum val= %lu\n\n", pdata, *pdata );

      //*pData = rand();
      /* tell other system that new data is available */
      rmIntSend(INT2, -1);
      /*printf("Pong! (data = %ld)\n", *pData);*/
   }
}

void prep(int irq) {
   static int rmConnected = 0;
   long status =0;

   if (!rmConnected) {
       if ((status = rmIntConnect(irq, pingpongISR)) != OK) {
           perror("rmIntConnect(INT2, pingpong)");
           errlogPrintf("rmIntConnect fail with %ld. Pingpong task not started\n", status);
       }
       rmConnected = -1; /* Success. Store the state.*/
   } else {
      rmIntDisconnect(irq); 
      rmConnected = 0; 
   }

}

void ppStart(void) {

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

