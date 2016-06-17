#include <string.h>
#include <epicsThread.h>
#include <epicsEvent.h>
#include <iocsh.h>
#include <epicsExport.h>
#include <epicsPrint.h>

#include "vmi5588.h"
#define INT3 3

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



/* pingPong()
 * Main driver task for the communication
 *
 */
void pingPong(void *p) {
   unsigned char nodeId = rmNodeId();
   long volatile *pData = (long*)rmPageMemBase();

   *pData = 0;
   printf("nodeId = %d, pData = %p, data = %ld\n", nodeId, pData, *pData);

   /* if we're node 1, we kick things off */
   if(nodeId == 1) {
      printf("Initial serve...\n");
      /* tell other system that new data is available */
      rmIntSend(INT3, -1);
   }

   while(1) {
      if (stop) epicsThreadSuspendSelf();
      /* wait for interrupt */
      epicsEventMustWait(intFlag);
         printf("Ping....");
         /* wait another 2 seconds */
         epicsThreadSleep(2.0); 
      
      /* increment RM data storage */
      (*pData)++;
      /* tell other system that new data is available */
         rmIntSend(INT3, -1);
      printf("Pong! (data = %ld)\n", *pData);
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
      rmIntConnect(INT3, pingpongISR);
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

static void ppStartRegisterCommands(void)
{
   static int firstTime = 1;
   if (firstTime) {
      iocshRegister(&ppStartFuncDef, ppStartCallFunc);
      iocshRegister(&ppStopFuncDef,  ppStopCallFunc);
      firstTime = 0;
   }
}
epicsExportRegistrar(ppStartRegisterCommands);

