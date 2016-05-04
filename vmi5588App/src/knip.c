
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

LOCAL void      rmTalkCEM(void);
LOCAL void fireInterrupts();


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
