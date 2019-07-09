/* $Id: vmi5588.h,v 1.1 2002/02/05 13:16:43 gemvx Exp $ */
/* VMIC VMIVME5588 device driver C interface 
*
*   Author:     Andrew Johnson
*   Date:       10-10-94
*
* Experimental Physics and Industrial Control System (EPICS)
*
* Developed at the Royal Greenwich Observatory for the Gemini
* 8M Telescopes Project.
*/

/*
modification history
--------------------
$Log: vmi5588.h,v $
Revision 1.1  2002/02/05 13:16:43  gemvx
added vmic files

Revision 1.3  2000/02/08 02:51:04  dayle
replaced tabs with blanks

Revision 1.2  2000/01/18 00:04:10  dayle
Moved constants in scs_const.h to appropriate files and rm'd scs_const.h

Revision 1.1.1.1  1998/10/20 09:15:57  cjm
Delivered and accepted version of SCS

Revision 1.1.1.1  1998/02/26 10:17:06  cjm
Initial creation of SCS application

Revision 1.1.1.1  1997/03/11 23:14:00  goodrich
Gem4

Revision 1.1.1.1  1996/10/21 20:56:25  jwright
Gemini EPICS


*/

#include <dbScan.h>

#ifndef _INCLUDED_VMI5588_H
#define _INCLUDED_VMI5588_H

/* Status bits returned by rmStatus */
#define RM_IRQ1   0x0001    /* Interrupt 1 Pending */
#define RM_IRQ2   0x0002    /* Interrupt 2 Pending */
#define RM_IRQ3   0x0004    /* Interrupt 3 Pending */
#define RM_NOSIG  0x0010    /* No Input Signal */
#define RM_NOSYNC 0x0020    /* Input PLL unsynchronized */
#define RM_RESYNC 0x0040    /* PLL Recently unsynchronized */
#define RM_NORING 0x0400    /* Fibre Ring broken */
#define RM_BADXFR 0x0800    /* Single Transfer Error */
#define RM_TXHALF 0x2000    /* Transmit FIFO Half-full */
#define RM_RXHALF 0x4000    /* Receive FIFO Half-full */

/* Data types for symbol lookup */
#define RM_TYPE_PAGE 0x80
#define RM_TYPE_ALOG 0x01
#define RM_TYPE_LONG 0x02
#define RM_TYPE_STRG 0x03
#define RM_TYPE_ARRY 0x04
#define RM_TYPE_USER 0x10

#define RM_PAGE_SIZE 0x0400 /* 1024 bytes per page */
#define RM_NUM_PAGE  254    /* max number of pages (for 256 kB board) */

#define RM_MAX_ATTEMPTS 10

extern int rmMaxAttempts;

/* structures for use in device support */
struct rm_data {
    unsigned char   rm_type;
    unsigned char   pad1;
    unsigned short  protect1;
    unsigned short  protect2;
    unsigned short  pad2;
    union {
        double          Alog;
        long            Long;
        char            Strg[40];
        struct {
            unsigned long   nelm;
            unsigned short  ftyp;
            unsigned short  pad3;
            char        data[4]; /* Size changes with array */
        } array;
    } value;
};

struct rmpvt {
    short           page;
    short           offset;
    struct rm_data *address;
};


/* Routines available to C applications */
long   rmIntConnect(int irqNumber, void (*proutine)(int));
long   rmIntDisconnect(int irqNumber);
long   rmIntSend(int irqNumber, int nodeId);
long   rmNodeId(void);
unsigned long   rmStatus(long reset);
void * rmPageMemBase(void);
void   vmi5588_reboot(void *p);

/* Routines for use within device support only */
long vmi5588_pageInit(short rmPage);
void vmi5588_pvtInit(struct rmpvt **ppdpvt, short rmPage, short rmOffset,
                     struct rm_data *prmData);
long vmi5588_getIoscanpvt(struct rmpvt *pdpvt, IOSCANPVT *pscanpvt);
long vmi5588_trigger(short rmPage);

#endif /* INCvmi5588h */

