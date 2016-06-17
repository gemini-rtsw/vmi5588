/* $Id: devBiVmi5588.c,v 1.1 2002/02/05 13:16:43 gemvx Exp $ */

/* devBiVmi5588.c - Device Support Routines for VMI5588 status */
/*
*   Author:     Andrew Johnson
*   Date:       10-10-94
*
* Experimental Physics and Industrial Control System (EPICS)
*
* Developed at the Royal Greenwich Observatory for the Gemini
* 8M Telescopes Project.
*
*/

/*
modification history
--------------------

*/

#include    <string.h>
#include    <epicsString.h>
#include    <epicsExport.h>
#include    <recGbl.h>
#include    <errlog.h>
#include    <alarm.h>
#include    <dbDefs.h>
#include    <dbScan.h>
#include    <dbAccess.h>
#include    <recSup.h>
#include    <devSup.h>
#include    <biRecord.h>
#include    "vmi5588.h"


/* Create the dset for devBiVmi5588 */
static long init_record();
static long read_bi();

struct {
    long        number;
    DEVSUPFUN   report;
    DEVSUPFUN   init;
    DEVSUPFUN   init_record;
    DEVSUPFUN   get_ioint_info;
    DEVSUPFUN   read_bi;
}devBiVmi5588={
    5,
    NULL,
    NULL,
    init_record,
    NULL,
    read_bi};
epicsExportAddress(dset, devBiVmi5588);



static long init_record(struct biRecord *prec)
{
    static struct {
    char        *string;
    unsigned long   mask;
    } rmState[] = {
    { "RM_IRQ1",    RM_IRQ1 },
    { "RM_IRQ2",    RM_IRQ2 },
    { "RM_IRQ3",    RM_IRQ3 },
    { "RM_NOSIG",   RM_NOSIG },
    { "RM_NOSYNC",  RM_NOSYNC },
    { "RM_RESYNC",  RM_RESYNC },
    { "RM_NORING",  RM_NORING },
    { "RM_BADXFR",  RM_BADXFR },
    { "RM_TXHALF",  RM_TXHALF },
    { "RM_RXHALF",  RM_RXHALF },
    { NULL,     0 }
    };

    struct instio *pinstio;
    int i;

    /* bi.inp must be an INST_IO */
    switch (prec->inp.type) {
       case (INST_IO) :
          pinstio = (struct instio *)&(prec->inp.value);
          break;
       default :
          recGblRecordError(S_db_badField,(void *)prec,
                "devBiVmi5588: Illegal INP field type");
          return(S_db_badField);
    }

    prec->mask=0;
    for (i=0; rmState[i].string != NULL; i++)
       if (strcmp(pinstio->string, rmState[i].string) == 0)
          prec->mask=rmState[i].mask;

    if (prec->mask == 0) {
       recGblRecordError(S_db_badField,(void *)prec,
           "devBiVmi5588: unrecognised RM Status string");
       return(S_db_badField);
    }
    return(0);
}

static long read_bi(struct biRecord *prec)
{
    struct instio *pinstio;
    long        status;

    
    pinstio = (struct instio *)&(prec->inp.value);
    status = rmStatus(prec->mask) & prec->mask;
    if(status >= 0L && status <= 0xffffL) {
        prec->rval = status;
        return(0);
    } else {
                if(recGblSetSevr(prec,READ_ALARM,INVALID_ALARM) && errVerbose
        && (prec->stat!=READ_ALARM || prec->sevr!=INVALID_ALARM))
            recGblRecordError(-1,(void *)prec,"rmStatus Error");
        return(2);
    }
    return(status);
}

