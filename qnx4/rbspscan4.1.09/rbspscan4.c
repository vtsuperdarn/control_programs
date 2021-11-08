/* rbspscan4.c
   ============
   Kevin T. Sterne
   This code is largely based off of the themisscan.c RCP.  The credits for this go to:

   Author: Dieter Andre
*/

/*
 Copyright 2004 The Johns Hopkins University/Applied Physics Laboratory.
 All rights reserved.
 
 This material may be used, modified, or reproduced by or for the U.S.
 Government pursuant to the license rights granted under the clauses at DFARS
 252.227-7013/7014.
 
 For any other permissions, please contact the Space Department
 Program Office at JHU/APL.
 
 This Distribution and Disclaimer Statement must be included in all copies of
 "Radar Operating System - Control Programs" (hereinafter "the Program").
 
 The Program was developed at The Johns Hopkins University/Applied Physics
 Laboratory (JHU/APL) which is the author thereof under the "work made for
 hire" provisions of the copyright law.  
 
 JHU/APL assumes no obligation to provide support of any kind with regard to
 the Program.  This includes no obligation to provide assistance in using the
 Program or to provide updated versions of the Program.
 
 THE PROGRAM AND ITS DOCUMENTATION ARE PROVIDED AS IS AND WITHOUT ANY EXPRESS
 OR IMPLIED WARRANTIES WHATSOEVER.  ALL WARRANTIES INCLUDING, BUT NOT LIMITED
 TO, PERFORMANCE, MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE ARE
 HEREBY DISCLAIMED.  YOU ASSUME THE ENTIRE RISK AND LIABILITY OF USING THE
 PROGRAM TO INCLUDE USE IN COMPLIANCE WITH ANY THIRD PARTY RIGHTS.  YOU ARE
 ADVISED TO TEST THE PROGRAM THOROUGHLY BEFORE RELYING ON IT.  IN NO EVENT
 SHALL JHU/APL BE LIABLE FOR ANY DAMAGES WHATSOEVER, INCLUDING, WITHOUT
 LIMITATION, ANY LOST PROFITS, LOST SAVINGS OR OTHER INCIDENTAL OR
 CONSEQUENTIAL DAMAGES, ARISING OUT OF THE USE OR INABILITY TO USE THE
 PROGRAM."
 
 
 
 
 
 
 */

#include <i86.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include <process.h>
#include "rtypes.h"
#include "option.h"
#include "rtime.h"
#include "limit.h"
#include "radar.h"
#include "rprm.h"
#include "iqdata.h"
#include "rawdata.h"
#include "fitblk.h"
#include "fitdata.h"
#include "fitacf.h"


#include "taskid.h"
#include "errlog.h"
#include "rmsg.h"
#include "radarshell.h"
#include "rmsgsnd.h"
#include "tsg.h"
#include "tsgtable.h"
#include "maketsg.h"
#include "global.h"
#include "setup.h"
#include "reopen.h"
#include "tmseq.h"
#include "build.h"
#include "sync.h"
#include "interface.h"
#include "hdw.h"

/*
 $Log: rbspscan.c,v $
 Revision 1.09 2012/12/12 20:09:00  kts
 Added triggered cpid change based on schedule line having -trig

 Revision 1.08 2012/10/31 18:15:46  kts
 Changes to beam array setting, placing a cap on 16 beam radars to only 
 do at most 28 soundings, as well as cleaning up unnecssary code. 

 Revision 1.07 2012/10/19 18:24:37  kts
 Minor change to nintgs calculation to deal with different results from
 different number of beams for the radar.

 Revision 1.06 2012/10/18 20:16:00  kts
 Minor revision to nintgs calculation

 Revision 1.05 2012/10/12 20:38:35  kts
 Modified the skip calculation to use intsc and intus instead of skipsc and
 skipus.  Also buffered the calculation by having it skip ahead by 2 beams.

 Revision 1.04 2012/10/11 18:38:54  kts
 Did away with static beam arrays.  Pulled code developed by Simon Shepherd
 and myself on dynamic beam creation.  Also had beam progression done like
 'Option 2' in which the mini-scan beams do not appear in the field of view
 beams.

 Revision 1.03 2012/09/28 10:38:00  kts                                                                
 Corrected the order of the mini-scan beams as well as changed                                         
 how the tracbm variable is set. 

 Revision 1.02 2012/09/25 21:06:00  kts
 Decreased number of beams, removed themisscan's synchronizing
 functionality after comments from Dieter.

 Revision 1.01 2012/09/25 12:47:12  kts
 Increased number of beams in scanning scheme to 38.

 Revision 1.0  2012/09/21 19:48:58  kts
 Initial version


Below are the themisscan.c revisions which this code was based off of

 Revision 1.3  2008/03/15 11:22:28  code
 Added I&Q sample capture with iqwrite.

 Revision 1.2  2008/03/14 17:51:48  code
 Added support for I&Q sample capture.

 Revision 1.1  2008/03/12 16:48:35  code
 Initial revision


*/

#define SCHEDULER "schedule"
#define ERRLOG "errlog"

#define CONTROL_NAME "control_program"
#define TASK_NAMES "echo_data","iqwrite","raw_write","fit_write","rawacfwrite","fitacfwrite"

char cmdlne[1024];
char progid[80]={"$Id: rbspscan.c,v 1.08 2012/10/31 18:15:46 kts Exp $"};
char progname[256];
struct TaskID *errlog;

char *tasklist[]= { TASK_NAMES, 0};

int arg=0;
struct OptionData opt;

int main(int argc,char *argv[]) {

  int ptab[8] = {0,14,22,24,27,31,42,43};
  int lags[LAG_SIZE][2] = {
    { 0, 0},		/*  0 */
    {42,43},		/*  1 */
    {22,24},		/*  2 */
    {24,27},		/*  3 */
    {27,31},		/*  4 */
    {22,27},		/*  5 */

    {24,31},		/*  7 */
    {14,22},		/*  8 */
    {22,31},		/*  9 */
    {14,24},		/* 10 */
    {31,42},		/* 11 */
    {31,43},		/* 12 */
    {14,27},		/* 13 */
    { 0,14},		/* 14 */
    {27,42},		/* 15 */
    {27,43},		/* 16 */
    {14,31},		/* 17 */
    {24,42},		/* 18 */
    {24,43},		/* 19 */
    {22,42},		/* 20 */
    {22,43},		/* 21 */
    { 0,22},		/* 22 */

    { 0,24},		/* 24 */

    {43,43}};		/* alternate lag-0  */


  char *sname=NULL;
  char *ename=NULL;
  char *sdname={SCHEDULER};
  char *edname={ERRLOG};
  char logtxt[1024];

  int n;
  pid_t sid;
  int exitpoll=0;
 
  int scnsc=120;
  int scnus=0;
  int skip;
  int cnt=0;
  int fixfrq=0;
  int meribm=5;			/* meridional beam */
  int westbm=4;			/* west beam */
  int eastbm=7;			/* east beam */
  int tempF; 			/* forward beam counter */
  int tempB;			/* backward beam counter */
  int nsecond, nthird;	/* used in beam progression code */
  int nintgs;			/* numer of soundings per scan */
  int *forward_beams;	/* array of forward beam progression */
  int *backward_beams;	/* array of backward beam progression */

  int tv;				/* Variables used in skip calculation */
  int bv;
  int iv;

/*  int num_scans= 30;  */
  /* Second within the 2min interval at which this beam is supposed to start */
/*  int scan_times[ 30]=     {   0,  3,  6,  9, 12, 15, 18, 21, 24, 27, 30, 33, 36, 39, 42, 45, 48, 51, 54, 57, 60,
                              63,  66,  69,  72,  75,  78,  81,  84,  87,  90 };  */
  /* beams for forward and backward scanning radars; -1 will be replaced by the selected camping beam */
/*  int forward_beams[ 30]=  {   0,  -1,   1,  -1,   2,  -1,   3,  -1,   4,  -1,   5,  -1,   6,  -1,   7,  -1,   8,  -1,   9,  -1,
                              10,  -1,  11,  -1,  12,  -1,  13,  -1,  14,  -1 };
  int backward_beams[ 30]= {  15,  -1,  14,  -1,  13,  -1,  12,  -1,  11,  -1,  10,  -1,   9,  -1,   8,  -1,   7,  -1,   6,  -1,
                               5,  -1,   4,  -1,   3,  -1,   2,  -1,   1,  -1 };
*/
  unsigned char discretion=0;
  unsigned char trig=0;		/* trigger flag variable used to set CT-trig CPID */

  strcpy(cmdlne,argv[0]);
  for (n=1;n<argc;n++) {
    strcat(cmdlne," ");
    strcat(cmdlne,argv[n]);
  } 

  strncpy(combf,progid,80);

  OpsSetupRadar();
  OpsSetupShell();

  RadarShellParse(&rstable,"sbm l ebm l dfrq l nfrq l dfrang l nfrang l dmpinc l nmpinc l frqrng l xcnt l",
                  &sbm, &ebm, &dfrq, &nfrq, &dfrang, &nfrang, &dmpinc, &nmpinc, &frqrng, &xcnt);


  SiteStart();

  cp=200;				/* Unofficial CPID for rbspscan -KTS 26Oct2012 */

  intsc= 3;             /* Int time of 3.55 sec was better for GBR radar 04Nov2013 */
  intus= 550000;        /* int time of 2.9 sec and the scan finsihes in 1 min 20 sec. */
  mppul=8;
  mplgs=23;
  nmpinc=2100;
  dmpinc=2100;
  nrang=100;
  rsep=45;		

  /* ========= PROCESS COMMAND LINE ARGUMENTS ============= */


  OptionAdd(&opt,"di",'x',&discretion);

  OptionAdd(&opt,"el",'t',&ename);
  OptionAdd(&opt,"sc",'t',&sname);

  OptionAdd(&opt,"frang",'i',&frang);
  OptionAdd(&opt,"rsep",'i',&rsep);

  OptionAdd( &opt, "dt", 'i', &day);
  OptionAdd( &opt, "nt", 'i', &night);
  OptionAdd( &opt, "df", 'i', &dfrq);
  OptionAdd( &opt, "nf", 'i', &nfrq);
  OptionAdd( &opt, "xcf", 'i', &xcnt);
 
  /* Transmit at this frequency */
  OptionAdd( &opt, "fixfrq", 'i', &fixfrq);

  OptionAdd( &opt, "meribm",'i', &meribm);		/* Meridional beam */
  OptionAdd( &opt, "westbm",'i', &westbm);		/* West beam */
  OptionAdd( &opt, "eastbm",'i', &eastbm);		/* East beam */
  OptionAdd( &opt, "trig",	'x', &trig);		/* Sets the triggered cpid below */

  arg= OptionProcess( 1, argc, argv, &opt, NULL);

  /* Figure out the number of soundings that can be done within the scnsc and scnus time */
  /* Add 0.1 seconds to integration time for loop overhead. */
  nintgs = floor((scnsc+scnus*1e-6)/(intsc+intus*1e-6 +0.1));

  /* Makes sure there is an equal number of miniscan beams */
  /* If running a 16 beam radar, the code below wants nintgs to be 2 smaller */
  if ((fabs(sbm-ebm)) == 15) {
	nintgs = nintgs - nintgs%6 - 2;
	if (nintgs > 28){
		nintgs = 28;   /*Cap the number of soundings to 28 -KTS 31Oct2012 */
	}
  } else { 
	nintgs = nintgs - nintgs%6;
  }

  /* arrays for beam progressions */
  forward_beams = (int *)malloc(nintgs*sizeof(int));
  backward_beams = (int *)malloc(nintgs*sizeof(int));

  /* Logic below always assumes that sbm <= ebm */
  if (backward == 1) {
		n = sbm;
		sbm = ebm;
		ebm = n;
  }

  /* Assume that forward scan radars will use west beam first and backward scan radars
     will use east beam first.  */

  tempB = ebm;
  tempF = sbm;
  for (n=0; n<nintgs; n++){
		nsecond = (n-2)%6;
		nthird  = (n-3)%6;
		/*Every 0th, 6th, 12th, etc sounding is a mini-scan beam */
		if (n%6 == 0) {
			forward_beams[n] = westbm;
			backward_beams[n] = eastbm;
		} else if (nsecond == 0) {
			forward_beams[n] = meribm;
			backward_beams[n] = meribm;
		} else if (nthird == 0) {
			forward_beams[n] = eastbm;
			backward_beams[n] = westbm;
		} else if (tempF<=ebm) {
	
			/* Here is where mini-scn beams are skipped in the FoV scan.  -KTS 10Oct2012 */
		
	/* If tempF is any of the mini-scan beams, then increment it again */
			if (tempF == westbm)
				tempF++;
			if (tempF == meribm)
				tempF++;
			if (tempF == eastbm)
				tempF++;

			forward_beams[n] = tempF;
			tempF++;

	/* If tempB is any of the mini-scan beams, then decrement it again */
			if (tempB == eastbm) 
				tempB--;
			if (tempB == meribm) 
				tempB--;
			if (tempB == westbm)
				tempB--;
			
			backward_beams[n] = tempB;
			tempB--;

		} else {

		/* In case the calculation earlier gets messed up, set the beam number
		   to something useful.  Otherwise leaving the array value unassigned
		   can be bad!!!  -KTS 31Oct2012  */

			backward_beams[n] = 7;
			forward_beams[n] = 7;
		}

  }

 
  if (sname==NULL) sname=sdname;
  if (ename==NULL) ename=edname;

  sid=RShellRegister(sname,CONTROL_NAME);

  errlog=TaskIDMake(ename);  
  OpsLogStart(errlog,progname,argc,argv);

  SiteSetupHardware();

  if ( trig) cp=220;
  if ( discretion) cp= -cp;

  txpl=(rsep*20)/3;

  sprintf(progname,"rbspscan");

  OpsFitACFStart();

  OpsSetupTask(tasklist);
  for (n=0;n<tnum;n++) {
    RMsgSndReset(tlist[n]);
    RMsgSndOpen(tlist[n],strlen(cmdlne),cmdlne);
  }


  /* OpsFindSkip assumes the beam arrays equal the number of
   * beams in the field of view. Rbspscan4's mini-scan beams make
   * the beam arrays larger than the field of view.  Since the function
   * is commented out, the bulk of the OpsFindSkip code has been
   * copied below. -KTS 12Oct2012 */
  /* skip=OpsFindSkip(scnsc,scnus); */

  TimeReadClock(&yr,&mo,&dy,&hr,&mt,&sc,&us);
  iv= intsc*1000000 + intus;
  bv= scnsc* 1000000 + scnus;
  tv=(mt* 60 + sc)* 1000000 + us + iv/2 - 100000;
  skip=floor((tv % bv)/iv)+2;
  if (skip> nintgs-1) {
	skip=nintgs-1;
  }
  if (skip<0) {
	skip=0;
  }


  do {

    if (SiteStartScan()==0) continue;

    if (OpsReOpen(2,0,0) !=0) {
      ErrLog(errlog,progname,"Opening new files.");
      for (n=0;n<tnum;n++) {
        RMsgSndClose(tlist[n]);
        RMsgSndOpen(tlist[n],strlen(cmdlne),cmdlne);     
      }
    }

    scan=1;

    ErrLog(errlog,progname,"Starting scan.");

    if (xcnt>0) {
      cnt++;
      if (cnt==xcnt) {
        xcf=1;
        cnt=0;
      } else xcf=0;
    } else xcf=0;

// sprintf( logtxt, "Skip: %d for %d:%d:%d:%d", skip, hr, mt, sc, us);
// ErrLog( errlog, progname, logtxt);
    if (backward) {
      bmnum= backward_beams[ skip];
    } else {
      bmnum=  forward_beams[ skip];
    }

    do {
// TimeReadClock( &yr, &mo, &dy, &hr, &mt, &sc, &us);
// sprintf( logtxt, "Start of do loop: %d:%d:%d:%d", hr, mt, sc, us);
// ErrLog( errlog, progname, logtxt);



      TimeReadClock(&yr,&mo,&dy,&hr,&mt,&sc,&us);

      if (OpsDayNight()==1) {
        stfrq=dfrq;
        mpinc=dmpinc;
        frang=dfrang;
      } else {
        stfrq=nfrq;
        mpinc=nmpinc;
        frang=nfrang;
      }

      sprintf(logtxt,"Integrating beam:%d intt:%ds.%dus (%d:%d:%d:%d) %d %d",bmnum,intsc,intus,hr,mt,sc,us, skip );
      ErrLog(errlog,progname,logtxt);

      ErrLog(errlog,progname,"Setting beam.");

      SiteSetIntt(intsc,intus);
      SiteSetBeam(bmnum);

      ErrLog(errlog,progname,"Doing clear frequency search."); 
      sprintf(logtxt, "FRQ: %d %d", stfrq, frqrng);
      ErrLog( errlog, progname, logtxt);

      if (SiteFCLR(stfrq,stfrq+frqrng)==FREQ_LOCAL)
        ErrLog(errlog,progname,"Frequency Synthesizer in local mode.");

      if ( (fixfrq > 8000) && (fixfrq < 25000) ) tfreq= fixfrq;
      SiteSetFreq(tfreq);

      sprintf(logtxt,"Transmitting on: %d (Noise=%g)",tfreq,noise);
      ErrLog(errlog,progname,logtxt);

      tsgid=SiteTimeSeq(ptab);

      nave=SiteIntegrate(lags);   
      if (nave<0) {
        sprintf(logtxt,"Integration error:%d",nave);
        ErrLog(errlog,progname,logtxt);
        /* restart the radar */
//        spawnl( P_WAIT, "/home/radar/script/restart.radar", NULL);
        continue;
      }
      sprintf(logtxt,"Number of sequences: %d",nave);
      ErrLog(errlog,progname,logtxt);

      OpsBuildPrm(&prm,ptab,lags);
      OpsBuildIQ(&iq);
      OpsBuildRaw(&raw);

      FitACF(&prm,&raw,&fblk,&fit);
      ErrLog(errlog,progname,"Sending messages."); 

      msg.num=0;
      msg.tsize=0;
      RMsgSndAdd(&msg,sizeof(struct RadarParm),(unsigned char *) &prm, PRM_TYPE,0); 
 
      RMsgSndAdd(&msg,sizeof(struct IQData),(unsigned char *) &iq,
                 IQ_TYPE,0);

      RMsgSndAdd(&msg,strlen(sharedmemory)+1,sharedmemory,
                 IQS_TYPE,0);

      RMsgSndAdd(&msg,sizeof(struct RawData),(unsigned char *) &raw, RAW_TYPE,0);
      RMsgSndAdd(&msg,sizeof(struct FitData),(unsigned char *) &fit, FIT_TYPE,0);
      RMsgSndAdd(&msg,strlen(progname)+1,progname, NME_TYPE,0);
      for (n=0;n<tnum;n++) RMsgSndSend(tlist[n],&msg); 

      ErrLog(errlog,progname,"Polling for exit.");
      exitpoll=RadarShell(sid,&rstable);
      if (exitpoll !=0) break;
      scan=0;
      if (skip == (nintgs-1)) break;
      skip= skip + 1;
      if (backward) {
        bmnum= backward_beams[ skip];
      } else {
        bmnum=  forward_beams[ skip];
      }
    } while (1);
    ErrLog(errlog,progname,"Waiting for scan boundary."); 
  
    if (exitpoll==0) OpsWaitBoundary(scnsc,scnus);

	/* Here skip is the counter for moving through the beam arrays, bbms and fbms.
     * because the initial skip code has moved outside of the loop to keep from
     * errorneously setting the beam, skip much be reset here.  -KTS 16Oct2012  */
    skip=0;


  } while (exitpoll==0);
  SiteEnd();
  for (n=0;n<tnum;n++) RMsgSndClose(tlist[n]);
  ErrLog(errlog,progname,"Ending program.");
  RShellTerminate(sid);
  return 0;   
} 
 
