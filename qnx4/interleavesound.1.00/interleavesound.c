/* interleavesound.c
   =================
   Author: E.G.Thomas
*/


#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/kernel.h>
#include <string.h>
#include <time.h>
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
 $Log: interleavesound.c,v $
 Revision 1.0  2019/06/14 egthomas
 Initial revision from interleavedscan and normalsound
 
*/

#define UCONT_NAME "ucont_moni"

#define SCHEDULER "schedule"
#define ERRLOG "errlog"

void u_read_uconts(void);

#define CONTROL_NAME "control_program"
#define TASK_NAMES "echo_data","iqwrite","rawacfwrite","fitacfwrite"

char cmdlne[1024];
char progid[80]={"$Id: interleavesound.c,v 1.0 2019/06/14 egthomas Exp $"};
char progname[256];
struct TaskID *errlog;

pid_t uucont_proxy;

char *tasklist[]=
 { TASK_NAMES,
  0};

int arg=0;
struct OptionData opt;

#define NUM_SND_DATA 360
#define SND_NRANG 75
#define SND_NBM 16
#define SND_NFBIN 26
#define MAX_SND_FREQS 12

struct sounder_struct {
  double stime;
  char program_name[40];
  int site_id;
  int beam_num;
  int freq;
  int noise;
  int frange;
  int rsep;
  float pwr[SND_NRANG];
  float vel[SND_NRANG];
  float width[SND_NRANG];
  float AOA[SND_NRANG];
  int gsct[SND_NRANG];
  int qflg[SND_NRANG];
}

int main(int argc,char *argv[]) {

  /* The pulse sequence table and lags for katscan */ 
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

  unsigned char fast=0;
  unsigned char discretion=0;

  /* ---------- Beam sequence for interleavedscan ---------- */
  /*
    forward_beams[]/backward_beams[] are for scan in a clockwise/counterclockwise direction. 
    Please choose one of the num_scans-forward/backward_scans[] sets below 
    according to the beam number of the radar. 
  */

  /* For a 16-beam radar */
  int num_scans = 16;
  int forward_beams[16] = { 0,4,8,12, 2,6,10,14, 1,5,9,13, 3,7,11,15 };
  int backward_beams[16]= { 15,11,7,3, 13,9,5,1, 14,10,6,2, 12,8,4,0 };

  /* For an eastward-looking radar with 20- or more beams (using only 20 beams to complete every 1 min) */ 
  /*
  int num_scans = 20;
  int forward_beams[20] = { 0,4,8,12,16, 2,6,10,14,18, 1,5,9,13,17, 3,7,11,15,19 };
  int backward_beams[20]= { 19,15,11,7,3, 17,13,9,5,1, 18,14,10,6,2, 16,12,8,4,0 };
  */

  /* For an westward-looking radar with 20- or more beams (using only 20 beams to complete every 1 min) */ 

  /* max beam number: 23 (24-beam) */
  /*
  int num_scans = 20;
  int forward_beams[20] = { 4,8,12,16,20, 6,10,14,18,22, 5,9,13,17,21, 7,11,15,19,23 };
  int backward_beams[20]= { 23,19,15,11,7 ,21,17,13,9,5, 22,18,14,10,6, 20,16,12,8,4 };
  */

  /* max beam number: 21 (22-beam) */ 
  /*
  int num_scans = 20;
  int forward_beams[20] = { 2,6,10,14,18, 4,8,12,16,20, 3,7,11,15,19, 5,9,13,17,21 };
  int backward_beams[20]= { 21,17,13,9,5, 19,15,11,7,3, 20,16,12,8,4, 18,14,10,6,2 };
  */

  /* ------------------------------------------------------- */ 
  int bmseqnum = 0;


  /* ---------------- Variables for sounding --------------- */
  char snd_filename[100];
  FILE *snd_dat;
  /* If the file $SD_HDWPATH/sounder.dat exists, the next two parameters are read from it */
  /* the file contains one integer value per line */
  int sounder_freqs_total=8;
  int sounder_freqs[MAX_SND_FREQS] = {11000, 12000, 13000, 14000, 15000, 16000, 17000, 18000, 0, 0, 0, 0};
  int sounder_beams[] = {0,2,4,6,8,10,12,14};
  int sounder_freq_count=0, sounder_beam_count=0;
  int sounder_beams_total=8, odd_beams=0;
  int sounder_freq;
  int sounder_beam_loop=1;
  int normal_intt=6;
  int fast_intt=3;
  int sounder_intt=2;
  float sounder_time, time_needed=1.25;
  struct sounder_struct *sounder_data;
  int act_snd_rec=0;

  sprintf(snd_filename, "%s/sounder.dat", getenv("SD_HDWPATH"));
  snd_dat=fopen(snd_filename, "r");
  if (snd_dat != NULL) {
    fscanf(snd_dat, "%d", &sounder_freqs_total);
    if (sounder_freqs_total > 12) sounder_freqs_total=12;
    for (sounder_freq_count=0; sounder_freq_count < sounder_freqs_total; sounder_freq_cnt++)
      fscanf(snd_dat, "%d", &sounder_freqs[sounder_freq_cnt]);
    sounder_freq_count=0;
    fclose(snd_dat);
  }

  sounder_data=(struct sounder_struct *) calloc(sizeof(struct sounder_struct), NUM_SND_DATA);
  /* ------------------------------------------------------- */


  if ((uucont_proxy=SiteInitProxy(UCONT_NAME))==-1) {
    perror("cannot attach proxy");
  }

  strcpy(cmdlne,argv[0]);
  for (n=1;n<argc;n++) {
    strcat(cmdlne," ");
    strcat(cmdlne,argv[n]);
  } 

  strncpy(combf,progid,80);   
  OpsSetupCommand(argc,argv);
  OpsSetupRadar();
  OpsSetupShell();
 
  RadarShellParse(&rstable,"sbm l ebm l dfrq l nfrq l dfrang l nfrang l dmpinc l nmpinc l frqrng l xcnt l",
                  &sbm,&ebm,
                  &dfrq,&nfrq,
                  &dfrang,&nfrang,
                  &dmpinc,&nmpinc,
                  &frqrng,&xcnt);

  // For 2-min normal scan
  cp=195; /* interleavesound */
  intsc=normal_intt;
  intus=0;
  mppul=8;
  mplgs=23;
  mpinc=1500;
  dmpinc=1500;
  nrang=75;
  rsep=45;
  txpl=300; /* recalculated below with rsep */ 
  frang=180;

  SiteStart();

#if 1
  //maxatten=1;	//Chris
  //maxatten=2;	//Sessai
  //maxatten=1;	//Sessai
#endif

  /* ========= PROCESS COMMAND LINE ARGUMENTS ============= */

  OptionAdd(&opt, "el", 't', &ename);
  OptionAdd(&opt, "sc", 't', &sname);

  OptionAdd(&opt, "frang", 'i', &frang);
  OptionAdd(&opt, "rsep", 'i', &rsep);

  OptionAdd(&opt, "di", 'x', &discretion);
  OptionAdd(&opt, "dt", 'i', &day);
  OptionAdd(&opt, "nt", 'i', &night);
  OptionAdd(&opt, "df", 'i', &dfrq);
  OptionAdd(&opt, "nf", 'i', &nfrq);
  OptionAdd(&opt, "xcf", 'i', &xcnt);
  OptionAdd(&opt, "frqrng", 'i', &frqrng);

  OptionAdd(&opt, "fast", 'x', &fast);

  arg=OptionProcess(1,argc,argv,&opt,NULL);

  if (sname==NULL) sname=sdname;
  if (ename==NULL) ename=edname;

  sid=RShellRegister(sname,CONTROL_NAME);

  errlog=TaskIDMake(ename);
  OpsLogStart(errlog,progname,argc,argv);

  SiteSetupHardware();

  /* the parameters are set for fastscan */
  if (fast) {
     cp=197;  /* fast interleavesound */
     scnsc=60;
     scnus=0;
     intsc=fast_intt;
     intus=0;
  }

  // set a negative CPID for discretionary time
  if ( discretion) cp= -cp;

  // recalculate txpl
  txpl=(rsep*20)/3;

  // set the program name
  if (fast) sprintf(progname,"interleavesound (fast)");
  else sprintf(progname,"interleavesound");

  OpsFitACFStart();

  OpsSetupTask(tasklist);
  for (n=0;n<tnum;n++) {
    RMsgSndReset(tlist[n]);
    RMsgSndOpen(tlist[n],strlen(cmdlne),cmdlne);
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

    skip=OpsFindSkip(scnsc,scnus);

    bmseqnum=skip;
    if (backward) {
      bmnum=backward_beams[bmseqnum];
      if (bmnum<ebm) bmnum=sbm;
    } else {
      bmnum=forward_beams[bmseqnum];
      if (bmnum>ebm) bmnum=sbm;
    }

    do {
      TimeReadClock(&yr,&mo,&dy,&hr,&mt,&sc,&us);

      if (OpsDayNight()==1) {
        stfrq=dfrq;
        mpinc=dmpinc;
      } else {
        stfrq=nfrq;
        mpinc=nmpinc;
      }

      sprintf(logtxt,"Integrating beam:%d intt:%ds.%dus (%d:%d:%d:%d)",bmnum,
                      intsc,intus,hr,mt,sc,us);
      ErrLog(errlog,progname,logtxt);

      u_read_uconts();

      ErrLog(errlog,progname,"Setting beam.");

      SiteSetIntt(intsc,intus);
      SiteSetBeam(bmnum);

      ErrLog(errlog,progname,"Doing clear frequency search."); 

      sprintf(logtxt, "FRQ: %d %d", stfrq, frqrng);
      ErrLog( errlog, progname, logtxt);

      if (SiteFCLR(stfrq,stfrq+frqrng)==FREQ_LOCAL)
        ErrLog(errlog,progname,"Frequency Synthesizer in local mode.");

      SiteSetFreq(tfreq);

      sprintf(logtxt,"Transmitting on: %d (Noise=%g)",tfreq,noise);

      ErrLog(errlog,progname,logtxt);

      tsgid=SiteTimeSeq(ptab);

      nave=SiteIntegrate(lags);
      if (nave<0) {
        sprintf(logtxt,"Integration error:%d",nave);
        ErrLog(errlog,progname,logtxt); 
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

      RMsgSndAdd(&msg,sizeof(struct IQData),(unsigned char *) &iq, IQ_TYPE,0);

      RMsgSndAdd(&msg,strlen(sharedmemory)+1,sharedmemory, IQS_TYPE,0);

      RMsgSndAdd(&msg,sizeof(struct RawData),(unsigned char *) &raw, RAW_TYPE,0);

      RMsgSndAdd(&msg,sizeof(struct FitData),(unsigned char *) &fit, FIT_TYPE,0);

      RMsgSndAdd(&msg,strlen(progname)+1,progname, NME_TYPE,0);

      for (n=0;n<tnum;n++) RMsgSndSend(tlist[n],&msg);

      ErrLog(errlog,progname,"Polling for exit.");

      exitpoll=RadarShell(sid,&rstable);
      if (exitpoll !=0) break;
      scan=0;
      if (bmseqnum==(num_scans-1)) break;

      /* Change the beamnum to the next beam */
      bmseqnum++;

      if (backward) {
        bmnum=backward_beams[bmseqnum];
        if (bmnum<ebm) bmnum=sbm;
      } else {
        bmnum=forward_beams[bmseqnum];
        if (bmnum>ebm) bmnum=sbm;
      }

    } while (1);
    ErrLog(errlog,progname,"Waiting for scan boundary.");

    if (exitpoll==0) {
      /* In here comes the sounder code */
      /* set the "sounder mode" scan variable */
      scan=-2;

      /* set the xcf variable to do cross-correlations (AOA) */
      xcf=1;

      /* we have time until the end of the minute to do sounding */
      /* minus a safety factor given in time_needed */
      TimeReadClock(&yr,&mo,&dy,&hr,&mt,&sc,&us);
      sounder_time = 60.0 - ( sc + us/ 1000000.0);

      sounder_beam_loop = ( sounder_time-(float)sounder_intt > time_needed );
      while (sounder_beam_loop) {
        intsc=sounder_intt;

        /* set the beam */
        bmnum=sounder_beams[sounder_beam_count]+odd_beams;

        /* sounder_freq will be an array of frequencies to step through */
        sounder_freq=sounder_freqs[sounder_freq_count];

        /* the scanning code here */
        sprintf(logtxt,"Integrating SND beam:%d intt:%ds.%dus (%d:%d:%d:%d)",bmnum,intsc,intus,hr,mt,sc,us);
        ErrLog(errlog,progname,logtxt);
        ErrLog(errlog,progname,"Setting SND beam.");
        SiteSetIntt(intsc,intus);
        SiteSetBeam(bmnum);
        ErrLog(errlog, progname, "Doing SND clear frequency search."); 
        if (SiteFCLR( sounder_freq, sounder_freq + frqrng)==FREQ_LOCAL)
          ErrLog(errlog,progname,"Frequency Synthesizer in local mode.");
        SiteSetFreq(tfreq);
/*
        sprintf(logtxt,"Transmitting SND on: %d (Noise=%g)",tfreq,noise);
        ErrLog( errlog, progname, logtxt);
*/
        tsgid=SiteTimeSeq(ptab);
        nave=SiteIntegrate(lags);
        if (nave < 0) {
          sprintf(logtxt, "SND integration error: %d", nave);
          ErrLog(errlog,progname, logtxt); 
          continue;
        }
        sprintf(logtxt,"Number of SND sequences: %d",nave);
        ErrLog(errlog,progname,logtxt);

        OpsBuildPrm(&prm,ptab,lags);
        OpsBuildIQ(&iq);
        OpsBuildRaw(&raw);

        FitACF(&prm,&raw,&fblk,&fit);

        ErrLog(errlog, progname, "Sending SND messages.");
        msg.num= 0;
        msg.tsize= 0;
        RMsgSndAdd( &msg, sizeof(struct RadarParm), (unsigned char *) &prm, PRM_TYPE, 0); 

        RMsgSndAdd( &msg,sizeof(struct IQData),(unsigned char *) &iq, IQ_TYPE,0);

        RMsgSndAdd( &msg,strlen(sharedmemory)+1,sharedmemory, IQS_TYPE,0);

        RMsgSndAdd( &msg, sizeof(struct RawData), (unsigned char *) &raw, RAW_TYPE, 0);
        RMsgSndAdd( &msg, sizeof(struct FitData), (unsigned char *) &fit, FIT_TYPE, 0);
        RMsgSndAdd( &msg, strlen(progname)+1, progname, NME_TYPE, 0);

        /* Only send these to echo_data; otherwise they get written to the data files */
        RMsgSndSend(tlist[ 0], &msg);

        sprintf(logtxt, "SBC: %d  SFC: %d\n", sounder_beam_count, sounder_freq_count);
        ErrLog(errlog, progname, logtxt);

        /* save the sounding mode data */
        write_sounding_record_new(progname, &prm, &fit, sounder_data, &act_snd_rec);

        ErrLog(errlog, progname, "Polling SND for exit.");
        exitpoll=RadarShell(sid,&rstable);
        if (exitpoll !=0) break;

        /* check for the end of a beam loop */
        sounder_freq_count++;
        if (sounder_freq_count >= sounder_freqs_total) {
          /* reset the freq counter and increment the beam counter */
          sounder_freq_count=0;
          sounder_beam_count++;
          if (sounder_beam_count>=sounder_beams_total) {
            sounder_beam_count=0;
            if (odd_beams==0)
              odd_beams=1;
            else
              odd_beams=0;
            sounder_freq_count=0;
          }
        }

        /* see if we have enough time for another go round */
        TimeReadClock(&yr, &mo, &dy, &hr, &mt, &sc, &us);
        sounder_time= 60.0 - ( sc + us/ 1000000.0);
        sounder_beam_loop=( sounder_time-(float)sounder_intt > time_needed );
      }

      /* now wait for the next interleavescan */
      intsc=normal_intt;
      if (fast) intsc=fast_intt;
      OpsWaitBoundary(scnsc,scnus);
    }

    bmseqnum=0;

  } while (exitpoll==0);
  SiteEnd();
  for (n=0;n<tnum;n++) RMsgSndClose(tlist[n]);
  ErrLog(errlog,progname,"Ending program.");
  RShellTerminate(sid);
  return 0;
} 

/* Sends a proxy message to the microcontroller monitoring
   task every beam.
*/

void u_read_uconts() {
  if (uucont_proxy != 0) Trigger(uucont_proxy);
}

/********************** function write_sounding_record_new() ************************/
/* changed the data structure */

void write_sounding_record_new(char *progname, struct RadarParm *prm, struct FitData *fit, struct sounder_struct *sounder_data, int *act_snd_rec)
{
  int i;

  struct header_struct {
    long int stime;
    short int site_id;
    short int beam_no;
    short int freq;
    short int noise;
    short int frange;
    short int rsep;
    short int gsct[SND_NRANG];
    short int qflg[SND_NRANG];
    char program_name[40];
  } header;

  struct data_struct {
    short int pwr;
    short int vel;
    short int width;
    short int AOA;
  } data;

  char data_path[100], data_filename[50], filename[80];

  int  good_ranges[SND_NRANG];

  char *snd_dir;
  FILE *out;

  struct sounder_struct *act_snd_data;


  /* set up the data directory */
  /* get the snd data dir */
  snd_dir= getenv("SD_SND_PATH");
  if (snd_dir==NULL)
    sprintf(data_path,"/data/snd/");
  else {
    memcpy(data_path,snd_dir,strlen(snd_dir));
    data_path[strlen(snd_dir)]= '/';
    data_path[strlen(snd_dir) + 1]= 0;
  }

  /* make up the filename */
  /* YYYYMMDDHH */
  sprintf(data_filename, "%04d%02d%02d%02d%s", prm->time.yr, prm->time.mo, prm->time.dy, (prm->time.hr/ 2)* 2, getenv("SD_RADARCODE"));

  /* finally make the filename */
  sprintf(filename, "%s%s.snd", data_path, data_filename);

  /* open the output file */
  out=fopen(filename,"a");
  if(out==NULL) {
    /* crap. might as well go home */
    return;
  }

  /* make the header */
  header.stime= TimeYMDHMSToEpoch(prm->time.yr, prm->time.mo, prm->time.dy, prm->time.hr, prm->time.mt, prm->time.sc);
  header.site_id= prm->stid;
  header.beam_no= prm->bmnum;
  header.freq= prm->tfreq;
  header.noise= prm->noise.mean;
  header.frange= prm->frang;
  header.rsep= prm->rsep;
  memcpy(header.program_name, progname, sizeof(header.program_name));

  /* zero out the gscat and qual bytes */
  for( i=0; i< SND_NRANG; i++ ) {
    header.gsct[i]= fit->rng[i].gsct;
    header.qflg[i]= fit->rng[i].qflg;
    good_ranges[i]= (fit->rng[i].qflg == 1);
  }

  /* write out the header */
  fwrite(&header, sizeof(header), 1, out);

  /* scale the fit data into the char/shorts */
  for( i=0; i< SND_NRANG; i++ ) {
    /* only do the good ranges */
    if (good_ranges[i]) {
      /* do the power */
      data.pwr= fit->rng[i].p_l;
      /* do the velocity */
      data.vel= fit->rng[i].v;
      /* do the AOA */
      data.AOA= fit->elv[i].normal;
      /* do the width */
      data.width= fit->rng[i].w_l;
      /* write out the data structure */
      fwrite(&data, sizeof(data), 1, out);
    }
  }
  fclose(out);

  /* Fill the next sounder data record */
  act_snd_data= sounder_data + *act_snd_rec;
  act_snd_data->stime= TimeYMDHMSToEpoch(prm->time.yr, prm->time.mo, prm->time.dy, prm->time.hr, prm->time.mt, prm->time.sc);
  memcpy(act_snd_data->program_name, progname, sizeof(act_snd_data->program_name));
  act_snd_data->site_id= prm->stid;
  act_snd_data->beam_num= prm->bmnum;
  act_snd_data->freq= prm->tfreq;
  act_snd_data->noise= prm->noise.mean;
  act_snd_data->frange= prm->frang;
  act_snd_data->rsep= prm->rsep;
  for( i=0; i< SND_NRANG; i++ ) {
    act_snd_data->pwr[i]= fit->rng[i].p_l;
    act_snd_data->vel[i]= fit->rng[i].v;
    act_snd_data->width[i]= fit->rng[i].w_l;
    act_snd_data->AOA[i]= fit->elv[i].normal;
    act_snd_data->gsct[i]= fit->rng[i].gsct;
    act_snd_data->qflg[i]= fit->rng[i].qflg;
  }
  *act_snd_rec= *act_snd_rec + 1;
  if (*act_snd_rec >= NUM_SND_DATA) *act_snd_rec= 0;
}
