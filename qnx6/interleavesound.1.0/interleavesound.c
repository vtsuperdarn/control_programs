/* interleavesound.c
 ===================
 Author: E.G.Thomas

 */

/*
 license stuff should go here...
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <zlib.h>
#include "rtypes.h"
#include "option.h"
#include "rtime.h"
#include "dmap.h"
#include "limit.h"
#include "radar.h"
#include "rprm.h"
#include "iq.h"
#include "rawdata.h"
#include "fitblk.h"
#include "fitdata.h"
#include "fitacf.h"
#include "errlog.h"
#include "freq.h"
#include "tcpipmsg.h"
#include "rmsg.h"
#include "rmsgsnd.h"
#include "radarshell.h"
#include "build.h"
#include "global.h"
#include "reopen.h"
#include "setup.h"
#include "sync.h"
#include "site.h"
#include "sitebuild.h"
#include "siteglobal.h"

#define SND_NRANG 75
#define MAX_SND_FREQS 12

void write_sounding_record_new(char *progname, struct RadarParm *prm struct FitData *fit);

#define RT_TASK 2

char *ststr=NULL;
char *dfststr="tst";
void *tmpbuf;
size_t tmpsze;
char progid[80]={"interleavesound"};
char progname[256];
int arg=0;
struct OptionData opt;
int baseport=44100;
struct TCPIPMsgHost errlog={"127.0.0.1",44100,-1};
struct TCPIPMsgHost shell={"127.0.0.1",44101,-1};
int tnum=4;      
struct TCPIPMsgHost task[4]={
                              {"127.0.0.1",1,-1}, /* iqwrite */
                              {"127.0.0.1",2,-1}, /* rawacfwrite */
                              {"127.0.0.1",3,-1}, /* fitacfwrite */
                              {"127.0.0.1",4,-1}  /* rtserver */
                            };

void usage(void);
int main(int argc,char *argv[]) {

  int ptab[8] = {0,14,22,24,27,31,42,43};

  int lags[LAG_SIZE][2] = {
    { 0, 0},    /*  0 */
    {42,43},    /*  1 */
    {22,24},    /*  2 */
    {24,27},    /*  3 */
    {27,31},    /*  4 */
    {22,27},    /*  5 */

    {24,31},    /*  7 */
    {14,22},    /*  8 */
    {22,31},    /*  9 */
    {14,24},    /* 10 */
    {31,42},    /* 11 */
    {31,43},    /* 12 */
    {14,27},    /* 13 */
    { 0,14},    /* 14 */
    {27,42},    /* 15 */
    {27,43},    /* 16 */
    {14,31},    /* 17 */
    {24,42},    /* 18 */
    {24,43},    /* 19 */
    {22,42},    /* 20 */
    {22,43},    /* 21 */
    { 0,22},    /* 22 */

    { 0,24},    /* 24 */

    {43,43}};   /* alternate lag-0  */

  char logtxt[1024]="";
  char tempLog[40];

  int exitpoll=0;
  int scannowait=0;

  int scnsc=60;
  int scnus=0;

  int skip;
  int cnt=0;
  int i,n;
  unsigned char discretion=0;
  int status=0;
  int fixfrq=0;

  /* new variables for dynamically creating beam sequences */
  int *bms;           /* scanning beams                                     */
  int intgt[20];      /* start times of each integration period             */
  int nintgs=20;      /* number of integration periods per scan; SGS 1-min  */
  int bufsc=0;        /* a buffer at the end of scan; historically this has */
  int bufus=0;        /*  been set to 3.0s to account for what???           */
  unsigned char hlp=0;

  /*
    beam sequences for 24-beam MSI radars but only using 20 most meridional
      beams; 
   */
  /* count     1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19 20 */
  /*          21 22 23 24 25 26 27 28 29 30 31 32 33 34 35 36 37 38 39 40 */
  int bmse[20] =
             { 0, 4, 8,12,16, 2, 6,10,14,18, 1, 5, 9,13,17, 3, 7,11,15,19};
  int bmsw[20] =
             {23,19,15,11, 7,21,17,13, 9, 5,22,18,14,10, 6,20,16,12, 8, 4};


  /* ---------------- Variables for sounding --------------- */
  char snd_filename[100];
  FILE *snd_dat;
  /* If the file $SD_HDWPATH/sounder.dat exists, the next two parameters are read from it */
  /* the file contains one integer value per line */
  int sounder_freqs_total=8;
  int sounder_freqs[MAX_SND_FREQS]= {11000, 12000, 13000, 14000, 15000, 16000, 17000, 18000, 0, 0, 0, 0 };
  int *sounder_beams;
  int sounder_beamse[]={0,2,4,6,8,10,12,14};        /* beam sequences for 24-beam MSI radars using only */
  int sounder_beamsw[]={22,20,18,16,14,12,10,8};    /*  the 16 most meridional beams */
  int sounder_freq_count=0, sounder_beam_count=0;
  int sounder_beams_total=8, odd_beams=0;
  int sounder_freq;
  int sounder_beam_loop=1;
  int fast_intt_sc=2;
  int fast_intt_us=500000;
  int sounder_intt_sc=1;
  int sounder_intt_us=500000;
  float sounder_time, time_needed=1.25;

  char *snd_dir;
  char data_path[100];
  snd_dir=getenv("SD_SND_PATH");
  if(snd_dir==NULL)
    sprintf(data_path,"/data/ros/snd/");
  else
    memcpy(data_path,snd_dir,strlen(snd_dir));

  sprintf(snd_filename,"%s/sounder.dat", data_path);
  fprintf(stderr,"Checking Sounder File: %s\n",snd_filename);
  snd_dat=fopen(snd_filename, "r");
  if(snd_dat != NULL) {
    fscanf(snd_dat, "%d", &sounder_freqs_total);
    if (sounder_freqs_total > 12) sounder_freqs_total=12;
    for (sounder_freq_count=0; sounder_freq_count < sounder_freqs_total; sounder_freq_count++)
      fscanf(snd_dat, "%d", &sounder_freqs[sounder_freq_count]);
    sounder_freq_count=0;
    fclose(snd_dat);
    fprintf(stderr,"Sounder File: %s read\n",snd_filename);
  } else {
    fprintf(stderr,"Sounder File: %s not found\n",snd_filename);
  }
  /* ------------------------------------------------------- */


  /* standard radar defaults */
  cp     = 197;
  intsc  = fast_intt_sc;
  intus  = fast_intt_us;
  mppul  = 8;
  mplgs  = 23;
  mpinc  = 1500;
  dmpinc = 1500;
  nrang  = 100;
  rsep   = 45;
  txpl   = 300;     /* note: recomputed below */
  dfrq   = 10200;

  /* ========= PROCESS COMMAND LINE ARGUMENTS ============= */
  OptionAdd(&opt,"di",    'x',&discretion);
  OptionAdd(&opt,"frang", 'i',&frang);
  OptionAdd(&opt,"rsep",  'i',&rsep);
  OptionAdd(&opt,"dt",    'i',&day);
  OptionAdd(&opt,"nt",    'i',&night);
  OptionAdd(&opt,"df",    'i',&dfrq);
  OptionAdd(&opt,"nf",    'i',&nfrq);
  OptionAdd(&opt,"xcf",   'x',&xcnt);
  OptionAdd(&opt,"nrang", 'i',&nrang);
  OptionAdd(&opt,"ep",    'i',&errlog.port);
  OptionAdd(&opt,"sp",    'i',&shell.port); 
  OptionAdd(&opt,"bp",    'i',&baseport); 
  OptionAdd(&opt,"stid",  't',&ststr);
  OptionAdd(&opt,"fixfrq",'i',&fixfrq);     /* fix the transmit frequency */
  OptionAdd(&opt,"-help", 'x',&hlp);        /* just dump some parameters */

  /* Process all of the command line options
      Important: need to do this here because we need stid and ststr */
  arg=OptionProcess(1,argc,argv,&opt,NULL);

  /* start time of each integration period */
  for (i=0; i<nintgs; i++)
    intgt[i] = i*(intsc + intus*1e-6);

  /* Point to the beams here */
  if (strcmp(ststr,"cve") == 0) {
    bms = bmse;
    sounder_beams = sounder_beamse;
  } else if (strcmp(ststr,"cvw") == 0) {
    bms = bmsw;
    sounder_beams = sounder_beamsw;
  } else {
    printf("Error: Not intended for station %s\n", ststr);
    return (-1);
  }

  if (hlp) {
    usage();

/*    printf("  start beam: %2d\n", sbm);	*/
/*    printf("  end   beam: %2d\n", ebm);	*/
    printf("\n");
    printf("sqnc  stme  bmno\n");
    for (i=0; i<nintgs; i++) {
      printf(" %2d   %3d    %2d", i, intgt[i], bms[i]);
      printf("\n");
    }

    return (-1);
  }

  /* end of main Dartmouth mods */
  /* not sure if -nrang commandline option works */

  if (ststr==NULL) ststr=dfststr;

  if ((errlog.sock=TCPIPMsgOpen(errlog.host,errlog.port))==-1) {
    fprintf(stderr,"Error connecting to error log.\n");
  }
  if ((shell.sock=TCPIPMsgOpen(shell.host,shell.port))==-1) {
    fprintf(stderr,"Error connecting to shell.\n");
  }

  for (n=0;n<tnum;n++) task[n].port+=baseport;

  /* rst/usr/codebase/superdarn/src.lib/os/ops.1.10/src/setup.c */
  OpsStart(ststr);

  /* rst/usr/codebase/superdarn/src.lib/os/site.1.3/src/build.c */
  /* note that stid is a global variable set in the previous function...
      rst/usr/codebase/superdarn/src.lib/os/ops.1.10/src/global.c */
  status=SiteBuild(stid);

  if (status==-1) {
    fprintf(stderr,"Could not identify station.\n");
    exit(1);
  }

  /* dump beams to log file */
  sprintf(progname,"interleavesound (fast)");
  for (i=0; i<nintgs; i++){
    sprintf(tempLog, "%3d", bms[i]);
    strcat(logtxt, tempLog);	
  }
  ErrLog(errlog.sock,progname,logtxt);

  /* IMPORTANT: sbm and ebm are reset by this function */
  SiteStart();

  /* Reprocess the command line to restore desired parameters */
  arg=OptionProcess(1,argc,argv,&opt,NULL);
  backward = (sbm > ebm) ? 1 : 0;   /* this almost certainly got reset */

  strncpy(combf,progid,80);

  /* rst/usr/codebase/superdarn/src.lib/os/ops.1.10/src */
  OpsSetupCommand(argc,argv);
  OpsSetupShell();

  RadarShellParse(&rstable,"sbm l ebm l dfrq l nfrq l dfrang l nfrang l"
                  " dmpinc l nmpinc l frqrng l xcnt l", &sbm,&ebm, &dfrq,&nfrq,
                  &dfrang,&nfrang, &dmpinc,&nmpinc, &frqrng,&xcnt);

  status=SiteSetupRadar();

  fprintf(stderr,"Status:%d\n",status);

  if (status !=0) {
    ErrLog(errlog.sock,progname,"Error locating hardware.");
    exit (1);
  }

  if (discretion) cp = -cp;

  txpl=(rsep*20)/3;     /* computing TX pulse length */

  OpsLogStart(errlog.sock,progname,argc,argv);
  OpsSetupTask(tnum,task,errlog.sock,progname);

  for (n=0;n<tnum;n++) {
    RMsgSndReset(task[n].sock);
    RMsgSndOpen(task[n].sock,strlen( (char *) command),command);
  }

  OpsFitACFStart();

  tsgid=SiteTimeSeq(ptab);  /* get the timing sequence */

  do {

    if (SiteStartScan() !=0) continue;

    if (OpsReOpen(2,0,0) !=0) {
      ErrLog(errlog.sock,progname,"Opening new files.");
      for (n=0;n<tnum;n++) {
        RMsgSndClose(task[n].sock);
        RMsgSndOpen(task[n].sock,strlen( (char *) command),command);
      }
    }

    scan=1;

    ErrLog(errlog.sock,progname,"Starting scan.");

    if (xcnt>0) {
      cnt++;
      if (cnt==xcnt) {
        xcf=1;
        cnt=0;
      } else xcf=0;
    } else xcf=0;

    skip=OpsFindSkip(scnsc,scnus);

    bmnum = bms[skip];		/* no longer need forward and backward arrays... */

    do {

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

      sprintf(logtxt,"Integrating beam:%d intt:%ds.%dus (%d:%d:%d:%d)",bmnum,
          intsc,intus,hr,mt,sc,us);
      ErrLog(errlog.sock,progname,logtxt);

      ErrLog(errlog.sock,progname,"Starting Integration.");
      SiteStartIntt(intsc,intus);

      ErrLog(errlog.sock,progname,"Doing clear frequency search."); 
      sprintf(logtxt, "FRQ: %d %d", stfrq, frqrng);
      ErrLog(errlog.sock,progname, logtxt);
      tfreq=SiteFCLR(stfrq,stfrq+frqrng);

      if ( (fixfrq > 8000) && (fixfrq < 25000) ) tfreq = fixfrq; 

      sprintf(logtxt,"Transmitting on: %d (Noise=%g)",tfreq,noise);
      ErrLog(errlog.sock,progname,logtxt);
      nave=SiteIntegrate(lags);
      if (nave<0) {
        sprintf(logtxt,"Integration error:%d",nave);
        ErrLog(errlog.sock,progname,logtxt); 
        continue;
      }
      sprintf(logtxt,"Number of sequences: %d",nave);
      ErrLog(errlog.sock,progname,logtxt);

      OpsBuildPrm(prm,ptab,lags);
      OpsBuildIQ(iq,&badtr);
      OpsBuildRaw(raw);

      FitACF(prm,raw,fblk,fit);

      msg.num=0;
      msg.tsize=0;

      tmpbuf=RadarParmFlatten(prm,&tmpsze);
      RMsgSndAdd(&msg,tmpsze,tmpbuf,PRM_TYPE,0);

      tmpbuf=IQFlatten(iq,prm->nave,&tmpsze);
      RMsgSndAdd(&msg,tmpsze,tmpbuf,IQ_TYPE,0);

      RMsgSndAdd(&msg,sizeof(unsigned int)*2*iq->tbadtr,
                 (unsigned char *) badtr,BADTR_TYPE,0);

      RMsgSndAdd(&msg,strlen(sharedmemory)+1,(unsigned char *) sharedmemory,
                 IQS_TYPE,0);

      tmpbuf=RawFlatten(raw,prm->nrang,prm->mplgs,&tmpsze);
      RMsgSndAdd(&msg,tmpsze,tmpbuf,RAW_TYPE,0);

      tmpbuf=FitFlatten(fit,prm->nrang,&tmpsze);
      RMsgSndAdd(&msg,tmpsze,tmpbuf,FIT_TYPE,0);

      RMsgSndAdd(&msg,strlen(progname)+1,(unsigned char *) progname,
                 NME_TYPE,0);

      for (n=0;n<tnum;n++) RMsgSndSend(task[n].sock,&msg);

      for (n=0;n<msg.num;n++) {
        if (msg.data[n].type==PRM_TYPE) free(msg.ptr[n]);
        if (msg.data[n].type==IQ_TYPE) free(msg.ptr[n]);
        if (msg.data[n].type==RAW_TYPE) free(msg.ptr[n]);
        if (msg.data[n].type==FIT_TYPE) free(msg.ptr[n]);
      }

      RadarShell(shell.sock,&rstable);

      if (exitpoll !=0) break;
      scan=0;
      if (skip == (nintgs-1)) break;
      skip++;
      bmnum = bms[skip];

    } while (1);

    ErrLog(errlog.sock,progname,"Waiting for scan boundary.");

    if (exitpoll==0) {
      /* In here comes the sounder code */
      /* set the "sounder mode" scan variable */
      scan=-2;

      /* set the xcf variable to do cross-correlations (AOA) */
      xcf=1;

      /* we have time until the end of the minute to do sounding */
      /* minus a safety factor given in time_needed */
      TimeReadClock(&yr,&mo,&dy,&hr,&mt,&sc,&us);
      sounder_time= 60.0 - ( sc + us/ 1000000.0);

      sounder_beam_loop= ( sounder_time-(float)sounder_intt > time_needed );
      while(sounder_beam_loop) {
        intsc=sounder_intt_sc;
        intus=sounder_intt_us;

        /* set the beam */
        bmnum=sounder_beams[sounder_beam_count]+odd_beams;

        /* sounder_freq will be an array of frequencies to step through */
        sounder_freq=sounder_freqs[sounder_freq_count];

        /* the scanning code is here */
        sprintf(logtxt,"Integrating SND beam:%d intt:%ds.%dus (%d:%d:%d:%d)",bmnum,intsc,intus,hr,mt,sc,us);
        ErrLog(errlog.sock,progname,logtxt);
        ErrLog(errlog.sock,progname,"Setting SND beam.");
        SiteStartIntt(intsc,intus);
        ErrLog(errlog.sock, progname, "Doing SND clear frequency search.");
        sprintf(logtxt, "FRQ: %d %d", sounder_freq, frqrng);
        ErrLog(errlog.sock,progname, logtxt);
        tfreq=SiteFCLR(sounder_freq, sounder_freq + frqrng);
/*
 *           sprintf(logtxt,"Transmitting SND on: %d (Noise=%g)",tfreq,noise);
 *                     ErrLog( errlog.sock, progname, logtxt);
 *                     */
        tsgid=SiteTimeSeq(ptab);
        nave=SiteIntegrate( lags);
        if (nave < 0) {
          sprintf(logtxt, "SND integration error: %d", nave);
          ErrLog(errlog.sock,progname, logtxt);
          continue;
        }
        sprintf(logtxt,"Number of SND sequences: %d",nave);
        ErrLog(errlog.sock,progname,logtxt);

        OpsBuildPrm(prm,ptab,lags);
        OpsBuildIQ(iq,&badtr);
        OpsBuildRaw(raw);
        FitACF(prm,raw,fblk,fit);

        ErrLog(errlog.sock, progname, "Sending SND messages.");
        msg.num= 0;
        msg.tsize= 0;

        tmpbuf=RadarParmFlatten(prm,&tmpsze);
        RMsgSndAdd(&msg,tmpsze,tmpbuf,PRM_TYPE,0);

        tmpbuf=IQFlatten(iq,prm->nave,&tmpsze);
        RMsgSndAdd(&msg,tmpsze,tmpbuf,IQ_TYPE,0);

        RMsgSndAdd(&msg,sizeof(unsigned int)*2*iq->tbadtr,
               (unsigned char *) badtr,BADTR_TYPE,0);

        RMsgSndAdd(&msg,strlen(sharedmemory)+1,
               (unsigned char *) sharedmemory,IQS_TYPE,0);

        tmpbuf=RawFlatten(raw,prm->nrang,prm->mplgs,&tmpsze);
        RMsgSndAdd(&msg,tmpsze,tmpbuf,RAW_TYPE,0); 

        tmpbuf=FitFlatten(fit,prm->nrang,&tmpsze);
        RMsgSndAdd(&msg,tmpsze,tmpbuf,FIT_TYPE,0);

        RMsgSndAdd(&msg,strlen(progname)+1,(unsigned char *) progname,NME_TYPE,0);

        RMsgSndSend(task[RT_TASK].sock,&msg);
        for (n=0;n<msg.num;n++) {
          if (msg.data[n].type==PRM_TYPE) free(msg.ptr[n]);
          if (msg.data[n].type==IQ_TYPE) free(msg.ptr[n]);
          if (msg.data[n].type==RAW_TYPE) free(msg.ptr[n]);
          if (msg.data[n].type==FIT_TYPE) free(msg.ptr[n]);
        }

        sprintf(logtxt, "SBC: %d  SFC: %d\n", sounder_beam_count, sounder_freq_count);
        ErrLog(errlog.sock, progname, logtxt);

        /* save the sounding mode data */
        write_sounding_record_new(progname, prm, fit);

        ErrLog(errlog.sock, progname, "Polling SND for exit.");
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
        sounder_beam_loop= ( sounder_time-(float)sounder_intt > time_needed );
      }

      /* now wait for the next interleavescan */
      intsc=fast_intt_sc;
      intus=fast_intt_us;
      if (scannowait==0) SiteEndScan(scnsc,scnus);
    }

  } while (exitpoll==0);

  for (n=0;n<tnum;n++) RMsgSndClose(task[n].sock);

  ErrLog(errlog.sock,progname,"Ending program.");

  SiteExit(0);

  return 0;
}


void usage(void)
{
    printf("\ninterleavesound [command-line options]\n\n");
    printf("command-line options:\n");
    printf("    -di     : indicates running during discretionary time\n");
    printf(" -frang int : delay to first range (km) [180]\n");
    printf("  -rsep int : range separation (km) [45]\n");
    printf("    -dt int : hour when day freq. is used [site.c]\n");
    printf("    -nt int : hour when night freq. is used [site.c]\n");
    printf("    -df int : daytime frequency (kHz) [site.c]\n");
    printf("    -nf int : nighttime frequency (kHz) [site.c]\n");
    printf("   -xcf     : set for computing XCFs [global.c]\n");
    printf(" -nrang int : number or range gates [limit.h]\n");
    printf("    -ep int : error log port (must be set here for dual radars)\n");
    printf("    -sp int : shell port (must be set here for dual radars)\n");
    printf("    -bp int : base port (must be set here for dual radars)\n");
    printf("  -stid char: radar string (must be set here for dual radars)\n");
    printf("-fixfrq int : transmit on fixed frequency (kHz)\n");
    printf(" --help     : print this message and quit.\n");
    printf("\n");
}

/********************** function write_sounding_record_new() ************************/
/* changed the data structure */

void write_sounding_record_new(char *progname, struct RadarParm *prm, struct FitData *fit)
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
    sprintf( data_path,"/data/ros/snd/");
  else {
    memcpy(data_path,snd_dir,strlen(snd_dir));
    data_path[strlen(snd_dir)]= '/';
    data_path[strlen(snd_dir) + 1]= 0;
  }

  /* make up the filename */
  /* YYYYMMDDHH */
  sprintf( data_filename, "%04d%02d%02d%02d%s", prm->time.yr, prm->time.mo, prm->time.dy, (prm->time.hr/ 2)* 2, ststr);

  /* finally make the filename */
  sprintf( filename, "%s%s.snd", data_path, data_filename);

  /* open the output file */
  fprintf(stderr,"Sound Data File: %s\n",filename);
  out=fopen(filename,"a");
  if (out==NULL) {
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
  fwrite(&header, sizeof( header), 1, out);

  /* scale the fit data into the char/shorts */
  for( i=0; i< SND_NRANG; i++ ) {
    /* only do the good ranges */
    if( good_ranges[i] ) {
      /* do the power */
      data.pwr= fit->rng[i].p_l;
      /* do the velocity */
      data.vel= fit->rng[i].v;
      /* do the AOA */
      data.AOA= fit->elv[i].normal;
      /* do the width */
      data.width= fit->rng[i].w_l;
      /* write out the data structure */
      fwrite( &data, sizeof( data), 1, out);
    }
  }
  fclose(out);
}
