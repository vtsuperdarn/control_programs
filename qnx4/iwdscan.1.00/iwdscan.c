/*
 * iwdscan.c ============ Author: KKrieger
 */

#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <string.h>
#include <time.h>
#include <process.h>
#include <unistd.h>
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
#include "freq.h"
/*
 * $Log: iwdscan.c,v $ 
 * Revision 1.00 2016/09/15 21:00:00 KKrieger
 * Initial revision. Based off epopsound.1.02
 * 
 */

#define SCHEDULER "schedule"
#define ERRLOG "errlog"

#define CONTROL_NAME "control_program"
#define TASK_NAMES "echo_data","iqwrite","rawacfwrite","fitacfwrite"

char cmdlne[1024];
char progid[80] = { "$Id: iwdscan.c,v 1.00 2016/09/15 21:00:00 KKrieger Exp $" };
char progname[256];

struct TaskID *errlog;

char *tasklist[] = { TASK_NAMES,
	0
};

int arg = 0;
struct OptionData opt;

int main(int argc, char *argv[])
{
	/* Option to have a 'marker' pulse sequence every x beams.
	We will use the 7 pulse sequence for this and have
	command line flags to enable and configure the period of
	this marker pulse sequence */
	int ptab_7[7] = { 0, 9, 12, 20, 22, 26, 27};

	int lags_7[LAG_SIZE][2] = {
		{0,0},
		{26,27},
		{20,22},
		{9,12},
		{22,26},
		{22,27},
		{20,26},
		{20,27},
		{12,20},
		{0,9},
		{12,22},
		{9,20},
		{0,12},
		{9,22},
		{12,26},
		{12,27},
		
		{9,26},
		{9,27},

		{27,27}};

	int mppul_7 = 7;
	int mplgs_7 = 18;
	int mpinc_7 = 2400;
	int dmpinc_7 = 2400;
	int nmpinc_7 = 2400;
	int use_marker = 0; /* Default do not use marker */
	int marker_period = 15; /* Default every 15 beams marker period */
	int marker_counter = marker_period; /* Counter for the marker periods */

	int ptab_8[8] = { 0, 14, 22, 24, 27, 31, 42, 43 };

	int lags_8[LAG_SIZE][2] = {
		{0, 0},		/* 0 */
		{42, 43},	/* 1 */
		{22, 24},	/* 2 */
		{24, 27},	/* 3 */
		{27, 31},	/* 4 */
		{22, 27},	/* 5 */

		{24, 31},	/* 7 */
		{14, 22},	/* 8 */
		{22, 31},	/* 9 */
		{14, 24},	/* 10 */
		{31, 42},	/* 11 */
		{31, 43},	/* 12 */
		{14, 27},	/* 13 */
		{0, 14},	/* 14 */
		{27, 42},	/* 15 */
		{27, 43},	/* 16 */
		{14, 31},	/* 17 */
		{24, 42},	/* 18 */
		{24, 43},	/* 19 */
		{22, 42},	/* 20 */
		{22, 43},	/* 21 */
		{0, 22},	/* 22 */

		{0, 24},	/* 24 */

		{43, 43}
	};			/* alternate lag-0 */

	int mppul_8 = 8;
	int mplgs_8 = 23;
	int mpinc_8 = 1500;
	int dmpinc_8 = 1500;
	int nmpinc_8 = 1500;

	char *sname = NULL;
	char *ename = NULL;
	char *sdname = { SCHEDULER };
	char *edname = { ERRLOG };
	char logtxt[1024];

	int n;
	pid_t sid;
	int exitpoll = 0;

	int scnsc = 10;
	int scnus = 0;
/*	double totalscantime = 0;
	time_t scanstarttime;
	time_t scanstoptime;
	int skip;*/ 
	int cnt = 0;
    int fixfrq = 0;		/* Fixed frequency to transmit on (kHz) */
	int startbeam = 0; /* Optional start and stop beams */ 
	int stopbeam = 15; /*  for default mode camping */


	/* variable for temporary use */
	int temp = 0;

	unsigned char discretion = 0;

	/*
	 * Get the command line arguments 
	 */
	strcpy(cmdlne, argv[0]);
	for (n = 1; n < argc; n++) {
		strcat(cmdlne, " ");
		strcat(cmdlne, argv[n]);
	}
	
	strncpy(combf, progid, 80);
	OpsSetupCommand(argc, argv);
	OpsSetupRadar();
	OpsSetupShell();

	RadarShellParse(&rstable,
			"sbm l ebm l dfrq l nfrq l dfrang l nfrang l dmpinc l nmpinc l frqrng l xcnt l",
			&sbm, &ebm, &dfrq, &nfrq, &dfrang, &nfrang, &dmpinc,
			&nmpinc, &frqrng, &xcnt);

	cp = 3385;		
	intsc = 1;
	intus = 0;		
			 
	mppul = mppul_8;
	mplgs = mplgs_8;
	mpinc = mpinc_8;
	dmpinc = dmpinc_8;
	nmpinc = nmpinc_8;
	nrang = 75;
	rsep = 45;
	txpl = 300;

	SiteStart();

	/*
	 * ========= PROCESS COMMAND LINE ARGUMENTS ============= 
	 */

	OptionAdd(&opt, "di", 'x', &discretion);
	OptionAdd(&opt, "el", 't', &ename);
	OptionAdd(&opt, "sc", 't', &sname);
	OptionAdd(&opt, "rsep", 'i', &rsep);
	OptionAdd(&opt, "xcf", 'i', &xcnt);
    OptionAdd(&opt, "startbeam", 'i', &startbeam);
	OptionAdd(&opt, "stopbeam", 'i', &stopbeam);
	OptionAdd(&opt, "fixfrq", 'i', &fixfrq);
	OptionAdd(&opt, "fixfreq", 'i', &fixfrq); /* different spelling of fixfrq */
	OptionAdd(&opt, "use_marker", 'i',&use_marker);
	OptionAdd(&opt, "marker_period",'i',&marker_period);
	OptionAdd(&opt, "dayfreq", 'i', &dfrq);
	OptionAdd(&opt, "nightfreq", 'i', &nfrq);
	OptionAdd(&opt, "dayfrq", 'i', &dfrq); /* Different spelling */
	OptionAdd(&opt, "nightfrq", 'i', &nfrq); /* Different spelling */
	OptionAdd(&opt, "df", 'i', &dfrq); /* Different spelling */
	OptionAdd(&opt, "nf", 'i', &nfrq); /* Different spelling */


    arg = OptionProcess(1, argc, argv, &opt, NULL);

	/* Error checking on use_marker and marker_period */
	if(use_marker != 0) {
        use_marker = 1;
    }
	if(marker_period < 0) {
        marker_period = 0;
    }
	/* Set marker counter to the period so we send marker
       on the first beam */
	marker_counter = marker_period;
	
	/* Error checking on start and stop beams */
	if (startbeam < 0) {
        startbeam = 0;
    }
	if (stopbeam > 15) {
        stopbeam = 15;
    }
	if (startbeam > stopbeam) {
		temp = stopbeam;
		stopbeam = startbeam;
		startbeam = temp;
	}
	if(backward) {
        bmnum = stopbeam;
    } else {
        bmnum = startbeam; 
    }
	
	if (sname == NULL) {
		sname = sdname;
    }
	if (ename == NULL) {
		ename = edname;
    }

	sid = RShellRegister(sname, CONTROL_NAME);

	errlog = TaskIDMake(ename);
	OpsLogStart(errlog, progname, argc, argv);
	sprintf(logtxt, "startbeam: %d, stopbeam: %d", startbeam, stopbeam);
	ErrLog(errlog,progname,logtxt);	
	if(use_marker) {
		sprintf(logtxt, "Using marker sequence. Every %d beams",marker_period);
		ErrLog(errlog,progname,logtxt);
	}
	SiteSetupHardware();

	if (discretion) {
		cp = -cp;
    }
	txpl = (rsep * 20) / 3;

    /* If fixfrq is 0 that means we haven't recieved fixed frq on cmd line, 
     * so we'll use dfrq and nfrq */
	if(fixfrq==0) {
        sprintf(progname, "iwdscan");
    } else {
        sprintf(progname, "iwdscan (fixfrq)");
    }

	OpsFitACFStart();

	OpsSetupTask(tasklist);
	for (n = 0; n < tnum; n++) {
		RMsgSndReset(tlist[n]);
		RMsgSndOpen(tlist[n], strlen(cmdlne), cmdlne);
	}

/*	TimeReadClock(&yr,&mo,&dy,&hr,&mt,&sc,&us);*/
	do {/* while(exitpoll == 0) */

		/*scanstarttime = time(NULL);*/

		/*
		 * Currently SiteStartScan() just returns 1 
		 */
		if (SiteStartScan() == 0) {
			continue;
        }

		/*
		 * OpsReOpen(int hbnd, int mtbnd, int scbnd) will return 1 when
		 * it is first called, and at every hbdn*3600+60*mtbnd+scbnd
		 * seconds of the day. It opens new files (ex: rawacf) 
		 */
		if (OpsReOpen(2, 0, 0) != 0) {
			ErrLog(errlog, progname, "Opening new files.");
			for (n = 0; n < tnum; n++) {
				RMsgSndClose(tlist[n]);
				RMsgSndOpen(tlist[n], strlen(cmdlne), cmdlne);
			}
		}

		scan = 1;
		ErrLog(errlog, progname, "Starting scan.");

		/*
		 * count up to xcnt on every scan, then set xcf = 1 and reset
		 * counter 
		 */
		if (xcnt > 0) {
			cnt++;
			if (cnt == xcnt) {
				xcf = 1;
				cnt = 0;
			} else
				xcf = 0;
		} else {
 		   	xcf = 0;
		}

		/*skip = OpsFindSkip(scnsc,scnus);
		sprintf(logtxt, "Beam skip: %d", skip);
		ErrLog(errlog,progname,logtxt);
		if (backward) {
			bmnum = stopbeam - skip;
			if(bmnum<startbeam) bmnum = stopbeam;
		} else {
			bmnum = startbeam + skip;
			if (bmnum>stopbeam) bmnum = startbeam;
		}*/
		
        do {
            /*
             * Update the time stored in variables yr, mo, dy, hr, mt, sc, 
             * us 
             */
            TimeReadClock(&yr, &mo, &dy, &hr, &mt, &sc, &us);

            if (OpsDayNight() == 1 && fixfrq == 0) {
               stfrq = dfrq;
            } else {
                stfrq = nfrq;
            }
                
            sprintf(logtxt,"Integrating beam:%d intt:%ds.%dus (%d:%d:%d:%d)", bmnum, intsc, intus, hr, mt, sc, us);
            ErrLog(errlog, progname, logtxt);

            ErrLog(errlog, progname, "Setting beam.");
            SiteSetIntt(intsc, intus);
            SiteSetBeam(bmnum);
            
            /* If we're not in fixed frequency mode, do clear freq search */
            if (fixfrq == 0) {
                if (SiteFCLR(stfrq,stfrq+frqrng)==FREQ_LOCAL) {
                    ErrLog(errlog,progname,"Frequency Synthesizer in local mode.");
                }
            } else {
                /* Now check and set the frequency if we are in fixed freq mode */
                if (FreqTest(ftable, fixfrq) != 0) {
                    /*
                     * If our set frequency is restricted, just set to default 
                     * frequency and print error 
                     */
                    sprintf(logtxt,"ERROR: Frequency of %d is restricted, using default freq.",tfreq);
                    ErrLog(errlog,progname,logtxt);
                    fixfrq = ftable->dfrq;
                }
                tfreq = fixfrq;
            }
            
            SiteSetFreq(tfreq);

            if (fixfrq == 0) {
				sprintf(logtxt, "Transmitting on: %d (Noise=%g)",tfreq,noise);
			} else {
				sprintf(logtxt, "Transmitting on: %d",tfreq);
            }
			ErrLog(errlog, progname, logtxt);

            if(use_marker) {
                ErrLog(errlog,progname, "Using marker pulse sequence");
                if(marker_counter == marker_period) {
                    /* Use the 7 pulse sequence */
                    mppul = mppul_7;
                    mplgs = mplgs_7;
                    mpinc = mpinc_7;
                    tsgid = SiteTimeSeq(ptab_7);
                    nave = SiteIntegrate(lags_7);
                    if(nave < 0) {
                        sprintf(logtxt, "Integration error:%d",nave);
                        ErrLog(errlog,progname,logtxt);
                        spawnl(P_WAIT,"/home/radar/script/restart.radar",NULL);
                        exit(nave);
                    }
                    sprintf(logtxt,"Number of sequences [7]: %d",nave);
                    ErrLog(errlog,progname,logtxt);
                    OpsBuildPrm(&prm,ptab_7,lags_7);
                    marker_counter = 0;
                } else {
                    /* Use the 8 pulse sequence */
                    mppul = mppul_8;
                    mplgs = mplgs_8;
                    mpinc = mpinc_8;
                    tsgid = SiteTimeSeq(ptab_8);
                    nave = SiteIntegrate(lags_8);
                    if(nave < 0) {
                        sprintf(logtxt, "Integration error:%d",nave);
                        ErrLog(errlog,progname,logtxt);
                        spawnl(P_WAIT,"/home/radar/script/restart.radar",NULL);
                        exit(nave);
                    }
                    sprintf(logtxt,"Number of sequences [8]: %d",nave);
                    ErrLog(errlog,progname,logtxt);
                    OpsBuildPrm(&prm,ptab_8,lags_8);
                    marker_counter += 1;
                }
            } else {
                ErrLog(errlog,progname, "Not using marker pulse sequence");
                tsgid = SiteTimeSeq(ptab_8);
                nave = SiteIntegrate(lags_8);
                if (nave < 0) {
                    sprintf(logtxt, "Integration error:%d", nave);
                    ErrLog(errlog, progname, logtxt);
                    spawnl(P_WAIT,"/home/radar/script/restart.radar",NULL);
                    exit(nave);
                }
            
                sprintf(logtxt, "Number of sequences: %d", nave);
                ErrLog(errlog, progname, logtxt);

                OpsBuildPrm(&prm, ptab_8, lags_8);
            }

            OpsBuildIQ(&iq);
            OpsBuildRaw(&raw);

            FitACF(&prm, &raw, &fblk, &fit);
            ErrLog(errlog, progname, "Sending messages.");

            msg.num = 0;
            msg.tsize = 0;
            RMsgSndAdd(&msg, sizeof(struct RadarParm),(unsigned char *)&prm, PRM_TYPE, 0);
            RMsgSndAdd(&msg, sizeof(struct IQData),(unsigned char *)&iq, IQ_TYPE, 0);
            RMsgSndAdd(&msg, strlen(sharedmemory) + 1, sharedmemory,IQS_TYPE, 0);
            RMsgSndAdd(&msg, sizeof(struct RawData),(unsigned char *)&raw, RAW_TYPE, 0);
            RMsgSndAdd(&msg, sizeof(struct FitData),(unsigned char *)&fit, FIT_TYPE, 0);
            RMsgSndAdd(&msg, strlen(progname) + 1, progname,NME_TYPE, 0);
            for (n = 0; n < tnum; n++) RMsgSndSend(tlist[n], &msg);

            ErrLog(errlog, progname, "Polling for exit.");
            exitpoll = RadarShell(sid, &rstable);
            if (exitpoll != 0) break;
            scan = 0;

            /* Set the next beam */
            if (backward) {
                if (bmnum<=startbeam) {
                    bmnum=stopbeam;
					break;
                } else {
                    bmnum--;
                }
            } else {
                if(bmnum>=stopbeam) {
                    bmnum=startbeam;
					break;
                } else {
                    bmnum++;
                } 
            }
        } while(1);
		/*scanstoptime = time(NULL);
		totalscantime = difftime(scanstoptime,scanstarttime);
		if(totalscantime >= scnsc) {
			sprintf(logtxt, "Scan time over limit! %f seconds. Continuing", totalscantime);
			ErrLog(errlog,progname,logtxt);
		} else {
			sprintf(logtxt, "Scan total time: %f seconds", totalscantime);
			ErrLog(errlog,progname,logtxt);
			ErrLog(errlog, progname, "Waiting for scan boundary.");
			if (exitpoll == 0) OpsWaitBoundary(scnsc, scnus);
		}*/
		ErrLog(errlog,progname,"Waiting for scan boundary.");
		if (exitpoll == 0) OpsWaitBoundary(scnsc, scnus);
	} while (exitpoll == 0);
	
	SiteEnd();
	for (n = 0; n < tnum; n++) RMsgSndClose(tlist[n]);
	ErrLog(errlog, progname, "Ending program.");
	RShellTerminate(sid);
	return 0;
}
