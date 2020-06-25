/* sndwrite.c
   ========== 
   Author: R.J.Barnes
*/

#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>
#include "rtypes.h"
#include "dmap.h"
#include "limit.h"
#include "rprm.h"
#include "fitblk.h"
#include "fitdata.h"

/*
 $Log: sndwrite.c,v $
 Revision 1.0  2020/06/25 egthomas
 Initial revision

*/

int SndWrite(int fid,struct RadarParm *prm,struct FitData *fit) {

  int s,c;
  struct DataMap *data;

  int32 snum,xnum;

  int16 slist[MAX_RANGE];

  char qflg[MAX_RANGE];
  char gflg[MAX_RANGE];

  float v[MAX_RANGE];
  float v_e[MAX_RANGE];
  float p_l[MAX_RANGE];
  float w_l[MAX_RANGE];

  char x_qflg[MAX_RANGE];

  float phi0[MAX_RANGE];
  float phi0_e[MAX_RANGE];
  float elv[MAX_RANGE];
  float elv_low[MAX_RANGE];
  float elv_high[MAX_RANGE];

  float sky_noise;

  data=DataMapMake();

  DataMapAddScalar(data,"cp",DATASHORT,&prm->cp);
  DataMapAddScalar(data,"stid",DATASHORT,&prm->stid);
  DataMapAddScalar(data,"time.yr",DATASHORT,&prm->time.yr);
  DataMapAddScalar(data,"time.mo",DATASHORT,&prm->time.mo);
  DataMapAddScalar(data,"time.dy",DATASHORT,&prm->time.dy);
  DataMapAddScalar(data,"time.hr",DATASHORT,&prm->time.hr);
  DataMapAddScalar(data,"time.mt",DATASHORT,&prm->time.mt);
  DataMapAddScalar(data,"time.sc",DATASHORT,&prm->time.sc);
  DataMapAddScalar(data,"time.us",DATAINT,&prm->time.us);
  DataMapAddScalar(data,"nave",DATASHORT,&prm->nave);
  DataMapAddScalar(data,"lagfr",DATASHORT,&prm->lagfr);
  DataMapAddScalar(data,"smsep",DATASHORT,&prm->smsep);
  DataMapAddScalar(data,"noise.search",DATAFLOAT,&prm->noise.search);
  DataMapAddScalar(data,"noise.mean",DATAFLOAT,&prm->noise.mean);

  DataMapAddScalar(data,"bmnum",DATASHORT,&prm->bmnum);
  DataMapAddScalar(data,"bmazm",DATAFLOAT,&prm->bmazm);

  DataMapAddScalar(data,"rxrise",DATASHORT,&prm->rxrise);
  DataMapAddScalar(data,"intt.sc",DATASHORT,&prm->intt.sc);
  DataMapAddScalar(data,"intt.us",DATAINT,&prm->intt.us);

  DataMapAddScalar(data,"nrang",DATASHORT,&prm->nrang);
  DataMapAddScalar(data,"frang",DATASHORT,&prm->frang);
  DataMapAddScalar(data,"rsep",DATASHORT,&prm->rsep);
  DataMapAddScalar(data,"tfreq",DATASHORT,&prm->tfreq);

  sky_noise=fit->noise.skynoise;
  DataMapAddScalar(data,"noise.sky",DATAFLOAT,&sky_noise);

  snum=0;
  for (c=0;c<prm->nrang;c++) {
    if ((fit->rng[c].qflg==1) || (fit->xrng[c].qflg==1)) {
      slist[snum]=c;

      qflg[snum]=fit->rng[c].qflg;
      gflg[snum]=fit->rng[c].gsct;

      v[snum]=fit->rng[c].v;
      v_e[snum]=fit->rng[c].v_err;
      p_l[snum]=fit->rng[c].p_l;
      w_l[snum]=fit->rng[c].w_l;

      x_qflg[snum]=fit->xrng[c].qflg;

      phi0[snum]=fit->xrng[c].phi0;
      phi0_e[snum]=fit->xrng[c].phi0_err;
      elv[snum]=fit->elv[c].normal;
      elv_low[snum]=fit->elv[c].low;
      elv_high[snum]=fit->elv[c].high;

      snum++;
    }
  }
  if (prm->xcf !=0) xnum=snum;
  else xnum=0;

  if (snum !=0) {
    DataMapAddArray(data,"slist",DATASHORT,1,&snum,slist);

    DataMapAddArray(data,"qflg",DATACHAR,1,&snum,qflg);
    DataMapAddArray(data,"gflg",DATACHAR,1,&snum,gflg);

    DataMapAddArray(data,"v",DATAFLOAT,1,&snum,v);
    DataMapAddArray(data,"v_e",DATAFLOAT,1,&snum,v_e);
    DataMapAddArray(data,"p_l",DATAFLOAT,1,&snum,p_l);
    DataMapAddArray(data,"w_l",DATAFLOAT,1,&snum,w_l);

    if (prm->xcf !=0) {
      DataMapAddArray(data,"x_qflg",DATACHAR,1,&xnum,x_qflg);

      DataMapAddArray(data,"phi0",DATAFLOAT,1,&xnum,phi0);
      DataMapAddArray(data,"phi0_e",DATAFLOAT,1,&xnum,phi0_e);
      DataMapAddArray(data,"elv",DATAFLOAT,1,&xnum,elv);
      DataMapAddArray(data,"elv_low",DATAFLOAT,1,&xnum,elv_low);
      DataMapAddArray(data,"elv_high",DATAFLOAT,1,&xnum,elv_high);
    }
  }

  if (fid !=-1) s=DataMapWrite(fid,data);
  else s=DataMapSize(data);
  DataMapFree(data);
  return s;
}


int SndFwrite(FILE *fp,struct RadarParm *prm,struct FitData *fit) {
  return SndWrite(fileno(fp),prm,fit);
}
