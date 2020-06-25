/* sndwrite.c
   ========== 
   Author E.G.Thomas
*/


#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <zlib.h>
#include "rtypes.h"
#include "dmap.h"
#include "rprm.h"
#include "fitblk.h"
#include "fitdata.h"



int SndWrite(int fid,struct RadarParm *prm,
             struct FitData *fit) {

  int s;
  struct DataMap *ptr=NULL;

  ptr=DataMapMake();
  if (ptr==NULL) return -1;

  int c,x;
  int32 snum,xnum;

  int16 *slist=NULL;

  char *qflg=NULL;
  char *gflg=NULL;

  float *v=NULL;
  float *v_e=NULL;
  float *p_l=NULL;
  float *w_l=NULL;

  char *x_qflg=NULL;

  float *phi0=NULL;
  float *phi0_e=NULL;
  float *elv=NULL;
  float *elv_low=NULL;
  float *elv_high=NULL;

  float sky_noise;

  DataMapAddScalar(ptr,"cp",DATASHORT,&prm->cp);
  DataMapAddScalar(ptr,"stid",DATASHORT,&prm->stid);
  DataMapAddScalar(ptr,"time.yr",DATASHORT,&prm->time.yr);
  DataMapAddScalar(ptr,"time.mo",DATASHORT,&prm->time.mo);
  DataMapAddScalar(ptr,"time.dy",DATASHORT,&prm->time.dy);
  DataMapAddScalar(ptr,"time.hr",DATASHORT,&prm->time.hr);
  DataMapAddScalar(ptr,"time.mt",DATASHORT,&prm->time.mt);
  DataMapAddScalar(ptr,"time.sc",DATASHORT,&prm->time.sc);
  DataMapAddScalar(ptr,"time.us",DATAINT,&prm->time.us);
  DataMapAddScalar(ptr,"nave",DATASHORT,&prm->nave);
  DataMapAddScalar(ptr,"lagfr",DATASHORT,&prm->lagfr);
  DataMapAddScalar(ptr,"smsep",DATASHORT,&prm->smsep);
  DataMapAddScalar(ptr,"noise.search",DATAFLOAT,&prm->noise.search);
  DataMapAddScalar(ptr,"noise.mean",DATAFLOAT,&prm->noise.mean);

  DataMapAddScalar(ptr,"bmnum",DATASHORT,&prm->bmnum);
  DataMapAddScalar(ptr,"bmazm",DATAFLOAT,&prm->bmazm);

  DataMapAddScalar(ptr,"rxrise",DATASHORT,&prm->rxrise);
  DataMapAddScalar(ptr,"intt.sc",DATASHORT,&prm->intt.sc);
  DataMapAddScalar(ptr,"intt.us",DATAINT,&prm->intt.us);

  DataMapAddScalar(ptr,"nrang",DATASHORT,&prm->nrang);
  DataMapAddScalar(ptr,"frang",DATASHORT,&prm->frang);
  DataMapAddScalar(ptr,"rsep",DATASHORT,&prm->rsep);
  DataMapAddScalar(ptr,"xcf",DATASHORT,&prm->xcf);
  DataMapAddScalar(ptr,"tfreq",DATASHORT,&prm->tfreq);

  sky_noise=fit->noise.skynoise;

  DataMapStoreScalar(ptr,"noise.sky",DATAFLOAT,&sky_noise);

  snum=0;
  for (c=0;c<prm->nrang;c++) {
    if ( (fit->rng[c].qflg==1) || 
         ((fit->xrng !=NULL) && (fit->xrng[c].qflg==1))) snum++;
  }

  if (prm->xcf !=0) xnum=snum;
  else xnum=0;

  if (snum==0) return 0;

  slist=DataMapStoreArray(ptr,"slist",DATASHORT,1,&snum,NULL);

  qflg=DataMapStoreArray(ptr,"qflg",DATACHAR,1,&snum,NULL);
  gflg=DataMapStoreArray(ptr,"gflg",DATACHAR,1,&snum,NULL);

  v=DataMapStoreArray(ptr,"v",DATAFLOAT,1,&snum,NULL);
  v_e=DataMapStoreArray(ptr,"v_e",DATAFLOAT,1,&snum,NULL);
  p_l=DataMapStoreArray(ptr,"p_l",DATAFLOAT,1,&snum,NULL);
  w_l=DataMapStoreArray(ptr,"w_l",DATAFLOAT,1,&snum,NULL);

  if (prm->xcf !=0) {
    x_qflg=DataMapStoreArray(ptr,"x_qflg",DATACHAR,1,&xnum,NULL);

    phi0=DataMapStoreArray(ptr,"phi0",DATAFLOAT,1,&xnum,NULL);
    phi0_e=DataMapStoreArray(ptr,"phi0_e",DATAFLOAT,1,&xnum,NULL);
    elv=DataMapStoreArray(ptr,"elv",DATAFLOAT,1,&xnum,NULL);
    elv_low=DataMapStoreArray(ptr,"elv_low",DATAFLOAT,1,&xnum,NULL);
    elv_high=DataMapStoreArray(ptr,"elv_high",DATAFLOAT,1,&xnum,NULL);
  }

  x=0;

  for (c=0;c<prm->nrang;c++) {
    if ( (fit->rng[c].qflg==1) ||
         ((fit->xrng !=NULL) && (fit->xrng[c].qflg==1))) {
      slist[x]=c;

      qflg[x]=fit->rng[c].qflg;
      gflg[x]=fit->rng[c].gsct;

      p_l[x]=fit->rng[c].p_l;
      v[x]=fit->rng[c].v;
      v_e[x]=fit->rng[c].v_err;
      w_l[x]=fit->rng[c].w_l;

      if (xnum !=0) {
        x_qflg[x]=fit->xrng[c].qflg;

        phi0[x]=fit->xrng[c].phi0;
        phi0_e[x]=fit->xrng[c].phi0_err;
        elv[x]=fit->elv[c].normal;
        elv_low[x]=fit->elv[c].low;
        elv_high[x]=fit->elv[c].high;
      }
      x++;
    }
  }

  if (fid !=-1) s=DataMapWrite(fid,ptr);
  else s=DataMapSize(ptr);

  DataMapFree(ptr);
  return s;

}


int SndFwrite(FILE *fp,struct RadarParm *prm,
              struct FitData *fit) {
  return SndWrite(fileno(fp),prm,fit);
}
