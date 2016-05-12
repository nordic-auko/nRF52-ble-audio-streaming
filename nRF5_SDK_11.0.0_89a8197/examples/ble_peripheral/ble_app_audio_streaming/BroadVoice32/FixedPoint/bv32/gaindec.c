/*****************************************************************************/
/* BroadVoice(R)32 (BV32) Fixed-Point ANSI-C Source Code                     */
/* Revision Date: October 5, 2012                                            */
/* Version 1.2                                                               */
/*****************************************************************************/

/*****************************************************************************/
/* Copyright 2000-2012 Broadcom Corporation                                  */
/*                                                                           */
/* This software is provided under the GNU Lesser General Public License,    */
/* version 2.1, as published by the Free Software Foundation ("LGPL").       */
/* This program is distributed in the hope that it will be useful, but       */
/* WITHOUT ANY SUPPORT OR WARRANTY; without even the implied warranty of     */
/* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the LGPL for     */
/* more details.  A copy of the LGPL is available at                         */
/* http://www.broadcom.com/licenses/LGPLv2.1.php,                            */
/* or by writing to the Free Software Foundation, Inc.,                      */
/* 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.                 */
/*****************************************************************************/


/*****************************************************************************
  gaindec.c : gain decoding functions

  $Log$
******************************************************************************/

#include "typedef.h"
#include "bvcommon.h"
#include "bv32cnst.h"
#include "bv32strct.h"
#include "bv32externs.h"
#include "basop32.h"
#include "mathutil.h"

Word32 gaindec(			/* Q18 */
               Word32	*lgq,		/* Q25 */
               Word16  gidx,
               Word16  *lgpm,		/* Q11 */
               Word32	*prevlg,
               Word32	level,
               Word16  *nclglim,
               Word16  lctimer)
{
   Word32 	elg, lgq_nh;
   Word16	lg_exp, lg_frac, lgc;
   Word16  i, n, k;
   
   /* CALCULATE ESTIMATED LOG-GAIN */
   elg = L_shr(L_deposit_h(lgmean),1);		/* Q26 */
   for (i = 0; i < LGPORDER; i++) {
      elg = L_mac0(elg, lgp[i],lgpm[i]);	/* Q26 */
   }
   elg = L_shr(elg,1);
   
   /* CALCULATE DECODED LOG-GAIN */
   *lgq = L_add(L_shr(L_deposit_h(lgpecb[gidx]), 2), elg); /* Q25 */
   
   /* next higher gain */
   if(gidx < LGPECBSZ-1){
      lgq_nh = L_add(L_shr(L_deposit_h(lgpecb_nh[gidx]), 2), elg);	/* Q25 */
      if(*lgq < ((Word32)MinE-(Word32)8192) && L_abs(L_sub(lgq_nh,((Word32)MinE-(Word32)8192))) < L_abs(L_sub(*lgq,((Word32)MinE-(Word32)8192)))){ 
         /* To avoid thresholding when the enc Q makes it below the threshold */
         *lgq = (Word32)MinE;
      }
   }
   
   /* LOOK UP FROM lgclimit() TABLE THE MAXIMUM LOG GAIN CHANGE ALLOWED */
   i = shr(sub(shr(extract_h(L_sub(prevlg[0],level)),9),LGLB),1);   /* get column index */
   if (i >= NGB) {
      i = NGB - 1;
   } else if (i < 0) {
      i = 0;
   }
   n = shr(sub(shr(extract_h(L_sub(prevlg[0],prevlg[1])),9),LGCLB),1);
   /* get row index */
   if (n >= NGCB) {
      n = NGCB - 1;
   } else if (n < 0) {
      n = 0;
   }
   i = i * NGCB + n;
   
   /* UPDATE LOG-GAIN PREDICTOR MEMORY */
   for (k = LGPORDER - 1; k > 0; k--) {
      lgpm[k] = lgpm[k-1];
   }
   lgc = extract_h(L_sub(*lgq, prevlg[0]));		/* Q9 */
   
   
   /* CHECK WHETHER DECODED LOG-GAIN EXCEEDS LGCLIMIT */
   if ((lgc > lgclimit[i]) && (gidx > 0) && lctimer == 0) { /* if decoded log-gain exceeds limit */
      *lgq = prevlg[0];   /* use the log-gain of previous frame */
      lgpm[0] = extract_h(L_shl(L_sub(*lgq, elg), 2));
      *nclglim = *nclglim+1;
      if(*nclglim > NCLGLIM_TRAPPED)
         *nclglim = NCLGLIM_TRAPPED;
   } else {
      lgpm[0] = lgpecb[gidx];
      *nclglim = 0;
   }
   
   /* UPDATE LOG-GAIN PREDICTOR MEMORY */
   prevlg[1] = prevlg[0];
   prevlg[0] = *lgq;
   
   /* CONVERT QUANTIZED LOG-GAIN TO LINEAR DOMAIN */
   elg = L_shr(*lgq,10);		/* Q25 -> Q26 (0.5F) --> Q16 */
   L_Extract(elg, &lg_exp, &lg_frac);
   lg_exp = add(lg_exp, 18);			/* output to be Q2 */
   return Pow2(lg_exp, lg_frac);
}

void gainplc(Word32 E, Word16 *lgeqm, Word32 *lgqm)
{
   int k;
   Word16 exponent, fraction, lge;
   Word32 lg, mrlg, elg;
   
   exponent = 1;
   fraction = 0;
   if (E > TMinlgXsfsz)
   {
      Log2( E, &exponent, &fraction);
      lg = L_add(L_shl(L_deposit_h(exponent),9), 
         L_shr(L_deposit_h(fraction),6));    /* Q25 */
      lg = L_sub(lg, 178574274); /* 178574274 = log2(1/SFSZL) Q 25 */
      
   }
   else
      lg = 0;   /* Minlg */
   
   mrlg = L_shr(L_deposit_h(lgmean),2); /* Q25 */
   mrlg = L_sub(lg, mrlg);  /* Q25 */
   
   elg = 0;
   for(k=0; k<GPO; k++)
      elg = L_mac0(elg, lgp[k], lgeqm[k]);  /* Q26 */
   
   elg = L_shr(elg,1);                    /* Q25 */
   
   /* predicted log-gain error */
   lge = round(L_shl(L_sub(mrlg, elg),2));   /* Q11 */
   
   /* update quantizer memory */
   for(k=GPO-1; k>0; k--)
      lgeqm[k] = lgeqm[k-1];
   lgeqm[0] = lge;
   
   /* update quantized log-gain memory */
   lgqm[1] = lgqm[0];
   lgqm[0] = lg;
   
   return;
}
