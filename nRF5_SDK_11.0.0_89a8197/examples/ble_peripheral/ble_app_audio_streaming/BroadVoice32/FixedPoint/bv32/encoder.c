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
  encoder.c : BV32 Fixed-Point Encoder Main Subroutines

  $Log$
******************************************************************************/

#include "typedef.h"
#include "bvcommon.h"
#include "bv32cnst.h"
#include "bv32strct.h"
#include "bv32externs.h"
#include "basop32.h"
#include "utility.h"

void Reset_BV32_Encoder(struct BV32_Encoder_State *c)
{
   W16zero((Word16 *) c, sizeof(struct BV32_Encoder_State)/sizeof(Word16));
   c->lsplast[0] =   3641;
   c->lsplast[1] =   7282;
   c->lsplast[2] =  10923;
   c->lsplast[3] =  14564;
   c->lsplast[4] =  18204;
   c->lsplast[5] =  21845;
   c->lsplast[6] =  25486;
   c->lsplast[7] =  29127;
   c->old_a[0] = 4096;		/* 1.0 Q12 */ 
   c->xwd_exp = 31;
   c->cpplast = 12*DECF;
   c->prevlg[0] = MinE;
   c->prevlg[1] = MinE;
   c->lmax = MIN_32;
   c->lmin = MAX_32;
   c->lmean = 0x10000000;     /* 8.0 Q25 */
   c->x1 = 0x1b000000;        /* 13.5 Q25 */
   c->level = 0x1b000000;     /* 13.5 Q25 */
}

void BV32_Encode(
                 struct 	BV32_Bit_Stream	*bs,
                 struct	BV32_Encoder_State	*cs,
                 Word16	*inx)	
{
   Word16 ltsym[MAXPP1+FRSZ];
   Word16 ltnfm[MAXPP1+FRSZ];
   Word32 r[LPCO+1];
   Word16 a[LPCO+1];
   Word16 aw[LPCO+1];
   Word16 x[LX];			/* Q0 signal buffer */
   Word16 dq[LX];		/* Q0 quantized short term pred error  */
   Word16 sdq[LX];
   Word16 xw[FRSZ];		/* Q0 perceptually weighted version of x() */
   Word16 lsp[LPCO], lspq[LPCO];					/* Q15 */
   Word16 cbs[VDIM*CBSZ];
   Word16 bq[3];		/* Q15 */
   Word16 beta;		/* Q13 */	
   Word16 ppt;		/* Q9 */
   Word16 pp, cpp;
   Word16 	i, ssf, ssfo;
   Word32	a0; 
   Word16	gainq;	/* Q3 */   
   Word32	ee;		/* Q3 */ 
   Word16	gain_exp;
   
   /* copy state memory to local memory buffers */
   W16copy(x, cs->x, XOFF);
   W16copy(ltnfm, cs->ltnfm, MAXPP1);
   W16copy(ltsym, cs->ltsym, MAXPP1);
   
   /* highpass filtering & pre-emphasis filtering */
   preprocess(cs,x+XOFF,inx,FRSZ);
   
   /* copy to coder state */
   W16copy(cs->x,x+FRSZ,XOFF);
   
   /* perform lpc analysis with asymmetrical window */
   Autocorr(r, x+LX-WINSZ, winl, WINSZ, LPCO);
   Spectral_Smoothing(LPCO, r, sstwinl_h, sstwinl_l);
   Levinson(r, a, cs->old_a, LPCO);
   
   for (i=1;i<=LPCO;i++) 
      a[i] = mult_r(bwel[i],a[i]);
   
   a2lsp(a,lsp,cs->lsplast);
   W16copy(cs->lsplast,lsp,LPCO);
   
   lspquan(lspq,bs->lspidx,lsp,cs->lsppm);
   
   lsp2a(lspq,a);
   
   /* calculate lpc prediction residual */
   W16copy(dq,cs->dq,XOFF);
   azfilterQ0_Q1(a,LPCO,x+XOFF,dq+XOFF,FRSZ);
   
   /* use weighted version of lpc filter as noise feedback filter */
   
   aw[0] = a[0];
   for (i=1;i<=LPCO; i++) aw[i] = mult_r(STWAL[i],a[i]);
   
   /* get perceptually weighted version of speech */
   for (i=0;i<FRSZ;i++) xw[i] = shr(dq[XOFF+i], 2);
   apfilter(aw, LPCO, xw, xw, FRSZ, cs->stwpm, 1); 
   
   /* get the coarse version of pitch period using 8:1 decimation */
   cpp = coarsepitch(xw, cs);
   cs->cpplast=cpp;
   
   /* refine the pitch period in the neighborhood of coarse pitch period
   also calculate the pitch predictor tap for single-tap predictor */
   
   for (i=0;i<LX;i++) sdq[i] = shr(dq[i],3);
   pp = refinepitch(sdq, cpp, &ppt);
   bs->ppidx = pp - MINPP;
   
   /* vq 3 pitch predictor taps with minimum residual energy */
   bs->bqidx=pitchtapquan(dq, pp, bq);
   
   /* get coefficients for long-term noise feedback filter */
   if (ppt > 512) beta = LTWFL;
   else if (ppt <= 0) beta = 0;
   else beta = extract_h(L_shl(L_mult(LTWFL, ppt),6)); 
   
   /* Loop over excitation sub-frames */
   for (ssf=0;ssf<NSF;ssf++) {
      
      ssfo = ssf*SFSZ;		/* SUB-SUBFRAME OFFSET */
      
      /* calculate pitch prediction residual energy */
      ee = residual_energy(dq+ssfo, pp, bq);
      
      /* log-gain quantization within each sub-frame */
      bs->gidx[ssf] = gainquan(&a0,ee,cs->lgpm,cs->prevlg,cs->level);
      
      /* find appropriate gain block floating point exponent */
      gain_exp =  sub(norm_l(a0), 2);
      gainq  = round(L_shl(a0, gain_exp));
      
      /* Level Estimation */
      estlevel(cs->prevlg[0],&cs->level,&cs->lmax,&cs->lmin,
         &cs->lmean,&cs->x1);
      
      /* scale the excitation codebook */
      for (i=0;i<(VDIM*CBSZ);i++) cbs[i] = mult_r(gainq, cccb[i]);
      
      /* perform noise feedback coding of the excitation signal */
      excquan(bs->qvidx+ssf*NVPSSF,dq+XOFF+ssfo,aw,bq,beta,ltsym+ssfo,
         ltnfm+ssfo,cs->stnfm,cbs,pp,gain_exp);   
      
   }	/* end of sub-subframe loop */ 
   
   W16copy(cs->dq,dq+FRSZ,XOFF);
   
   /* update long-term predictor memory after processing current frame */
   W16copy(cs->ltsym,ltsym+FRSZ,MAXPP1);
   W16copy(cs->ltnfm,ltnfm+FRSZ,MAXPP1);
   
}
