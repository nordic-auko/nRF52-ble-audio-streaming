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
  decoder.c : BV32 Fixed-Point Decoder Main Subroutines

  $Log$
******************************************************************************/

#include "typedef.h"
#include "bvcommon.h"
#include "bv32cnst.h"
#include "bv32strct.h"
#include "bv32externs.h"
#include "basop32.h"
#include "utility.h"
#include "mathutil.h"

void Reset_BV32_Decoder(struct BV32_Decoder_State *c)
{
   int i;
   W16zero((Word16 *) c, sizeof(struct BV32_Decoder_State)/sizeof(Word16));
   c->lsplast[0] =   3641;
   c->lsplast[1] =   7282;
   c->lsplast[2] =  10923;
   c->lsplast[3] =  14564;
   c->lsplast[4] =  18204;
   c->lsplast[5] =  21845;
   c->lsplast[6] =  25486;
   c->lsplast[7] =  29127;
   c->lgq_last = MinE;        /* -2.5 DQ25 */
   c->prevlg[0] = MinE;
   c->prevlg[1] = MinE;
   c->lmax = MIN_32;
   c->lmin = MAX_32;
   c->lmean = 0x10000000;     /* 8.0 Q25 */
   c->x1 = 0x1b000000;        /* 13.5 Q25 */
   c->level = 0x1b000000;     /* 13.5 Q25 */
   c->prv_exp = 0;
   c->idum  =0;
   c->per   = 0;
   for(i=0; i<LPCO; i++)
      c->atplc[i+1] = 0;
}

void BV32_Decode(
                 struct BV32_Bit_Stream      *bs,
                 struct BV32_Decoder_State   *ds,
                 Word16 *x)
{
   Word32 a0;
   Word16 gainq;              /* Q2 */
   Word16 gain_exp;
   Word16 pp, ssf, ssfo;
   Word16 i;
   Word16 tmp, max, new_exp, dif_exp;
   Word32 lgq[NSF];           /* DQ25 */
   Word16 xq[FRSZ+PFO];
   Word16 ltsym[LTMOFF+FRSZ];
   Word16 qv[FRSZ];
   Word32 qv32[FRSZ];
   Word16 a[LPCO+1];
   Word16 lspq[LPCO];        /* Q15 */
   Word16 cbs[VDIM*CBSZ];     /* Q0 */
   Word16 bq[3];              /* Q15 */
   Word16 memtmp[LPCO]; /* Q0 */
   Word32 bss;
   Word32  E;
   
   ds->cfecount = 0; /* reset frame erasure counter */ 
   
   /* decode spectral information */
   lspdec(lspq,bs->lspidx,ds->lsppm,ds->lsplast);
   lsp2a(lspq,a);
   W16copy(ds->lsplast, lspq, LPCO);
   
   /* copy state memory ltsym[] to local buffer */
   W16copy(ltsym, ds->ltsym, LTMOFF);
   
   /* decode pitch period */
   pp = bs->ppidx + MINPP;
   
   /* decode pitch taps */
   pp3dec(bs->bqidx, bq);
   
   /* loop thru sub-subframes */
   for (ssf=0;ssf<2;ssf++) {
      
      ssfo = ssf*SFSZ;     /* SubFrame Offset */
      
      /* decode gain */
      a0 = gaindec(lgq+ssf,bs->gidx[ssf],ds->lgpm,ds->prevlg,ds->level,
         &ds->nclglim,ds->lctimer);
      
      if(ds->lctimer > 0)
         ds->lctimer = ds->lctimer-1;
      if(ds->nclglim == NCLGLIM_TRAPPED)
         ds->lctimer = LEVEL_CONVERGENCE_TIME;
      
      /* gain normalization */
      gain_exp = sub(norm_l(a0), 2);
      gainq = round(L_shl(a0, gain_exp)); /* Q2 w.r.t gain_exp */
      
      /* Level Estimation */
      estlevel(ds->prevlg[0],&ds->level,&ds->lmax,&ds->lmin,
         &ds->lmean,&ds->x1);
      
      /* normalize (by gainq & gain_exp) the excitation codebook */
      for (i=0;i<(VDIM*CBSZ);i++)         /* Q0 w.r..t. gain_exp */ 
         cbs[i] = mult_r(gainq, cccb[i]);
      
      /* decode the excitation signal */
      excdec_w_LT_synth(qv32+ssfo,ltsym+LTMOFF+ssfo,bs->qvidx+ssf*NVPSSF,
         bq,cbs,pp,gain_exp, &E);
      
      ds->E = E;
      
   }  /* end of subframe loop */
   
   
   /* LPC pre-synthesis to determine optimal shift */
   for(i=0; i<LPCO; i++)
      memtmp[i] = shr(ds->stsym[i], ds->prv_exp);
   apfilterQ1_Q0(a, LPCO, ltsym+LTMOFF, xq+PFO, FRSZ, memtmp, 0);
   
   /* find exponent */
   max = abs_s(xq[PFO]);
   for(i=1; i<FRSZ; i++){
      tmp = abs_s(xq[PFO+i]);
      if(sub(tmp, max) > 0)
         max = tmp;
   }
   
   /* leave head-room of 1 bit */
   if(max == 0){
      new_exp = 15;
   }
   else{
      new_exp = sub(norm_s(max), 1);
      if(new_exp < 0)
         new_exp = 0;
   }
   
   /* shift of memory relative to previous exponent */ 
   dif_exp = sub(ds->prv_exp, new_exp);
   
   /* adapt the exponent s.t. the scaled instance memory does not overflow */
   do {
      Overflow = 0; /* reset the overflow flag */
      for(i=0; i<LPCO; i++) shr(ds->stsym[i], dif_exp);
      shr(ds->depfm[0], dif_exp);
      shr(ds->dezfm[0], dif_exp);
      if (Overflow) {
         dif_exp++;
         new_exp--;
      }
   } while (Overflow);  
   
   /* shift short-term memory */
   for(i=0; i<LPCO; i++)
      ds->stsym[i] = shr(ds->stsym[i], dif_exp);
   
   /* shift de-emphasis memory */
   ds->depfm[0] = shr(ds->depfm[0], dif_exp);
   ds->dezfm[0] = shr(ds->dezfm[0], dif_exp);
   
   /* normalize 32-bit excitation and round to 16-bit */
   for(i=0; i<FRSZ; i++)
      qv[i] = round(L_shl(qv32[i], new_exp)); /* Q0 normalized by new_exp */
   
   /* LPC synthesis */
   apfilter(a, LPCO, qv, xq+PFO, FRSZ, ds->stsym, 1);
   
   /* update pitch period of last frame */
   ds->pp_last = pp;
   
   /* update signal memory */
   W16copy(ds->ltsym, ltsym+FRSZ, LTMOFF);
   W16copy(ds->bq_last, bq, 3);
   
   /* update average quantized log-gain */
   ds->lgq_last = L_shr(L_add(lgq[0],lgq[1]),1);
   
   /* DE-EMPHASIS FILTERING */
   apfilter(b_pre,PFO,xq+PFO,xq+PFO,FRSZ,ds->depfm,1);
   W16copy(xq,ds->dezfm,PFO);
   W16copy(ds->dezfm,xq+FRSZ,PFO);
   azfilter(a_pre,PFO,xq+PFO,x,FRSZ);
   
   /* shift output back */
   for(i=0; i<FRSZ; i++)
      x[i] = round(L_shr(L_deposit_h(x[i]),new_exp));
   
   ds->prv_exp = new_exp;
   
   W16copy(ds->atplc, a, LPCO+1);
   bss = L_add(L_add(bq[0], bq[1]), bq[2]);
   if (bss > 32768)
      bss = 32768;
   else if (bss < 0)
      bss = 0;
   ds->per = add(shr(ds->per, 1), (Word16)L_shr(bss, 1));
   
}
