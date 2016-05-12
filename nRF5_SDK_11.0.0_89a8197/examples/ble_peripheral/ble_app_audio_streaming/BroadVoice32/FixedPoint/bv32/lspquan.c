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
  lspquan.c : Lsp quantization based on inter-frame moving-average
              prediction and two-stage VQ.

  $Log$
******************************************************************************/

#include "typedef.h"
#include "bvcommon.h"
#include "bv32cnst.h"
#include "bv32strct.h"
#include "bv32externs.h"
#include "basop32.h"

void vqmse(
           Word16  *xq,
           Word16  *idx,
           Word16  *x,
           Word16  *cb,
           Word16  vdim,
           Word16  cbsz);

void vqwmse(
            Word16 	*xq,
            Word16 	*idx, 
            Word16 	*x,
            Word16 	*w,
            Word16 	*cb,
            Word16 	vdim,
            Word16 	cbsz);

void vqwmse_stbl(
                 Word16  *xq,
                 Word16  *idx,
                 Word16  *x,
                 Word16  *w,
                 Word16	*xa,
                 Word16  *cb,
                 Word16  vdim,
                 Word16  cbsz);

void lspquan(
             Word16  *lspq, 		/* Q15 */ 
             Word16  *lspidx,  
             Word16  *lsp,    	/* Q15 */ 
             Word16  *lsppm)		/* Q15 */
{
   Word32 a0;
   Word16 min_d;
   Word16 *fp1, *fp2;
   Word16 i, k;
   Word16 d[LPCO], w[LPCO];
   Word16 elsp[LPCO], lspe[LPCO]; 
   Word16 lspeq1[LPCO], lspeq2[LPCO];
   Word16 lspa[LPCO];
  
   /* CALCULATE THE WEIGHTS FOR WEIGHTED MEAN-SQUARE ERROR DISTORTION */
   min_d = MAX_16;
   for (i = 0; i < LPCO - 1 ; i++) {
      d[i] = sub(lsp[i+1],lsp[i]);       /* LSP difference vector */
      if (d[i] < min_d) min_d = d[i];
   }
   
   w[0] = div_s(min_d, d[0]);
   for (i = 1; i < LPCO - 1 ; i++) {
      if (d[i] < d[i-1]) 
         w[i] = div_s(min_d, d[i]);
      else
         w[i] = div_s(min_d, d[i-1]);
   }
   w[LPCO-1] = div_s(min_d, d[LPCO-2]);
   
   /* CALCULATE ESTIMATED (MA-PREDICTED) LSP VECTOR */
   fp1 = lspp;		/* Q14 */
   fp2 = lsppm;	/* Q15 */
   for (i = 0; i < LPCO; i++) {
      a0 = 0;
      for (k = 0; k < LSPPORDER; k++) {
         a0 = L_mac(a0, *fp1++, *fp2++);
      }
      elsp[i] = round(L_shl(a0,1));	/* Q15 */
   }
   
   /* SUBTRACT LSP MEAN VALUE & ESTIMATED LSP TO GET PREDICTION ERROR */
   for (i = 0; i < LPCO; i++) {
      lspe[i] = shl(sub(sub(lsp[i],lspmean[i]),elsp[i]),1);	/* Q15 -> Q16 */
   }
   
   /* PERFORM FIRST-STAGE VQ CODEBOOK SEARCH, MSE VQ */
   vqmse(lspeq1,lspidx,lspe,lspecb1,LPCO,LSPECBSZ1);
   
   /* CALCULATE QUANTIZATION ERROR VECTOR OF FIRST-STAGE VQ */
   for (i = 0; i < LPCO; i++) {
      lspe[i] = shl(sub(lspe[i],lspeq1[i]),2);		/* Q16 -> Q18 */
   }
   
   /* PERFORM SECOND-STAGE VQ CODEBOOK SEARCH */
   for (i = 0; i < SVD1; i++)
      lspa[i] = add(add(shr(lspeq1[i],1),elsp[i]),lspmean[i]); 	/* Q15 */
   
   vqwmse_stbl(lspeq2,lspidx+1,lspe,w,lspa,lspecb21,SVD1,LSPECBSZ21);
   
   vqwmse(lspeq2+SVD1,lspidx+2,lspe+SVD1,w+SVD1,lspecb22,SVD2,LSPECBSZ22);
   
   /* GET OVERALL QUANTIZER OUTPUT VECTOR OF THE TWO-STAGE VQ */
   for (i = 0; i < LPCO; i++) {
      lspe[i] = (Word16) L_shr( L_add(
         L_shl(L_deposit_l(lspeq1[i]),3),
         L_shl(L_deposit_l(lspeq2[i]),1) ),4);
   }
   
   /* UPDATE LSP MA PREDICTOR MEMORY */
   i = LPCO * LSPPORDER - 1;
   fp1 = &lsppm[i];
   fp2 = &lsppm[i - 1];
   for (i = LPCO - 1; i >= 0; i--) {
      for (k = LSPPORDER; k > 1; k--) {
         *fp1-- = *fp2--;
      }
      *fp1-- = lspe[i];
      fp2--;
   }
   
   /* CALCULATE QUANTIZED LSP */
   for (i = 0; i < LPCO; i++) {
      lspq[i] = add(add(lspe[i],elsp[i]),lspmean[i]);
   }
   
   /* ENSURE CORRECT ORDERING OF LSP TO GUARANTEE LPC FILTER STABILITY */
   stblz_lsp(lspq,LPCO);
}

/*==========================================================================*/

void vqwmse(
            Word16  *xq,    /* Q18 VQ output vector (quantized version of input vector) */
            Word16  *idx,   /* VQ codebook index for the nearest neighbor */
            Word16  *x,     /* Q18 input vector */
            Word16  *w,     /* weights for weighted Mean-Square Error */
            Word16  *cb,    /* VQ codebook */
            Word16  vdim,   /* vector dimension */
            Word16  cbsz)   /* codebook size (number of codevectors) */
{
   
   Word32 	dmin, d;
   Word16 	*fp1, t, s;
   Word16 	j, k;
   
   fp1 = cb;
   dmin = MAX_32;
   for (j = 0; j < cbsz; j++) {
      d = 0;
      for (k = 0; k < vdim; k++) {
         t = sub(x[k],shr(*fp1++,1));
         s = extract_h(L_mult0(w[k],t));
         d = L_mac0(d, s, t);
      }
      if (d < dmin) {
         dmin = d;
         *idx = j;
      }
   }
   
   j = *idx * vdim;
   for (k = 0; k < vdim; k++) {
      xq[k] = shr(cb[j + k],1);
   }
}  

void vqwmse_stbl(
                 Word16  *xq,    /* Q18 VQ output vector (quantized version of input vector) */
                 Word16  *idx,   /* VQ codebook index for the nearest neighbor */
                 Word16  *x,     /* Q18 input vector */
                 Word16  *w,     /* weights for weighted Mean-Square Error */
                 Word16	*xa,
                 Word16  *cb,    /* Q19 VQ codebook */
                 Word16  vdim,   /* vector dimension */
                 Word16  cbsz)   /* codebook size (number of codevectors) */
{
   Word32 dmin, d;
   Word16 *fp1, *fp2;
   Word16 j, k, stbl, s, t;
   Word16 xqc[LPCO];
   
   fp1     = cb;
   dmin    = MAX_32;
   *idx = -1;
   
   for (j = 0; j < cbsz; j++) {
      
      /* check stability */
      fp2  = fp1;
      xqc[0]  = add(xa[0],shr(*fp2++,4));			/* Q15 */
      if (xqc[0] < 0) stbl = 0;
      else stbl = 1;
      for (k=1; k<vdim; k++) {
         xqc[k]  = add(xa[k],shr(*fp2++,4));		/* Q15 */
         if(xqc[k] < xqc[k-1]) stbl = 0;
      }
      
      /* calculate distortion */
      d = 0;
      for (k=0; k<vdim; k++){
         t = sub(x[k],shr(*fp1++,1));
         s = extract_h(L_mult0(w[k],t));
         d = L_mac0(d, s, t);
      }
      
      /* update matches */
      if (stbl > 0){
         if (d < dmin) {
            dmin = d;
            *idx = j;
         }
      }
   }	/* end of j-loop */
   
   if(*idx == -1){
      *idx = 1;
   }
   
   fp1 = cb + (*idx)*vdim;
   for (k = 0; k < vdim; k++) xq[k] = shr(*fp1++,1);
   
}  

void vqmse(
           Word16  *xq,    /* Q16 VQ output vector (quantized version of input vector) */
           Word16  *idx,   /* VQ codebook index for the nearest neighbor */
           Word16  *x,     /* Q16 input vector */
           Word16  *cb,    /* VQ codebook */
           Word16	vdim,   /* vector dimension */
           Word16	cbsz)   /* codebook size (number of codevectors) */
{
   
   Word32 	dmin, d;
   Word16 	*fp1;
   Word16 	j, k;
   
   Word16 e;
   
   fp1 = cb;
   dmin = MAX_32;
   for (j = 0; j < cbsz; j++) {
      d = 0;
      for (k = 0; k < vdim; k++) {
         e = sub(x[k], *fp1++);     // Q17
         d = L_mac0(d, e, e);       // Q34
      }
      if (L_sub(d, dmin) < 0) {
         dmin = d;
         *idx = j;
      }
   }
   j = *idx * vdim;
   for (k = 0; k < vdim; k++) {
      xq[k] = cb[j + k];
   }
}  

