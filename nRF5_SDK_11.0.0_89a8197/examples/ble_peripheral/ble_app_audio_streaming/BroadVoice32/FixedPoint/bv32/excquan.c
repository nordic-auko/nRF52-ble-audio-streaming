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
  excquan.c : Vector Quantizer for 2-Stage Noise Feedback Coding 
            with long-term predictive noise feedback coding embedded 
            inside the short-term predictive noise feedback coding loop.

  Note that the Noise Feedback Coding of the excitation signal is implemented
  using the Zero-State Responsse and Zero-input Response decomposition as 
  described in: J.-H. Chen, "Novel Codec Structures for Noise Feedback 
  Coding of Speech," Proc. ICASSP, 2006. 

  $Log$
******************************************************************************/

#include "typedef.h"
#include "bvcommon.h"
#include "bv32cnst.h"
#include "basop32.h"
#include "utility.h"

void excquan(
             Word16 *idx,   /* quantizer codebook index for uq[] vector */
             Word16	*d,     /* (i/o) Q0 prediction residual signal vector */
             Word16	*h,     /* (i) Q12 noise feedback filter coefficient array */
             Word16	*b,     /* (i) Q15 coefficient of 3-tap pitch predictor */
             Word16	beta,   /* (i) Q13 coefficient of pitch feedback filter */
             Word16 *ltsym, /* (i/o) Q16 long-term synthesis filter memory */
             Word16 *ltnfm, /* (i/o) Q16 long-term noise feedback filter memory */
             Word16	*stnfm, /* (i/o) Q16 filter memory before filtering */
             Word16 *cb,    /* (i) Q1 scalar quantizer codebook */
             Word16 pp,     /* pitch period (# of 8 kHz samples) */
             Word16	gain_exp
             )
{
   Word32 a0, a1, a2;
   Word16 *sp1, *sp2, *sp3, *sp4;
   Word16 t, sign=1;
   Word32 *lp2, *lp3, *lp4;
   Word16 i, j, m, n, jmin, iv;
   Word16 gexpm3;
   Word32 Emin, E;
   Word16 e;
   Word16 qzir[VDIM];	        /* Q0 */
   Word16 buf[LPCO+SFSZ];      /* Q16 buffer for filter memory & signal */
   Word32 ltfv[VDIM], ppv[VDIM]; /* Q16 */
   Word16 qzsr[VDIM*CBSZ];
   Word32 Ezsr[CBSZ];
   
   gexpm3 = sub(gain_exp, 3);
   
   /* COPY FILTER MEMORY TO BEGINNING PART OF TEMPORARY BUFFER */
   W16copy(buf, stnfm, LPCO);
   
   /* CALCULATE ZERO-STATE RESPONSE CODEBOOK */ 
   sp2 = cb;						/* Q0 normalized by gain_exp */
   sp3 = qzsr;
   
   for (j=0; j<CBSZ; j++) {
      *sp3 = shr(*sp2++,1);				/* Q-1 by gain_exp */
      Ezsr[j] = L_mult0(*sp3, *sp3);		/* Q-2 by 2*gain_exp */
      for (n=1; n<VDIM; n++) {
         sp1 = &h[n];		/* Q12 */
         sp4 = sp3;			/* Q-1 */
         a0 = 0;
         for (i=0;i<n;i++) a0 = L_msu0(a0,*sp4++,*sp1--); /* Q11 gain_exp */
         a0 = L_shl(a0, 4);	/* Q15 gain_exp */
         a0 = L_add(L_shr(L_deposit_h(*sp2++),1), a0);  /* Q15 gain_exp */
         *sp4 = t = round(a0);					/* Q-1 gain_exp */
         Ezsr[j] = L_mac0(Ezsr[j], t, t);		/* Q-2 2*gain_exp */
      }
      sp3 += VDIM;
   }
   
   /* LOOP THROUGH EVERY VECTOR OF THE CURRENT SUBFRAME */
   iv = 0;     /* iv = index of the current vector */
   for (m = 0; m < SFSZ; m += VDIM) {
      
      /* COMPUTE PITCH-PREDICTED VECTOR, WHICH SHOULD BE INDEPENDENT OF THE
      RESIDUAL VQ CODEVECTORS BEING TRIED IF vdim < MIN. PITCH PERIOD */
      lp2 = ltfv; lp3 = ppv;		   /* Q16 */
      for (n = m; n < m + VDIM; n++) {
         
         sp1 = &ltsym[MAXPP1+n-pp+1];   /* Q1 */
         a0 = L_mult0(*sp1--, b[0]);     /* Q16 */
         a0 = L_mac0(a0, *sp1--, b[1]);
         a0 = L_mac0(a0, *sp1--, b[2]);
         *lp3++ = a0;                    /* write result to ppv[] vector */
         a1 = L_mult0(ltnfm[MAXPP1+n-pp], beta); /* Q14 */
         a1 = L_shl(a1, 2);
         *lp2++ = L_add(a0, a1);   /* Q16 */
         
      }
      
      /* COMPUTE ZERO-INPUT RESPONSE */
      lp2 = ppv;		/* Q16 */
      sp3 = qzir;		/* Q0 */
      lp4 = ltfv;		/* Q16 */
      
      for (n = m; n < m + VDIM; n++) {
         
         /* PERFORM MULTIPLY-ADDS ALONG THE DELAY LINE OF FILTER */
         sp1 = &buf[n];	/* Q16 */
         a0 = L_mult(d[n], 2048); /* Q13 */
         for (i = LPCO; i > 0; i--) a0 = L_msu(a0, *sp1++, h[i]);
         a0 = L_shl(a0, 3);								/* Q16 */
         
                                                   /* a0 NOW CONTAINS v[n]; ADD THE SUM OF THE TWO LONG_TERM
         FILTERS TO GET THE INPUT TO THE VQ BLOCK */
         *sp3++ = round(L_shl(L_sub(a0,*lp4++),gexpm3));
         
         /* q[n] = u[n] during ZIR computation */
         
         /* SUBTRACT VQ OUTPUT TO GET VQ QUANTIZATION ERROR q[n] */
         a0 = L_sub(a0, *lp2++); 
         
         /* a0 now contains zero-input qs[n] */
         *sp1 = round(a0); 
         /* update short-term noise feedback filter memory */
      }
      
      /* LOOP THROUGH EVERY CODEVECTOR OF THE RESIDUAL VQ CODEBOOK */
      /* AND FIND THE ONE THAT MINIMIZES THE ENERGY OF q[n] */
      Emin = MAX_32;
      jmin = 0;
      sign = 1;
      sp4 = qzsr;
      for (j = 0; j < CBSZ; j++) {
         /* Try positive sign */
         sp2 = qzir;
         E = 0;
         for (n=0;n<VDIM;n++){
            e = sub(shl(*sp2++,2), *sp4++);
            E = L_mac0(E, e, e);
         }
         if(L_sub(E, Emin) < 0){
            jmin = j;
            Emin = E;
            sign = 1;
         }
         /* Try negative sign */
         sp4 -= VDIM;
         sp2 = qzir;
         E = 0;
         for (n=0;n<VDIM;n++){
            e = add(shl(*sp2++,2), *sp4++);
            E = L_mac0(E, e, e);
         }
         if(L_sub(E, Emin) < 0){
            jmin = j;
            Emin = E;
            sign = -1;
         }
      }
      
      /* THE BEST CODEVECTOR HAS BEEN FOUND; ASSIGN VQ CODEBOOK INDEX */
      idx[iv++] = (sign==-1) ? (jmin+CBSZ) : jmin ;
      
      /* BORROW qzir[] to STORE VQ OUTPUT VECTOR WITH CORRECT SIGN */
      sp3 = &cb[jmin*VDIM]; /* sp3 points to start of best codevector */
      for (n=0;n<VDIM;n++) {
         qzir[n] = sign * (*sp3++);	/* Q0 normalized by gain_exp */
      }
      
      lp2 = ppv;		/* Q16 lp2 points to start of pitch-predicted vector */ 
      sp3 = qzir;		/* sp3 points to start of VQ output vector */
      lp4 = ltfv;     /* Q16 lp4 points to long-term filtered vector */
      
      /* LOOP THROUGH EVERY ELEMENT OF THE CURRENT VECTOR */
      for (n = m; n < m + VDIM; n++) {
         
         /* PERFORM MULTIPLY-ADDS ALONG THE DELAY LINE OF FILTER */
         sp1 = &buf[n];
         a0 = L_mult(d[n], 2048); /* Q13 */
         for (i = LPCO; i > 0; i--) a0 = L_msu(a0, *sp1++, h[i]);
         a0 = L_shl(a0, 3); /* Q16 */
         
         /* COMPUTE VQ INPUT SIGNAL u[n] */
         a1 = L_sub(a0,*lp4++); /* a1 contains u[n] */
         
         /* COMPUTE VQ ERROR q[n] */
         a2 = L_shr(L_deposit_h(*sp3++),gain_exp);
         a1 = L_sub(a1, a2);
         
         /* UPDATE LONG-TERM NOISE FEEDBACK FILTER MEMORY */
         ltnfm[MAXPP1+n] = round(L_shl(a1,1)); /* Q1 */
         
         /* CALCULATE QUANTIZED LPC EXCITATION VECTOR qv[n] */
         a1 = L_add(a2, *lp2++); /* Q16 */
         
         /* UPDATE LONG-TERM PREDICTOR MEMORY */
         ltsym[MAXPP1+n] = d[n] = round(L_shl(a1,1)); /* Q1 */
         
         /* COMPUTE ERROR BETWEEN v[n] AND qv[n] */
         a0 = L_sub(a0, a1);
         /* a0 now contains u[n] - qv[n] = qs[n] */
         
         /* UPDATE SHORT-TERM NOISE FEEDBACK FILTER MEMORY */
         *sp1 = round(a0);      
      }
            
  }
  
  /* UPDATE NOISE FEEDBACK FILTER MEMORY AFTER FILTERING CURRENT SUBFRAME */
  W16copy(stnfm, (sp1-LPCO+1), LPCO);
  
}
