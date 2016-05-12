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
  excdec.c :excitation signal decoding including long-term synthesis.

  $Log$
******************************************************************************/

#include "typedef.h"
#include "bv32cnst.h"
#include "basop32.h"

void excdec_w_LT_synth(
                       Word32	*qv, 	/* (o) normalized excitation vector */
                       Word16	*ltsym,	/* (i/o) Q16 long-term synthesis filter memory */
                       Word16 	*idx,   /* (o) quantizer codebook index for uq[] vector */
                       Word16	*b,     /* (i) Q15 coefficient of 3-tap pitch predictor */
                       Word16 	*cb,    /* (i) Q0 scalar quantizer codebook */
                       Word16 	pp,     /* pitch period (# of 8 kHz samples) */
                       Word16	new_exp,	/* gain_exp of current sub-frame */
                       Word32   *EE
                       )
{
   Word32 a0, a1;
   Word16 *sp1, *sp2;
   Word16 m, n, jmin, iv, sign, tt;
   Word32 E;
   
   E=0;
   
   /* LOOP THROUGH EVERY VECTOR OF THE CURRENT SUBFRAME */
   iv = 0;     /* iv = index of the current vector */
   for (m = 0; m < SFSZ; m += VDIM) {
      
      /* THE BEST CODEVECTOR HAS BEEN FOUND; ASSIGN VQ CODEBOOK INDEX */
      jmin = idx[iv++];
      sign = (jmin&CBSZ);
      jmin = jmin-sign;
      sp2 = &cb[jmin*VDIM];	/* normalized (new_exp) excitation codebook */
      
                              /* COMPUTE PITCH-PREDICTED VECTOR, WHICH SHOULD BE INDEPENDENT OF THE
      RESIDUAL VQ CODEVECTORS BEING TRIED IF vdim < MIN. PITCH PERIOD */
      
      for (n = m; n < m + VDIM; n++) {
         
         sp1 = &ltsym[n-pp+1]; /* Q1 */
         a0 = L_mult0(*sp1--, b[0]); /* Q16 */
         a0 = L_mac0(a0, *sp1--, b[1]); /* Q16 */
         a0 = L_mac0(a0, *sp1--, b[2]); /* Q16 */
         a1 = L_shr(L_deposit_h(*sp2++), new_exp);
         if (sign!=0) a1 = L_negate(a1);
         a0 = L_add(a0, a1); /* Q16 */
         qv[n] = a0; /* Q16 */
         ltsym[n] = round(L_shl(a0,1)); /* Q1 */
         
         /* Excitation energy for PLC */
         tt = round(a1);                     // Q0
         E = L_mac0(E, tt, tt);
      }
      
   }
   *EE = E;
   
}
