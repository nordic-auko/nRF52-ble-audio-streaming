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
  lspdec.c :  LSP decoding function

  $Log$
******************************************************************************/

#include "typedef.h"
#include "bvcommon.h"
#include "bv32cnst.h"
#include "bv32strct.h"
#include "bv32externs.h"
#include "basop32.h"

void lspdec(
            Word16  *lspq, 		/* Q15 */ 
            Word16  *lspidx,  
            Word16  *lsppm,		/* Q15 */
            Word16	*lspq_last)
{
   Word32	a0;
   Word16 	*fp1, *fp2;
   Word16  i, k, lspdordr;
   Word16 elsp[LPCO], lspe[LPCO]; 
   Word16 lspeq1[LPCO], lspeq2[LPCO];
   
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
   
   /* PERFORM FIRST-STAGE VQ CODEBOOK DECODE */
   vqdec(lspeq1,lspidx[0],lspecb1,LPCO);	/* lspeq1 is Q16 */ 
   
   /* PERFORM SECOND-STAGE VQ CODEBOOK DECODE */
   vqdec(lspeq2,lspidx[1],lspecb21,SVD1);
   vqdec(lspeq2+SVD1,lspidx[2],lspecb22,SVD2);	/* lspeq2 is Q19 */
   
   /* ENFORCE BIT_EXACT AS IN lspquan.c V1.10 */
   for (i=0;i<LPCO;i++) lspeq2[i] = shr(lspeq2[i], 1);
   
   /* GET OVERALL QUANTIZER OUTPUT VECTOR OF THE TWO-STAGE VQ */
   /* CALCULATE QUANTIZED LSP */
   for (i = 0; i < LPCO; i++) {
      lspe[i] = (Word16) L_shr(L_add(				/* rounding */
         L_shl(L_deposit_l(lspeq1[i]),3),
         L_shl(L_deposit_l(lspeq2[i]),1) ), 4);
      lspq[i] = add(add(lspe[i],elsp[i]),lspmean[i]);
   }
   
   /* detect bit-errors based on ordering property */
   if (lspq[0] < 0) lspdordr = 1;
   else lspdordr = 0;
   for (i=1; i<SVD1; i++) {
      if (lspq[i] < lspq[i-1]) lspdordr = 1;
   }
   
   /* substitute LSP and MA predictor update if bit-error detected */
   if (lspdordr) {
      for (i=0; i<LPCO; i++){
         lspq[i] = lspq_last[i];
         lspe[i] = sub(sub(lspq[i],elsp[i]),lspmean[i]);
      }
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
   
   /* ENSURE CORRECT ORDERING OF LSP TO GUARANTEE LPC FILTER STABILITY */
   stblz_lsp(lspq, LPCO);
   
}

void lspplc(
            Word16  *lspq, 		/* Q15 */ 
            Word16  *lsppm)		/* Q15 */
{
   Word32	a0;
   Word16 	*fp1, *fp2;
   Word16  i, k;
   Word16 elsp[LPCO];
   
   /* CALCULATE ESTIMATED (MA-PREDICTED) LSP VECTOR */
   fp1 = lspp;		/* Q14 */
   fp2 = lsppm;	/* Q15 */
   for (i = 0; i < LPCO; i++) {
      a0 = 0;
      for (k = 0; k < LSPPORDER; k++) {
         a0 = L_mac(a0, *fp1++, *fp2++);
      }
      elsp[i] = round(L_shl(a0,1));	/* Q15 */
      a0 = L_shl(a0, 1);
   }
   
   /* UPDATE LSP MA PREDICTOR MEMORY */
   i = LPCO * LSPPORDER - 1;
   fp1 = &lsppm[i];
   fp2 = &lsppm[i - 1];
   for (i = LPCO - 1; i >= 0; i--) {
      for (k = LSPPORDER; k > 1; k--) {
         *fp1-- = *fp2--;
      }
      *fp1-- = sub(sub(lspq[i],lspmean[i]),elsp[i]);
      fp2--;
   }
   
}
