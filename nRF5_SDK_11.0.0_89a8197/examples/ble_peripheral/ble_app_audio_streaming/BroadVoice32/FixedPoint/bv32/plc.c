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
  plc.c : Packet Loss Concealment

  $Log$
******************************************************************************/

#include "typedef.h"
#include "bvcommon.h"
#include "bv32cnst.h"
#include "utility.h"
#include "bv32strct.h"
#include "basop32.h"
#include "mathutil.h"
#include "bv32externs.h"

void BV32_PLC(
              struct  BV32_Decoder_State   *ds,
              Word16  *out)
{
   int n;
   Word16   xq[SFSZ];
   Word32   E;
   Word16   tmp;
   Word16   scplcg;        /* Q14 */
   Word32   acc;
   Word16   dsEw, dsEexp, Eexp, Ew;
   Word16   gain, gainexp;
   Word16   hi, lo, i_sf,i;
   Word16   d[LTMOFF+FRSZ];
   Word16   r[SFSZ];

   /*************************************************************/
   /* FEC operates on fixed Q0 speech (stsym, xq, dezfm, depfm) */
   /*************************************************************/
   /* shift ST  memory */
   for(i=0; i<LPCO; i++)
      ds->stsym[i] = shr(ds->stsym[i], ds->prv_exp); /* Q0 */
   
   /* shift de-emphasis memory */
   ds->depfm[0] = shr(ds->depfm[0], ds->prv_exp); /* Q0 */
   ds->dezfm[0] = shr(ds->dezfm[0], ds->prv_exp); /* Q0 */
   
   ds->prv_exp = 0;
   
   /************************************************************/
   /*                 Copy decoder state memory                */
   /************************************************************/
   W16copy(d, ds->ltsym, LTMOFF);
   
   /************************************************************/
   /*        Update counter of consecutive list frames         */
   /************************************************************/
   if(ds->cfecount < HoldPLCG+AttnPLCG-1)
      ds->cfecount = add(ds->cfecount, 1);
   
   /************************************************************/
   /*                Generate Unscaled Excitation              */
   /************************************************************/
   
   /* loop over subframes */
   for(i_sf=0; i_sf<FECNSF; i_sf++)
   {   

      E  = 0;
      for(n=0; n<SFSZ; n++)
      {
         ds->idum = 1664525L*ds->idum + 1013904223L;
         r[n] = extract_l(L_sub(L_shr(ds->idum, 16), 32767));
         tmp  = shr(r[n], 3);
         E = L_mac0(E, tmp, tmp);
      }
      
      /************************************************************/
      /*                      Calculate Scaling                   */
      /************************************************************/
      scplcg = add(ScPLCG_a, mult(ScPLCG_b, ds->per)); 
      if(scplcg > ScPLCGmax)
         scplcg = ScPLCGmax;
      else if(scplcg < ScPLCGmin)
         scplcg = ScPLCGmin;
      scplcg = shl(scplcg, 1 ); /* Q14->Q15 */
      
      
      dsEexp = norm_l(ds->E);
      dsEexp = sub(dsEexp, 1);
      dsEw   = extract_h(L_shl(ds->E, dsEexp));
      Eexp   = norm_l(E);
      Ew     = extract_h(L_shl(E, Eexp));
      Eexp   = sub(Eexp, 6);
      gain   = div_s(dsEw, Ew);
      gainexp= add(sub(dsEexp, Eexp), 15);
      if ((gainexp&1)==0)  /* make sure it is odd before sqrt() */
      {
         gain    = shr(gain, 1);
         gainexp = sub(gainexp, 1);
      }
      gain    = sqrts(gain);
      gainexp = add(shr(sub(gainexp,15),1), 15);
      gain    = mult(gain, scplcg);
      
      /************************************************************/
      /*                  Long-term synthesis filter              */
      /************************************************************/
      gainexp = sub(gainexp, 15+1);
      
      for(n=0; n<SFSZ; n++)
      {
         acc = (L_shr(L_mult(gain, r[n]), gainexp));
         acc = L_mac(acc, ds->bq_last[0], d[LTMOFF+i_sf*SFSZ+n-ds->pp_last+1]);
         acc = L_mac(acc, ds->bq_last[1], d[LTMOFF+i_sf*SFSZ+n-ds->pp_last]);
         acc = L_mac(acc, ds->bq_last[2], d[LTMOFF+i_sf*SFSZ+n-ds->pp_last-1]);
         d[LTMOFF+i_sf*SFSZ+n] = round(acc);
      }

      /************************************************************/
      /*                Short-term synthesis filter               */
      /************************************************************/
      apfilterQ1_Q0(ds->atplc, LPCO,  d+i_sf*SFSZ+LTMOFF, xq, SFSZ, ds->stsym, 1);
      
      /**********************************************************/
      /*                    De-emphasis filter                  */
      /**********************************************************/
      for(n=0; n<SFSZ; n++)
      {
         acc = L_shl(xq[n], 16);
         acc = L_mac(acc, -PEAZFC, ds->depfm[0]);
         ds->depfm[0] = round(acc);
         acc = L_mac(acc, PEAPFC, ds->dezfm[0]);
         ds->dezfm[0] = ds->depfm[0];
         out[i_sf*SFSZ+n] = round(acc);
      }
      
      /************************************************************/
      /*        Update memory of predictive gain quantizer        */
      /************************************************************/
      gainplc(ds->E, ds->lgpm, ds->prevlg);
      
      /************************************************************/
      /*                  Signal level estimation                 */
      /************************************************************/
      estlevel(ds->prevlg[0],&ds->level,&ds->lmax,&ds->lmin,
         &ds->lmean,&ds->x1);
   }
   /************************************************************/
   /*                 Save decoder state memory                */
   /************************************************************/
   W16copy(ds->ltsym, d+FRSZ, LTMOFF); /* excitation */
   
   /************************************************************/
   /*        Update memory of predictive LSP quantizer         */
   /************************************************************/
   lspplc(ds->lsplast,ds->lsppm);  
   
   /************************************************************/
   /*          Attenuation during long packet losses           */
   /************************************************************/
   if(ds->cfecount >= HoldPLCG)
   {
      acc = L_mult0( -AttnFacPLCG, sub(ds->cfecount, HoldPLCG-1));
      acc = L_add(1l<<20, acc);
      gain = round(L_shl(acc, 11));
      ds->bq_last[0] = mult(gain, ds->bq_last[0]);
      ds->bq_last[1] = mult(gain, ds->bq_last[1]);
      ds->bq_last[2] = mult(gain, ds->bq_last[2]);
      gain = mult(gain, gain);
      L_Extract(ds->E, &hi, &lo);
      ds->E = Mpy_32_16(hi, lo, gain);
   }

   return;
}
