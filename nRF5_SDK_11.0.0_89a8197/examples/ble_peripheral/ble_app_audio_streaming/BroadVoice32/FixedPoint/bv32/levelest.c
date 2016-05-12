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
  levelest.c : Input level estimation

  $Log$
******************************************************************************/

#include "typedef.h"
#include "bv32cnst.h"
#include "basop32.h"

Word32	estlevel(
                  Word32	lg,
                  Word32	*level,
                  Word32	*lmax,
                  Word32	*lmin,
                  Word32	*lmean,
                  Word32	*x1)
{
   Word32	lth;
   Word32	a0;
   Word16	s, t;
   
   /* UPDATE THE NEW MAXIMUM, MINIMUM, & MEAN OF LOG-GAIN */
   if (lg > *lmax) *lmax=lg;	/* use new log-gain as max if it is > max */
   else { 					/* o.w. attenuate toward lmean */
      
      /* *lmax=*lmean+estl_alpha*(*lmax-*lmean); */
      a0 = L_sub(*lmax, *lmean);
      L_Extract(a0, &s, &t);
      a0 = Mpy_32_16(s, t, estl_alpha);
      *lmax = L_add(a0, *lmean);
   }
   
   if (lg < *lmin) *lmin=lg;	/* use new log-gain as min if it is < min */
   else { 					/* o.w. attenuate toward lmean */
      
      /* *lmin=*lmean+estl_alpha*(*lmin-*lmean); */
      a0 = L_sub(*lmin, *lmean);
      L_Extract(a0, &s, &t);
      a0 = Mpy_32_16(s, t, estl_alpha);
      *lmin = L_add(a0, *lmean);
   }
   
   /* *lmean=estl_beta*(*lmean)+estl_beta1*(0.5*(*lmax+*lmin)); */
   a0 = L_shr(L_add(*lmax, *lmin),1);
   L_Extract(a0, &s, &t);
   a0 = Mpy_32_16(s, t, estl_beta1);
   L_Extract(*lmean, &s, &t);
   *lmean = L_add(a0, Mpy_32_16(s, t, estl_beta));
   
  	/* UPDATE ESTIMATED INPUT LEVEL, BY CALCULATING A RUNNING AVERAGE
   (USING AN EXPONENTIAL WINDOW) OF LOG-GAINS EXCEEDING lmean */
   /* lth=*lmean+estl_TH*(*lmax-*lmean); */
   
   a0 = L_sub(*lmax, *lmean);
   L_Extract(a0, &s, &t);
   lth = L_add(*lmean, Mpy_32_16(s, t, estl_TH));
   
   if (lg > lth) {
      
      /* *x1=estl_a*(*x1)+estl_a1*lg; */
      
      L_Extract(*x1, &s, &t);
      a0 = Mpy_32_16(s, t, estl_a);
      L_Extract(lg, &s, &t);
      *x1 = L_add(a0, Mpy_32_16(s, t, estl_a1));
      
      /* *level=estl_a*(*level)+estl_a1*(*x1); */
      
      L_Extract(*level, &s, &t);
      a0 = Mpy_32_16(s, t, estl_a);
      L_Extract(*x1, &s, &t);
      *level = L_add(a0, Mpy_32_16(s, t, estl_a1));
   }
   
   return	lth;
   
}
