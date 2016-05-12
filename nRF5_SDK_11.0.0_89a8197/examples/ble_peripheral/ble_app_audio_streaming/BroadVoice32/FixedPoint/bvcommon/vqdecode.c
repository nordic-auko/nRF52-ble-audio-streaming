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
  vqdecode.c : Common Fixed-Point Library: 

  $Log$
******************************************************************************/

#include "typedef.h"

void vqdec(
           Word16  *xq,    /* VQ output vector (quantized version of input vector) */
           Word16  idx,   /* VQ codebook index for the nearest neighbor */
           Word16  *cb,    /* VQ codebook */
           Word16  vdim)   /* vector dimension */
{
   
   Word16   j, k;
   j = idx * vdim;
   for (k = 0; k < vdim; k++) 
      xq[k] = cb[j + k];
}
