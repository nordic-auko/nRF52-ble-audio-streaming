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
  bv32strct.h : BV32 Fixed-Point structures

  $Log$
******************************************************************************/

#ifndef BV32STRCT
#define BV32STRCT

struct BV32_Decoder_State {
   Word32   prevlg[2];
   Word32   lmax;
   Word32   lmin;
   Word32   lmean;
   Word32   x1;
   Word32   level;
   Word32   lgq_last;
   UWord32  idum;
   Word32   E;
   Word16   stsym[LPCO];
   Word16   ltsym[LTMOFF];
   Word16   lsppm[LPCO*LSPPORDER];
   Word16   lgpm[LGPORDER];
   Word16   lsplast[LPCO];
   Word16   dezfm[PFO];
   Word16   depfm[PFO];
   Word16   cfecount;
   Word16   bq_last[3];
   Word16   prv_exp;
   Word16   nclglim;
   Word16   lctimer;
   Word16   per;   /* Q15 */
   Word16   atplc[LPCO+1];
   Word16   pp_last;
};

struct BV32_Encoder_State {
   Word32   prevlg[2];
   Word32   lmax;
   Word32   lmin;
   Word32   lmean;
   Word32   x1;
   Word32   level;
   Word16   x[XOFF];          /* 8kHz down-sampled low-band signal memory */
   Word16   xwd[XDOFF];       /* memory of DECF:1 decimated version of xw() */
   Word16   xwd_exp;           /* or block floating-point in coarptch.c */
   Word16   dq[XOFF];        /* quantized short-term pred error */
   Word16   dfm_h[DFO];          /* decimated xwd() filter memory */
   Word16   dfm_l[DFO];
   Word16   stwpm[LPCO];        /* ST Weighting all-Pole Memory */
   Word16   stnfm[LPCO];        /* ST Noise Feedback filter Memory */
   Word16   ltsym[MAXPP1];    /* Q16 long-term synthesis filter memory */
   Word16   ltnfm[MAXPP1];    /* Q16 long-term noise feedback filter memory */
   Word16   lsppm[LPCO*LSPPORDER];  /* LSP Predictor Memory */
   Word16   old_a[LPCO+1];
   Word16   lsplast[LPCO];
   Word16   lgpm[LGPORDER];      /* Q11 Log-Gain Predictor Memory */
   Word16   hpfzm[HPO];
   Word16   hpfpm[2*HPO];
   Word16   cpplast;          /* pitch period pf the previous frame */
};

struct BV32_Bit_Stream {
   short    lspidx[3];
   short    ppidx;      /* 9 bit */
   short    bqidx;
   short    gidx[2];
   short    qvidx[NVPSF];
};

#endif

