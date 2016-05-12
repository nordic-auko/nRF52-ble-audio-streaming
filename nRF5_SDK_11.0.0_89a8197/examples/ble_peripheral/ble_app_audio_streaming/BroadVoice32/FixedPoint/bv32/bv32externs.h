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
  bv32externs.c : BV32 Fixed-Point externs

  $Log$
******************************************************************************/

/* POINTERS */
extern Word16 winl[];
extern Word16 sstwinl_h[];
extern Word16 sstwinl_l[];
extern Word16 idxord[];
extern Word16 hpfa[];
extern Word16 hpfb[];
extern Word16 adf_h[];
extern Word16 adf_l[];
extern Word16 bdf[];
extern Word16 x[];
extern Word16 x2[];
extern Word16 MPTH[];

/* LSP Quantization */
extern Word16 lspecb1[];
extern Word16 lspecb21[];
extern Word16 lspecb22[];
extern Word16 lspmean[];
extern Word16 lspp[];

/* Log-Gain Quantization */
extern Word16 lgpecb[];
extern Word16 lgp[];
extern Word16 lgmean;

/* Log-Gain Limitation */
extern Word16 lgclimit[];

/* Excitation Codebook */
extern Word16 cccb[];

extern Word16 lgpecb_nh[];
extern Word16 a_pre[];
extern Word16 b_pre[];

/* Function Prototypes */

extern Word32 estlevel(
Word32  lg,
Word32  *level,
Word32  *lmax,
Word32  *lmin,
Word32  *lmean,
Word32  *x1);

extern void excdec_w_LT_synth(
		       Word32	*qv,	/* normalized excitation vector */
		       Word16 	*ltsym, /* (i/o) Q1 long-term synthesis filter memory */
		       Word16  *idx,   /* quantizer codebook index for uq[] vector */
		       Word16	*b,     /* (i) Q15 coefficient of 3-tap pitch predictor */
		       Word16 	*cb,    /* (i) Q1 scalar quantizer codebook */
		       Word16 	pp,     /* pitch period (# of 8 kHz samples) */
		       Word16	exp,
             Word32  *EE);	/* gain_exp of current frame */

extern Word32 gaindec(	/* Q18 */
	Word32	*lgq,		/* DQ25 */
	Word16  gidx,
	Word16  *lgpm,		/* Q11 */
	Word32	*prevlg,
	Word32	level,
	Word16  *nclglim,
	Word16  lctimer);

void gainplc(Word32 E, Word16 *lgeqm, Word32 *lgqm);
	
extern void lspdec(
Word16  *lspq, 		/* Q15 */ 
Word16  *lspidx,  
Word16  *lsppm,		/* Q15 */
Word16	*lspq_last);

extern void lspplc(
Word16  *lspq,      /* Q15 */
Word16  *lsppm);    /* Q15 */

extern Word16 coarsepitch(
Word16 	*xw,		 	/* (i) Q1 weighted low-band signal frame */
struct BV32_Encoder_State *c); /* (i/o) coder state */

extern Word16 refinepitch(
Word16 	*x,
Word16  cpp,
Word16	*ppt);

extern Word16 pitchtapquan(
Word16	*x,
Word16 	pp,
Word16 	*b);

extern void excquan(
    Word16  *idx,   /* quantizer codebook index for uq[] vector */
    Word16  *d,     /* (i/o) Q1 prediction residual signal vector */
    Word16  *h,     /* (i) Q12 noise feedback filter coefficient array */
    Word16  *b,     /* (i) Q15 coefficient of 3-tap pitch predictor */
    Word16  beta,   /* (i) Q13 coefficient of weighted 3-tap pitch predictor */
    Word16  *ltsym, /* (i/o) long-term synthesis filter memory */
    Word16  *ltnfm, /* (i/o) long-term noise feedback filter memory */
    Word16  *stnfm, /* filter memory before filtering of current vector */
    Word16  *cbs,   /* (i) Q1 scalar quantizer codebook */
    Word16  pp,     /* pitch period (# of 8 kHz samples) */
	Word16	gexp);	/* gain_exp */

extern Word16 gainquan(
Word32  *gainq,      /* Q18 */	
Word32  ee,          /* Q3 */
Word16  *lgpm,	     /* Q11 */
Word32	*prevlg,
Word32	level);

extern void lspquan(
Word16  *lspq,  
Word16  *lspidx,  
Word16  *lsp,     
Word16  *lsppm);

extern Word32 residual_energy(
Word16  *x,
Word16  pp,
Word16  *b);

extern void preprocess(
struct 	BV32_Encoder_State    *cs,
Word16	*output,	 /* output signal */
Word16  *signal,     /* input signal */
Word16  lg);         /* length of signal    */
