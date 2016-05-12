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
  bv32cnst.h : BV32 constants

  $Log$
******************************************************************************/

#ifndef	BV32CNST_H
#define  BV32CNST_H

/* ----- Basic Codec Parameters ----- */
#define  SF             16             /* input Sampling Frequency (in kHz)            */
#define  FRSZ           80             /* frame size (in terms of # of 16 kHz samples)   */
#define  MAXPP          265            /* MAXimum Pitch Period (in # of 16 kHz samples)  */
#define  MINPP          10             /* MINimum Pitch Period (in # of 16 kHz samples)  */
#define  NSF            2              /* number of sub-subframes per subframe */
#define  PWSZ           240            /* Pitch analysis Window SiZe    */
#define  SFSZ           (FRSZ/NSF)
#define  WINSZ          160            /* lpc analysis WINdow SiZe         */
#define  MAXPP1         (MAXPP+1)      /* MAXimum Pitch Period + 1  */

/* NFC VQ coding parameters */
#define  VDIM           4              /* excitation vector dimension */      
#define  CBSZ           32             /* codebook size */
#define  PPCBSZ         32
#define  LGPORDER       16             /* Log-Gain Predictor OODER */
#define  LGPECBSZ       32             /* Log-Gain Prediction Error CodeBook SiZe */
#define  LSPPORDER      8              /* LSP MA Predictor ORDER */
#define  LSPECBSZ1      128            /* codebook size of 1st-stage LSP VQ */
#define  SVD1           3              /* split VQ dimension 1 */
#define  LSPECBSZ21     32             /* codebook size of 2nd-stage LSP split VQ */
#define  SVD2           5              /* split VQ dimension 2 */
#define  LSPECBSZ22     32             /* codebook size of 2nd stage LSP split VQ */

#define  NVPSF          (FRSZ/VDIM)
#define  NVPSSF         (SFSZ/VDIM)

#define  ScPLCGmin      1639           /* 0.1 Q14 */
#define  ScPLCGmax      14746          /* 0.9 Q14 */
#define  FECNSF         2              /* number of FEC subframes per frame         */
#define  ScPLCG_b       -32768         /* -2.0 in Q14 */
#define  ScPLCG_a       31129          /* 1.9 in Q14 */
#define  HoldPLCG       8
#define  AttnPLCG       50
#define  AttnFacPLCG    20971

/* Pre-emphasis filter coefficients */
#define PEAPFC 24576          /* 32768.0*0.75*/
#define PEAZFC	16384          /* 32768.0*0.5 */

#define  TMinlgXsfsz    ((Word16)(0.25*SFSZ)) /* minimum linear gain times subframe size */
#define  GPO            16             /* order of MA prediction       */

/* Level Estimation */
#define  estl_alpha     32764          /* (8191./8192.) */
#define  estl_beta      32736          /* (1023./1024.) */
#define  estl_beta1     32             /* (1.-estl_beta) */
#define  estl_a         32704          /* (511./512.) */
#define  estl_a1        64             /* (1-estl_a) */
#define  estl_TH        6554           /* 0.2 */

/* Log-Gain Limitation */
#define  LGLB           -24            /* Log-Gain Lower Bound */
#define  LGCLB          -8             /* Log-Gain Change Lower Bound */
#define  NGB            18             /* Number of Gain Bins */
#define  NGCB           11             /* Number of Gain Change Bins */
#define  MinE           0xfc000000     /* -2.0 Q25 */

#define  PFO            1              /* preemphasis filter order */

#define LTMOFF MAXPP1	/* Long-Term filter Memory OFFset */	

/* Parameters related to the gain decoder trap */
#define  NCLGLIM_TRAPPED        50     /* 0.125 sec */
#define  LEVEL_CONVERGENCE_TIME 100    /* 0.25 sec */

/* front-end highpass filter */ 
#define  HPO            2              /* High-pass filter order */

/* lpc weighting filter */
#define  LTWFL          4096           /* 0.5 in Q13 perceptual Weighting Factor Lowband  */

/* Minimum gain threshold */
#define  TMinE          10*2           /* Q1 (SSFSZ*0.25) */

/* ------ Decimation Parameters ----- */
#define  DECF           8
#define  FRSZD          (FRSZ/DECF)
#define  MAXPPD         (MAXPP/DECF)   /* MAX Pitch in DECF:1 */
#define  MINPPD         ((int) (MINPP/DECF))
#define  PWSZD          (PWSZ/DECF)    /* Pitch ana. Window SiZe */
#define  DFO            4
#define  MAXPPD1        (MAXPPD+1)
#define  LXD            (MAXPPD1+PWSZD)
#define  XDOFF          (LXD-FRSZD)
#define  HMAXPPD        (MAXPPD/2)
#define  M1             (MINPPD-1)
#define  M2             MAXPPD1
#define  HDECF          (DECF/2)

/* coarse pitch */
#define MPTH4  9830
#define DEVTH  8192     /* 0.25 pitch period DEViation THreshold */
#define TH1   23921     /* first threshold for cor*cor/energy   */
#define TH2   13107     /* second threshold for cor*cor/energy  */
#define LPTH1 25559     /* Last Pitch cor*cor/energy THreshold 1 */
#define LPTH2 14090     /* Last Pitch cor*cor/energy THreshold 2 */
#define MPDTH  1966     /* Multiple Pitch Deviation THreshold */
#define SMDTH  3113     /* Sub-Multiple pitch Deviation THreshold  0.125 */
#define MAX_NPEAKS 7

/* buffer offset and length */
#define  XOFF           MAXPP1         /* offset for x() frame      */
#define  LX             (XOFF+FRSZ)    /* Length of x() buffer      */

#endif
