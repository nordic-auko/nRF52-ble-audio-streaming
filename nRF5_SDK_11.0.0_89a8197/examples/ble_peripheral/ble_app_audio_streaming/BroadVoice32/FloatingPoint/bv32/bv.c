/*****************************************************************************/
/* BroadVoice(R)32 (BV32) Floating-Point ANSI-C Source Code                  */
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
  bv.c : BroadVoice32 Codec Entry Interface Program

  $Log$
******************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "typedef.h"
#include "bv32cnst.h"
#include "bvcommon.h"
#include "bv32strct.h"
#include "bv32.h"
#include "utility.h"
#if G192BITSTREAM
#include "g192.h"
#else
#include "bitpack.h"
#endif

int      frame;
short   bfi=0;

void  usage(char *name)
{
   fprintf(stderr,"usage: %s enc|dec input output\n", name);
   fprintf(stderr,"\nFormat for speech_file:\n    Binary file of 16 kHz sampled 16-bit PCM data.\n");
#if G192BITSTREAM
   fprintf(stderr,"\nFormat for bitstream_file per frame: ITU-T G.192 format\n\
                                                                                One (2-byte) synchronization word [0x6B21],\n\
                                                                                One (2-byte) size word,\n\
                                                                                160 words (2-byte) containing 160 bits.\n\n");
#else
   fprintf(stderr, "\nFormat for bitstream_file per frame: Packed Bits\n");
#endif
   exit(1);
}

int   main(int argc, char **argv)
{
   FILE     *fi, *fo, *fbdi=NULL;
   int      enc=1;
   int      nread, i;
   short    x[FRSZ];
   struct	BV32_Bit_Stream bs;
   struct   BV32_Encoder_State cs;
   struct	BV32_Decoder_State ds;
   
#if !G192BITSTREAM
   UWord8 PackedStream[20];
#endif
   
   int next_bad_frame=-1;
   
   fprintf(stderr,"/***************************************************************************/\n");
   fprintf(stderr,"/* BroadVoice(R)32, Copyright (c) 2000-12, Broadcom Corporation.           */\n");
   fprintf(stderr,"/* All Rights Reserved.                                                    */\n");
   fprintf(stderr,"/*                                                                         */\n");
   fprintf(stderr,"/* This software is provided under the GNU Lesser General Public License,  */\n");
   fprintf(stderr,"/* version 2.1, as published by the Free Software Foundation (\"LGPL\").     */\n");
   fprintf(stderr,"/* This program is distributed in the hope that it will be useful, but     */\n");
   fprintf(stderr,"/* WITHOUT ANY SUPPORT OR WARRANTY; without even the implied warranty of   */\n");
   fprintf(stderr,"/* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the LGPL for   */\n");
   fprintf(stderr,"/* more details.  A copy of the LGPL is available at                       */\n");
   fprintf(stderr,"/* http://www.broadcom.com/licenses/LGPLv2.1.php,                          */\n");
   fprintf(stderr,"/* or by writing to the Free Software Foundation, Inc.,                    */\n");
   fprintf(stderr,"/* 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.               */\n");
   fprintf(stderr,"/***************************************************************************/\n");
   
   if ((argc!=4)&&(argc!=5)) usage(argv[0]);
   if (!strcmp(argv[1],"enc")) enc=1;
   else if (!strcmp(argv[1],"dec")) enc=0;
   else usage(argv[0]);
   
   if (!(fi=fopen(argv[2],"rb"))) 
   {
      fprintf(stderr,"error: can't read %s\n", argv[2]);
      exit(2);
   }
   if (!(fo=fopen(argv[3],"wb"))) 
   {
      fprintf(stderr,"error: can't write to %s\n", argv[3]);
      exit(3);
   }
   if (argc==5)
   {
      if (!(fbdi=fopen(argv[4],"rb"))) 
      {
         fprintf(stderr,"error: can't read %s\n", argv[4]);
         exit(3);
      }
   }
   
   if (!strcmp(argv[1],"enc")) 
   {
      Reset_BV32_Coder(&cs);
   } 
   else 
   {
      Reset_BV32_Decoder(&ds);
   }
   
   if (enc){
#if G192BITSTREAM
      fprintf(stderr," BroadVoice32 Floating-Point Encoder V1.2 with ITU-T G.192\n");
#else
      fprintf(stderr," BroadVoice32 Floating-Point Decoder V1.2 with packed bit-stream\n");
#endif
      fprintf(stderr," Input speech file     : %s\n",argv[2]);
      fprintf(stderr," Output bit-stream file: %s\n",argv[3]);
   }
   else{
#if G192BITSTREAM
      fprintf(stderr," BroadVoice32 Floating-Point Decoder V1.2 with ITU-T G.192\n");
#else
      fprintf(stderr," BroadVoice32 Floating-Point Decoder V1.2 with packed bit-stream\n");
#endif
      fprintf(stderr," Input bit-stream file : %s\n",argv[2]);
      fprintf(stderr," Output speech file    : %s\n",argv[3]);
   }
   
   
   /* START THE MAIN FRAME LOOP */
   frame=0; 
   /* read for the 1st bad frame */
   if (fbdi!=NULL)
      fscanf(fbdi,"%d", &next_bad_frame);
   
   while (1) 
   {
      
      /* FRAME COUNTER */
      frame++;
      
      /* READ IN ONE SPEECH FRAME */
      if (enc==1) 
      {
         nread=fread(x, sizeof(short), FRSZ, fi);
         if (nread<=0) goto End;
         for (i=nread;i<FRSZ;i++) x[i] = 0; 
      } 
      else 
      {
#if G192BITSTREAM
         nread = fread_wb_g192bitstrm(&bs, fi);
#else
         nread = fread(PackedStream, sizeof(UWord8), 20, fi);
         BV32_BitUnPack (PackedStream, &bs ); 
#endif
         if (nread<=0) goto End;
         if (frame==next_bad_frame) 
         {
            fscanf(fbdi,"%d", &next_bad_frame);
            bfi = 1;
         }
      }
      
      /* BV32 CODING */
      
      if (enc==1) 
      {
         BV32_Encode(&bs, &cs, x);
#if G192BITSTREAM
         fwrite_wb_g192bitstrm(&bs,fo);
#else
         BV32_BitPack( PackedStream, &bs );
         fwrite(PackedStream, sizeof(UWord8), 20, fo);         
#endif
      } 
      else 
      {
         if (!bfi) 
            BV32_Decode(&bs, &ds, x);   
         else 
         {
            BV32_PLC(&ds,x); 
         }
         
         fwrite(x, sizeof(short), FRSZ, fo);
      }
      
      if (((frame/100)*100)==frame) fprintf(stderr, "\r %d %d-sample frames processed.", frame, FRSZ);
      bfi = 0;
   } /* END OF FRAME LOOP */
   
End: ;
     
     frame--;
     fprintf(stderr, "\r %d 40-sample frames processed.\n", frame);
     
     fclose(fi);
     fclose(fo);
     
     if (fbdi!=NULL)
        fclose(fbdi);

     fprintf(stderr, "\n\n");
     
     return 0;
}
