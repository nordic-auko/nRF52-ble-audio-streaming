BV32DIR     = ../bv32
BVCOMMONDIR = ../bvcommon
OBJDIR      = ./obj

CC=gcc
CFLAGS= -DWMOPS=0 -DG192BITSTREAM=1 -I $(BV32DIR) -I $(BVCOMMONDIR) -O -Wall -o $@

OBJS = 	$(OBJDIR)/a2lsp.o \
	$(OBJDIR)/allpole.o \
	$(OBJDIR)/allzero.o \
	$(OBJDIR)/autocor.o \
	$(OBJDIR)/basop32.o \
	$(OBJDIR)/cmtables.o \
	$(OBJDIR)/levdur.o \
	$(OBJDIR)/lsp2a.o \
	$(OBJDIR)/mathtables.o \
	$(OBJDIR)/mathutil.o \
	$(OBJDIR)/memutil.o \
	$(OBJDIR)/ptdec.o \
	$(OBJDIR)/stblzlsp.o \
	$(OBJDIR)/utility.o \
	$(OBJDIR)/vqdecode.o \
	$(OBJDIR)/bitpack.o \
	$(OBJDIR)/bv.o \
	$(OBJDIR)/coarptch.o \
	$(OBJDIR)/decoder.o \
	$(OBJDIR)/encoder.o \
	$(OBJDIR)/excdec.o \
	$(OBJDIR)/excquan.o \
	$(OBJDIR)/fineptch.o \
	$(OBJDIR)/g192.o \
	$(OBJDIR)/gaindec.o \
	$(OBJDIR)/gainquan.o \
	$(OBJDIR)/levelest.o \
	$(OBJDIR)/lspdec.o \
	$(OBJDIR)/lspquan.o \
	$(OBJDIR)/plc.o \
	$(OBJDIR)/preproc.o \
	$(OBJDIR)/ptquan.o \
	$(OBJDIR)/tables.o \

BroadVoice32.exe: $(OBJS)
	$(CC) -lm $(CFLAGS) -o ../process/BroadVoice32 $(OBJS)

clean:
	rm -f ${OBJS}
	@echo "all .o files removed"

$(OBJDIR)/bitpack.o: $(BV32DIR)/bitpack.c $(BVCOMMONDIR)/typedef.h $(BVCOMMONDIR)/bvcommon.h $(BV32DIR)/bv32cnst.h $(BV32DIR)/bv32strct.h $(BV32DIR)/bv32.h
	$(CC) $(CFLAGS) -c $(BV32DIR)/bitpack.c

$(OBJDIR)/bv.o: $(BV32DIR)/bv.c $(BVCOMMONDIR)/typedef.h $(BVCOMMONDIR)/bvcommon.h $(BV32DIR)/bv32cnst.h $(BV32DIR)/bv32strct.h $(BV32DIR)/bv32.h $(BVCOMMONDIR)/utility.h $(BV32DIR)/g192.h $(BV32DIR)/bitpack.h $(BVCOMMONDIR)/memutil.h
	$(CC) $(CFLAGS) -c $(BV32DIR)/bv.c

$(OBJDIR)/coarptch.o: $(BV32DIR)/coarptch.c $(BVCOMMONDIR)/typedef.h $(BVCOMMONDIR)/bvcommon.h $(BV32DIR)/bv32cnst.h $(BV32DIR)/bv32externs.h $(BV32DIR)/bv32strct.h $(BVCOMMONDIR)/basop32.h ../itug191lib/basop32.h ../itug729ilib/oper_32b.h
	$(CC) $(CFLAGS) -c $(BV32DIR)/coarptch.c

$(OBJDIR)/decoder.o: $(BV32DIR)/decoder.c $(BVCOMMONDIR)/typedef.h $(BVCOMMONDIR)/bvcommon.h $(BV32DIR)/bv32cnst.h $(BV32DIR)/bv32externs.h $(BV32DIR)/bv32strct.h $(BVCOMMONDIR)/basop32.h ../itug191lib/basop32.h ../itug729ilib/oper_32b.h $(BVCOMMONDIR)/utility.h $(BVCOMMONDIR)/mathutil.h
	$(CC) $(CFLAGS) -c $(BV32DIR)/decoder.c

$(OBJDIR)/encoder.o: $(BV32DIR)/encoder.c $(BVCOMMONDIR)/typedef.h $(BVCOMMONDIR)/bvcommon.h $(BV32DIR)/bv32cnst.h $(BV32DIR)/bv32externs.h $(BV32DIR)/bv32strct.h $(BVCOMMONDIR)/basop32.h ../itug191lib/basop32.h ../itug729ilib/oper_32b.h $(BVCOMMONDIR)/utility.h
	$(CC) $(CFLAGS) -c $(BV32DIR)/encoder.c

$(OBJDIR)/excdec.o: $(BV32DIR)/excdec.c $(BVCOMMONDIR)/typedef.h $(BV32DIR)/bv32cnst.h $(BVCOMMONDIR)/basop32.h ../itug191lib/basop32.h ../itug729ilib/oper_32b.h
	$(CC) $(CFLAGS) -c $(BV32DIR)/excdec.c

$(OBJDIR)/excquan.o: $(BV32DIR)/excquan.c $(BVCOMMONDIR)/typedef.h $(BVCOMMONDIR)/bvcommon.h $(BV32DIR)/bv32cnst.h $(BVCOMMONDIR)/basop32.h ../itug191lib/basop32.h ../itug729ilib/oper_32b.h $(BVCOMMONDIR)/utility.h
	$(CC) $(CFLAGS) -c $(BV32DIR)/excquan.c

$(OBJDIR)/fineptch.o: $(BV32DIR)/fineptch.c $(BVCOMMONDIR)/typedef.h $(BVCOMMONDIR)/bvcommon.h $(BV32DIR)/bv32cnst.h $(BV32DIR)/bv32strct.h $(BVCOMMONDIR)/basop32.h ../itug191lib/basop32.h ../itug729ilib/oper_32b.h
	$(CC) $(CFLAGS) -c $(BV32DIR)/fineptch.c

$(OBJDIR)/g192.o: $(BV32DIR)/g192.c $(BVCOMMONDIR)/typedef.h $(BVCOMMONDIR)/bvcommon.h $(BV32DIR)/bv32cnst.h $(BV32DIR)/bv32strct.h
	$(CC) $(CFLAGS) -c $(BV32DIR)/g192.c

$(OBJDIR)/gaindec.o: $(BV32DIR)/gaindec.c $(BVCOMMONDIR)/typedef.h $(BVCOMMONDIR)/bvcommon.h $(BV32DIR)/bv32cnst.h $(BV32DIR)/bv32externs.h $(BVCOMMONDIR)/basop32.h ../itug191lib/basop32.h ../itug729ilib/oper_32b.h $(BVCOMMONDIR)/mathutil.h
	$(CC) $(CFLAGS) -c $(BV32DIR)/gaindec.c

$(OBJDIR)/gainquan.o: $(BV32DIR)/gainquan.c $(BVCOMMONDIR)/typedef.h $(BVCOMMONDIR)/bvcommon.h $(BV32DIR)/bv32cnst.h $(BV32DIR)/bv32externs.h $(BVCOMMONDIR)/basop32.h ../itug191lib/basop32.h ../itug729ilib/oper_32b.h $(BVCOMMONDIR)/mathutil.h
	$(CC) $(CFLAGS) -c $(BV32DIR)/gainquan.c

$(OBJDIR)/levelest.o: $(BV32DIR)/levelest.c $(BVCOMMONDIR)/typedef.h $(BV32DIR)/bv32cnst.h $(BVCOMMONDIR)/basop32.h ../itug191lib/basop32.h ../itug729ilib/oper_32b.h
	$(CC) $(CFLAGS) -c $(BV32DIR)/levelest.c

$(OBJDIR)/lspdec.o: $(BV32DIR)/lspdec.c $(BVCOMMONDIR)/typedef.h $(BVCOMMONDIR)/bvcommon.h $(BV32DIR)/bv32cnst.h $(BV32DIR)/bv32externs.h $(BVCOMMONDIR)/basop32.h ../itug191lib/basop32.h ../itug729ilib/oper_32b.h
	$(CC) $(CFLAGS) -c $(BV32DIR)/lspdec.c

$(OBJDIR)/lspquan.o: $(BV32DIR)/lspquan.c $(BVCOMMONDIR)/typedef.h $(BVCOMMONDIR)/bvcommon.h $(BV32DIR)/bv32cnst.h $(BV32DIR)/bv32externs.h $(BVCOMMONDIR)/basop32.h ../itug191lib/basop32.h ../itug729ilib/oper_32b.h
	$(CC) $(CFLAGS) -c $(BV32DIR)/lspquan.c

$(OBJDIR)/plc.o: $(BV32DIR)/plc.c $(BVCOMMONDIR)/typedef.h $(BVCOMMONDIR)/bvcommon.h $(BV32DIR)/bv32cnst.h $(BV32DIR)/bv32externs.h $(BV32DIR)/bv32strct.h $(BVCOMMONDIR)/basop32.h ../itug191lib/basop32.h ../itug729ilib/oper_32b.h $(BVCOMMONDIR)/utility.h $(BVCOMMONDIR)/mathutil.h
	$(CC) $(CFLAGS) -c $(BV32DIR)/plc.c

$(OBJDIR)/preproc.o: $(BV32DIR)/preproc.c $(BVCOMMONDIR)/typedef.h $(BVCOMMONDIR)/bvcommon.h $(BV32DIR)/bv32cnst.h $(BV32DIR)/bv32strct.h $(BV32DIR)/bv32externs.h $(BVCOMMONDIR)/basop32.h ../itug191lib/basop32.h ../itug729ilib/oper_32b.h
	$(CC) $(CFLAGS) -c $(BV32DIR)/preproc.c

$(OBJDIR)/ptquan.o: $(BV32DIR)/ptquan.c $(BVCOMMONDIR)/typedef.h $(BVCOMMONDIR)/bvcommon.h $(BV32DIR)/bv32cnst.h $(BV32DIR)/bv32strct.h $(BV32DIR)/bv32externs.h $(BVCOMMONDIR)/basop32.h ../itug191lib/basop32.h ../itug729ilib/oper_32b.h
	$(CC) $(CFLAGS) -c $(BV32DIR)/ptquan.c

$(OBJDIR)/tables.o: $(BV32DIR)/tables.c $(BVCOMMONDIR)/typedef.h
	$(CC) $(CFLAGS) -c $(BV32DIR)/tables.c

$(OBJDIR)/a2lsp.o: $(BVCOMMONDIR)/a2lsp.c $(BVCOMMONDIR)/typedef.h $(BVCOMMONDIR)/bvcommon.h $(BVCOMMONDIR)/basop32.h ../itug191lib/basop32.h ../itug729ilib/oper_32b.h $(BVCOMMONDIR)/mathutil.h $(BVCOMMONDIR)/utility.h
	$(CC) $(CFLAGS) -c $(BVCOMMONDIR)/a2lsp.c

$(OBJDIR)/allpole.o: $(BVCOMMONDIR)/allpole.c $(BVCOMMONDIR)/typedef.h $(BVCOMMONDIR)/bvcommon.h $(BVCOMMONDIR)/basop32.h ../itug191lib/basop32.h ../itug729ilib/oper_32b.h $(BVCOMMONDIR)/utility.h
	$(CC) $(CFLAGS) -c $(BVCOMMONDIR)/allpole.c

$(OBJDIR)/allzero.o: $(BVCOMMONDIR)/allzero.c $(BVCOMMONDIR)/typedef.h $(BVCOMMONDIR)/basop32.h ../itug191lib/basop32.h ../itug729ilib/oper_32b.h
	$(CC) $(CFLAGS) -c $(BVCOMMONDIR)/allzero.c

$(OBJDIR)/autocor.o: $(BVCOMMONDIR)/autocor.c $(BVCOMMONDIR)/typedef.h $(BVCOMMONDIR)/basop32.h ../itug191lib/basop32.h ../itug729ilib/oper_32b.h
	$(CC) $(CFLAGS) -c $(BVCOMMONDIR)/autocor.c

$(OBJDIR)/basop32.o: $(BVCOMMONDIR)/basop32.c $(BVCOMMONDIR)/typedef.h $(BVCOMMONDIR)/basop32.h ../itug191lib/basop32.h ../itug729ilib/oper_32b.h
	$(CC) $(CFLAGS) -c $(BVCOMMONDIR)/basop32.c

$(OBJDIR)/cmtables.o: $(BVCOMMONDIR)/cmtables.c $(BVCOMMONDIR)/typedef.h
	$(CC) $(CFLAGS) -c $(BVCOMMONDIR)/cmtables.c

$(OBJDIR)/levdur.o: $(BVCOMMONDIR)/levdur.c $(BVCOMMONDIR)/typedef.h $(BVCOMMONDIR)/bvcommon.h $(BVCOMMONDIR)/basop32.h ../itug191lib/basop32.h ../itug729ilib/oper_32b.h
	$(CC) $(CFLAGS) -c $(BVCOMMONDIR)/levdur.c

$(OBJDIR)/lsp2a.o: $(BVCOMMONDIR)/lsp2a.c $(BVCOMMONDIR)/typedef.h $(BVCOMMONDIR)/bvcommon.h $(BVCOMMONDIR)/basop32.h ../itug191lib/basop32.h ../itug729ilib/oper_32b.h $(BVCOMMONDIR)/mathutil.h
	$(CC) $(CFLAGS) -c $(BVCOMMONDIR)/lsp2a.c

$(OBJDIR)/mathtables.o: $(BVCOMMONDIR)/mathtables.c $(BVCOMMONDIR)/typedef.h
	$(CC) $(CFLAGS) -c $(BVCOMMONDIR)/mathtables.c

$(OBJDIR)/mathutil.o: $(BVCOMMONDIR)/mathutil.c $(BVCOMMONDIR)/typedef.h $(BVCOMMONDIR)/basop32.h ../itug191lib/basop32.h ../itug729ilib/oper_32b.h $(BVCOMMONDIR)/mathutil.h
	$(CC) $(CFLAGS) -c $(BVCOMMONDIR)/mathutil.c

$(OBJDIR)/memutil.o: $(BVCOMMONDIR)/memutil.c $(BVCOMMONDIR)/typedef.h
	$(CC) $(CFLAGS) -c $(BVCOMMONDIR)/memutil.c

$(OBJDIR)/ptdec.o: $(BVCOMMONDIR)/ptdec.c $(BVCOMMONDIR)/typedef.h $(BVCOMMONDIR)/bvcommon.h
	$(CC) $(CFLAGS) -c $(BVCOMMONDIR)/ptdec.c

$(OBJDIR)/stblzlsp.o: $(BVCOMMONDIR)/stblzlsp.c $(BVCOMMONDIR)/typedef.h $(BVCOMMONDIR)/bvcommon.h $(BVCOMMONDIR)/basop32.h ../itug191lib/basop32.h ../itug729ilib/oper_32b.h
	$(CC) $(CFLAGS) -c $(BVCOMMONDIR)/stblzlsp.c

$(OBJDIR)/utility.o: $(BVCOMMONDIR)/utility.c $(BVCOMMONDIR)/typedef.h
	$(CC) $(CFLAGS) -c $(BVCOMMONDIR)/utility.c

$(OBJDIR)/vqdecode.o: $(BVCOMMONDIR)/vqdecode.c $(BVCOMMONDIR)/typedef.h
	$(CC) $(CFLAGS) -c $(BVCOMMONDIR)/vqdecode.c
