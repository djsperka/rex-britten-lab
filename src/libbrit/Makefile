#	Makefile for Britten lab library libbrit.a
# Modified 5-26-15 djs to work outside the rex directory structure.
# Will rely on the REX structure for headers, but otherwise
# will live an independent life elsewhere. 


#########################################################################
# Configuration - must set the path to the rex folder for the machine you 
# are on.

REX = /rex

# End configuration - do not change stuff below here!
#########################################################################


# commands

RM = /bin/rm -f
MKDIR = /bin/mkdir -p
CP = /bin/cp



# compiler stuff

#ifndef DEBUG
#DEBUG = -g
#endif

ifndef OPTIM
OPTIM = -O
endif

PLATFORM=gcc_ntox86

# paths to various places in the directory tree

REXLIB = $(REX)/lib
REXHDR = $(REX)/hdr

BRITLIB = ../../lib
BRITHDR = ../../include

INSTALLDIR = /usr/local

CC = qcc
LD = qcc
CFLAGS  = -V$(PLATFORM) -w -c $(OPTIM) $(DEBUG) -I ../$(BRITHDR) -I $(REXHDR)

OUTPUT = libbrit.a

OBJ = bcode.o spinq.o slider.o randomtrialgenerator.o ivunif.o eyepos.o r3p.o
HEADERS = $(wildcard *.h)
JUNK = $(wildcard *~)

all:	$(OUTPUT)

copyfiles:
	$(CP) $(HEADERS) $(INSTALLDIR)

clean:
	  $(RM) $(OBJ) $(OUTPUT) $(JUNK)

install: $(OUTPUT)
	$(MKDIR) $(INSTALLDIR)/include
	$(CP) $(OUTPUT) $(INSTALLDIR)/lib
	$(CP) $(HEADERS) $(INSTALLDIR)/include

$(OUTPUT):	$(OBJ)
	$(CC) $(OBJ) -A $(OUTPUT)

bcode.o: bcode.c bcode.h $(REXHDR)/buf.h
	$(CC) $(CFLAGS) bcode.c

spinq.o: spinq.c spinq.h
	$(CC) $(CFLAGS) spinq.c

slider.o: slider.c slider.h
	$(CC) $(CFLAGS) slider.c

randomtrialgenerator.o: randomtrialgenerator.c randomtrialgenerator.h
	$(CC) $(CFLAGS) randomtrialgenerator.c

ivunif.o: ivunif.c ivunif.h
	$(CC) $(CFLAGS) ivunif.c

eyepos.o: eyepos.c eyepos.h
	$(CC) $(CFLAGS) eyepos.c

r3p.o: r3p.c r3p.h
	$(CC) $(CFLAGS) r3p.c

.PHONY: dirs clean
