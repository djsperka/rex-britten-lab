# Makefile for a REX paradigm, to be used in a 
# folder outside the rex tree. 


REX = /rex
INC = /usr/local/include
LIB = /usr/local/lib


#
#	Makefile for REX spot process.
#

PLATFORM=gcc_ntox86
PHABOPTS= 
REBEL=
rf=
CC = qcc
LD = qcc


CFLAGS  = -V$(PLATFORM) -w -c -O -I$(REX)/hdr -I$(INC) -I$(INC)/nr
BRITLIB = -L$(LIB) -lbrit -lr3c -lnumrec 

H = ../hdr
SPOT = $(REX)/spot/spot
R = $(REX)/run
L = $(REX)/lib

sf = tstramp

RMSRC = rm -f $(sf).d.c
RUNSPOT = $(SPOT) $(sf).d
SRC = $(sf).d.c
COMPLETECMD = (cd ../sset; make all sf=$(sf))


PWD = $(shell pwd)
LOBJ = $(patsubst %.c,$(PWD)/%.o,$(LSRC))
$(info LOBJ is $(LOBJ))



# A single local 'extra' source file can be compiled and linked by passing the 
# basename (no .c extension and no dir - should be in sset) in the variable 
# mysrc: make sf=zzz mysrc=yyy
# This will compile yyy.c and link it with the final product. 

ifdef mysrc
LOCALSRC = $(mysrc).c
LOCALOBJ = $(mysrc).o
EXTRAOBJ = $(mysrc).o
else
LOCALSRC = 
LOCALOBJ = 
EXTRAOBJ = 
endif

ifdef mysrc2
LOCALSRC2 = $(mysrc2).c
LOCALOBJ2 = $(mysrc2).o
EXTRAOBJ += $(mysrc2).o
endif

JUNK = $(wildcard *~) $(wildcard *.o) $(wildcard *.d.c)
OUTPUT = toolBar

OBJ = $(REX)/toolBar/src/gcc_ntox86/spot.d.o



all:	Make_Pre $(OUTPUT)

Make_Pre:
	$(RMSRC)
	rm -f $(OBJ) $(LOBJ)

$(OUTPUT): $(MYOBJ) $(OBJ) $(LOBJ)
	(cd $(REX)/toolBar/src/gcc_ntox86; make shr BRITLIB="$(BRITLIB)" LOCALOBJ="$(LOBJ)"; cp toolBar $(REX)/run/$(sf))
	usemsg $(REX)/run/$(sf) $(REX)/toolBar/src/Usemsg



$(OBJ):
	$(RUNSPOT)
	$(CC) $(CFLAGS) $(SRC) -o $(OBJ)

# pattern rule for local src files - NOT the spot file

$(PWD)/%.o : %.c
	$(CC) $(CFLAGS) $< -o $@

clean:
	@echo "\nCleaning local REX build stuff...\n"
	(rm -f $(JUNK))


