REX = /rex
INC = /usr/local/include
LIB = /usr/local/lib
BIN = /usr/local/bin

PLATFORM=gcc_ntox86

CC = qcc
LD = qcc

CFLAGS  = -V$(PLATFORM) -w -O -I$(INC)
BRITLIB = -L$(LIB) -lbrit -lr3c -lsocket

PWD = $(shell pwd)
LOBJ = $(patsubst %.c,$(PWD)/%.o,$(LSRC))
$(info LOBJ is $(LOBJ))


OBJ = $(REX)/toolBar/src/gcc_ntox86/spot.d.o



all: reren

reren: reren.c
	   $(CC) $(CFLAGS) reren.c -o reren $(BRITLIB)
	   

install: reren
		 cp -f reren $(BIN)

