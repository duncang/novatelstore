#########################################################
## Makefile for Novatel Store program
## $Id: Makefile 1939 2008-07-28 01:47:37Z greerd $
##
#########################################################


INCL=-I/usr/realtime/include -I/usr/src/linux/include
CFLAGS=-O2 -Wall -Wstrict-prototypes -pipe 
LIBS= -L/usr/realtime/lib -pthread -lrt

all: novatelstore novatelclient doc

novatelstore: novatelstore.c novatel.h shmdef.h novatel.o
	gcc -o novatelstore $(INCL) $(CFLAGS) -pthread -lrt novatel.o novatelstore.c 


novatelclient: novatelclient.c novatel.h shmdef.h 
	gcc -o novatelclient $(INCL) $(CFLAGS) -pthread -lrt novatelclient.c

doc: novatelstore.c novatel.h shmdef.h
	doxygen

novatel.o: novatel.c novatel.h
	gcc -o novatel.o $(CFLAGS) -c novatel.c

mac: novatel.o novatelstore.c novatel.h shmdef.h
	gcc -o novatelstore_mac $(INCL) $(CLFAGS) novatel.o novatelstore.c
	

clean:
	-rm *~ *.bak *.o novatelstore
