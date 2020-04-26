INCLUDE_DIRS =
LIB_DIRS =
CC=g++

CDEFS=
CFLAGS= -O0 -pg -g --std=c++11 $(INCLUDE_DIRS) $(CDEFS)
LIBS= -lrt
CPPLIBS= -L/usr/lib -lopencv_core -lopencv_flann -lopencv_video -lrt -lpthread -lzbar -lwiringPi

HFILES=
CFILES=
CPPFILES= main.cpp

SRCS= ${HFILES} ${CFILES}
CPPOBJS= ${CPPFILES:.cpp=.o}

all:    main

clean:
	-rm -f *.o *.d
	-rm -f main

distclean:
	-rm -f *.o *.d

main: main.o
	$(CC) $(LDFLAGS) $(CFLAGS) -o $@ $@.o `pkg-config --libs opencv` $(CPPLIBS)

depend:

.c.o:
	$(CC) $(CFLAGS) -c $<

.cpp.o:
	$(CC) $(CFLAGS) -c $<
