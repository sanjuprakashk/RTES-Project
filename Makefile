INCLUDE_DIRS = -Iinc/
LIB_DIRS =
CC=g++

CDEFS=
CFLAGS= -O0 -pg -g --std=c++11 $(INCLUDE_DIRS) $(CDEFS)
LIBS= -lrt
CPPLIBS= -L/usr/lib -lopencv_core -lopencv_flann -lopencv_video -lrt -lpthread -lzbar -lwiringPi

CPPFILES= camera.cpp motor_control.cpp ultrasonic.cpp main.cpp

CPPOBJS= ${CPPFILES:.cpp=.o}

vpath %.cpp src/

all:  $(CPPOBJS)
	$(CC) $(LDFLAGS) $(CFLAGS) $(CPPOBJS) -o main `pkg-config --libs opencv` $(CPPLIBS)

clean:
	-rm -f *.o *.d
	-rm -f main *.out

distclean:
	-rm -f *.o *.d

depend:

.c.o:
	$(CC) $(CFLAGS) -c $<

.cpp.o:
	$(CC) $(CFLAGS) -c $<
