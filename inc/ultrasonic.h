#ifndef _ULTRASONIC_H
#define _ULTRASONIC_H

#include <wiringPi.h>

#define TRIG 5
#define ECHO 6

void ultrasonic_setup();


int getCM();


void *Service_2(void *threadp);

#endif
