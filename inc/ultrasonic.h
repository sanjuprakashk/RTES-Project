#ifndef ULTRASONIC_H_
#define ULTRASONIC_H_

#include <wiringPi.h>

#define TRIG 5
#define ECHO 6

void ultrasonic_setup();


int getCM();


void *Service_2(void *threadp);

#endif