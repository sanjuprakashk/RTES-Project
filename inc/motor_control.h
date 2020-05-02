#ifndef _MOTOR_CONTROL_H
#define _MOTOR_CONTROL_H

#include <wiringPi.h>

// http://wiringpi.com/pins/
#define MOTOR_A_IN1 2
#define MOTOR_A_IN2 0

#define MOTOR_B_IN1 12
#define MOTOR_B_IN2 13


void l293d_setup();

void *Service_1(void *threadp);

void motor_forward();

void motor_reverse();

void motor_right();

void motor_left();

void motor_stop();
#endif
