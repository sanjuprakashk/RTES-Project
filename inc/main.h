#ifndef _MAIN_H
#define _MAIN_H

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <pthread.h>
#include <sched.h>
#include <time.h>
#include <semaphore.h>

#include <syslog.h>
#include <sys/time.h>
#include <sys/sysinfo.h>

#include <stdint.h>
#include <errno.h>


#include <opencv2/opencv.hpp>
#include <zbar.h>
#include <sstream>

#define USEC_PER_MSEC (1000)
#define NANOSEC_PER_SEC (1000000000)
#define NUM_CPU_CORES (2)


#define NUM_THREADS (3 + 1)
#define SEQUENCER_COUNT 3000

#define CHANGE_DIRECTION_COUNT 50

#define CAMERA_DEADLINE 200
#define MOTOR_DEADLINE 20
#define ULTRASONIC_DEADLINE 50

using namespace cv;
using namespace std;
using namespace zbar;

enum directions {STOP = 0, FORWARD, REVERSE, RIGHT, LEFT};



typedef struct
{
    int threadIdx;
    unsigned long long sequencePeriods;
} threadParams_t;


long unsigned int time_stamp(void);

void *Sequencer(void *threadp);

double getTimeMsec(void);

void print_scheduler(void);

#endif
