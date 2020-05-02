#ifndef MAIN_H_
#define MAIN_H_

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


#include <errno.h>

#define USEC_PER_MSEC (1000)
#define NANOSEC_PER_SEC (1000000000)
#define NUM_CPU_CORES (2)


#define NUM_THREADS (3 + 1)
#define SEQUENCER_COUNT 3000

#define CHANGE_DIRECTION_COUNT 50

using namespace cv;
using namespace std;
using namespace zbar;
using namespace std;

enum directions {STOP = 0, FORWARD, REVERSE, RIGHT, LEFT};


int abortTest=FALSE;
int abortS1=FALSE, abortS2=FALSE, abortS3=FALSE;
sem_t semS1, semS2, semS3;
struct timeval start_time_val;

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