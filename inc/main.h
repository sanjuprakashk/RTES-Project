/**
 * @\file   main.h
 * @\author Sanju Prakash Kannioth
 * @\brief  This files contains the declarations required for the sequencer 
 *          task and main task
 * @\date   05/02/2020
 * References : http://mercury.pr.erau.edu/~siewerts/cec450/code/
 *              http://mercury.pr.erau.edu/~siewerts/cec450/code/sequencer/
 * Note : Most of the code for the sequencer and the thread spawning have been taken directly 
 *        from the above references.
 *
 */

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

/* Macros with the deadlines in milliseconds */
#define CAMERA_DEADLINE 200
#define MOTOR_DEADLINE 20
#define ULTRASONIC_DEADLINE 50

using namespace cv;
using namespace std;
using namespace zbar;

/* Enumeration for the motor directions */
enum directions {STOP = 0, FORWARD, REVERSE, RIGHT, LEFT};


typedef struct
{
    int threadIdx;
    unsigned long long sequencePeriods;
} threadParams_t;


/**
--------------------------------------------------------------------------------------------
Sequencer
--------------------------------------------------------------------------------------------
*   Callback function for the sequencer task 
*
*   @\param         threadp 					
*
*   @\return        void
*
*/
void *Sequencer(void *threadp);


/**
--------------------------------------------------------------------------------------------
getTimeMsec
--------------------------------------------------------------------------------------------
*   Function to get the current time in milliseconds
*
*   @\param         void 					
*
*   @\return        double 		time
*
*/
double getTimeMsec(void);


/**
--------------------------------------------------------------------------------------------
print_scheduler
--------------------------------------------------------------------------------------------
*   Function to print the scheduler in use
*
*   @\param         void 					
*
*   @\return        void
*
*/
void print_scheduler(void);

#endif
