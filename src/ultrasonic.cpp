/**
 * @\file   ultrasonic.cpp
 * @\author Sorabh Gandhi, Sanju Prakash Kannioth
 * @\brief  This files contains the function definitions for the ultrasonic sensor
 * @\date   05/02/2020
 * References : http://wiringpi.com/pins/
 *              https://www.piprojects.xyz/ultrasonic-distance-sensor/
 *              
 */

#include "ultrasonic.h"
#include "main.h"

extern unsigned int motor_direction;
extern unsigned int change_direction;


extern int abortTest;
extern int abortS1, abortS2, abortS3;
extern sem_t semS1, semS2, semS3;
extern struct timeval start_time_val;

/* Function to setup the ultrasonic sensor */
void ultrasonic_setup() {
        wiringPiSetup();
        pinMode(TRIG, OUTPUT);
        pinMode(ECHO, INPUT);
 
        // Trigger pin should start LOW
        digitalWrite(TRIG, LOW);
        delay(30);
}

/* Function to get the distance in cm */
int get_dist_in_cm() {
    int dist;
    // Start the trigger pulse
    digitalWrite(TRIG, HIGH);
    delayMicroseconds(20);
    // Stop the triiger pulse
    digitalWrite(TRIG, LOW);

    while(digitalRead(ECHO) == LOW);

    // Block until the echo line receives the signal
    long startTime = micros();
    while(digitalRead(ECHO) == HIGH);
    long travelTime = micros() - startTime;

    // Convert the distance to cm
    dist = travelTime / 58;
    // printf("Obt dist in func = %d cm\n", dist);
    return dist;
}

/* Thread callback function for the ultrasonic task */
void *Service_2(void *threadp)
{
    struct timeval current_time_val;
    double current_time;
    unsigned long long S2Cnt=0;
    int distance = 0;
    threadParams_t *threadParams = (threadParams_t *)threadp;
    double start_time, worst_time, stop_time, avg_time = 0;
    
    double timeElapsed;
    double positiveJitter = 0;  
    double jitterTime = 0;
    
    ultrasonic_setup();
    

    gettimeofday(&current_time_val, (struct timezone *)0);
    syslog(LOG_CRIT, "Time-stamp with Ultrasonic thread thread @ sec=%d, msec=%d\n", (int)(current_time_val.tv_sec-start_time_val.tv_sec), (int)current_time_val.tv_usec/USEC_PER_MSEC);
    printf("Ultrasonic thread started @ sec=%d, msec=%d\n", (int)(current_time_val.tv_sec-start_time_val.tv_sec), (int)current_time_val.tv_usec/USEC_PER_MSEC);
    
    while(!abortS2)
    {
        sem_wait(&semS2);
        start_time = getTimeMsec();
        distance = get_dist_in_cm();
        
        gettimeofday(&current_time_val, (struct timezone *)0);
        //syslog(LOG_CRIT, "Time-stamp with ultrasonic release %llu @ sec=%d, msec=%d\n", S2Cnt, (int)(current_time_val.tv_sec-start_time_val.tv_sec), (int)current_time_val.tv_usec/USEC_PER_MSEC);
        //printf("Time-stamp with ultrasonic release %llu @ sec=%d, msec=%d\n", S2Cnt, (int)(current_time_val.tv_sec-start_time_val.tv_sec), (int)current_time_val.tv_usec/USEC_PER_MSEC);
        syslog(LOG_CRIT, "Time-stamp ultrasonic sensor release %llu @ sec=%d, msec=%d\n", S2Cnt, (int)(current_time_val.tv_sec-start_time_val.tv_sec), (int)current_time_val.tv_usec/USEC_PER_MSEC);
        printf("Distance = %d\n", distance);
        
        /* Logic to stop the motor if an obstacle is detected */
        if(distance < 10) {
            motor_direction = STOP;
            change_direction = 0;
        }
        else if(!change_direction) {
            motor_direction = FORWARD;
        }
        //printf("Time-stamp ultrasonic sensor release %llu @ sec=%d, msec=%d\n", S2Cnt, (int)(current_time_val.tv_sec-start_time_val.tv_sec), (int)current_time_val.tv_usec/USEC_PER_MSEC);
    
        stop_time = getTimeMsec() - start_time;
        
        timeElapsed = stop_time;
        avg_time += stop_time;
        
        /* Logic to calculate the Worst case execution time */
        if((stop_time) > worst_time)
        {
            worst_time = stop_time;	
        }  
        
        /* Jitter calculation */
        jitterTime = ULTRASONIC_DEADLINE - timeElapsed;
        if(jitterTime < 0) {
            positiveJitter -= jitterTime;
        }
    }
    printf("The WCET of ultrasonic thread:: %lf\n",worst_time);
    //printf("The AVCET of ultrasonic thread:: %lf\n", (avg_time/S2Cnt));
    printf("Ultrasonic task jitter time = %lf\n", positiveJitter);
    syslog(LOG_CRIT, "The WCET of ultrasonic thread:: %lf\n",worst_time);
    syslog(LOG_CRIT, "Ultrasonic task jitter time = %lf\n", positiveJitter);
    
    pthread_exit((void *)0);
}
