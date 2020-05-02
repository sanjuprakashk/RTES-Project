#include "motor_control.h"
#include "main.h"

unsigned int motor_direction = 1;
unsigned int change_direction = 0;


extern int abortTest;
extern int abortS1, abortS2, abortS3;
extern sem_t semS1, semS2, semS3;
extern struct timeval start_time_val;


void l293d_setup() {
    wiringPiSetup();
    pinMode(MOTOR_A_IN1, OUTPUT);
    pinMode(MOTOR_A_IN2, OUTPUT); 
    
    pinMode(MOTOR_B_IN1, OUTPUT);
    pinMode(MOTOR_B_IN2, OUTPUT);
}

void motor_forward() {
    digitalWrite(MOTOR_A_IN1, HIGH);
    digitalWrite(MOTOR_A_IN2, LOW);
    
    digitalWrite(MOTOR_B_IN1, HIGH);
    digitalWrite(MOTOR_B_IN2, LOW); 
}

void motor_reverse() {
    digitalWrite(MOTOR_A_IN1, LOW);
    digitalWrite(MOTOR_A_IN2, HIGH);
    
    digitalWrite(MOTOR_B_IN1, LOW);
    digitalWrite(MOTOR_B_IN2, HIGH);
}

void motor_right() {
    digitalWrite(MOTOR_A_IN1, HIGH);
    digitalWrite(MOTOR_A_IN2, LOW);
    
    digitalWrite(MOTOR_B_IN1, LOW);
    digitalWrite(MOTOR_B_IN2, HIGH);
}

void motor_left() {
    digitalWrite(MOTOR_A_IN1, LOW);
    digitalWrite(MOTOR_A_IN2, HIGH);
    
    digitalWrite(MOTOR_B_IN1, HIGH);
    digitalWrite(MOTOR_B_IN2, LOW);
}

void motor_stop() {
    digitalWrite(MOTOR_A_IN1, LOW);
    digitalWrite(MOTOR_A_IN2, LOW);
    
    digitalWrite(MOTOR_B_IN1, LOW);
    digitalWrite(MOTOR_B_IN2, LOW);
}

void *Service_1(void *threadp)
{
    struct timeval current_time_val;
    double current_time;
    unsigned long long S1Cnt=0;
    threadParams_t *threadParams = (threadParams_t *)threadp;
    
    double start_time, worst_time, stop_time, avg_time = 0;
    
    double timeElapsed;
    double positiveJitter = 0;  
    double jitterTime = 0;
    
    l293d_setup();

    gettimeofday(&current_time_val, (struct timezone *)0);
    syslog(LOG_CRIT, "Frame Sampler thread @ sec=%d, msec=%d\n", (int)(current_time_val.tv_sec-start_time_val.tv_sec), (int)current_time_val.tv_usec/USEC_PER_MSEC);
    printf("Moto thread started @ sec=%d, msec=%d\n", (int)(current_time_val.tv_sec-start_time_val.tv_sec), (int)current_time_val.tv_usec/USEC_PER_MSEC);

    while(!abortS1)
    {
        sem_wait(&semS1);
        start_time = getTimeMsec();
        S1Cnt++;
        //gettimeofday(&current_time_val, (struct timezone *)0);
        //syslog(LOG_CRIT, "Frame Sampler release %llu @ sec=%d, msec=%d\n", S1Cnt, (int)(current_time_val.tv_sec-start_time_val.tv_sec), (int)current_time_val.tv_usec/USEC_PER_MSEC);
        //printf("Frame Sampler release %llu @ sec=%d, msec=%d\n", S1Cnt, (int)(current_time_val.tv_sec-start_time_val.tv_sec), (int)current_time_val.tv_usec/USEC_PER_MSEC);
        if(change_direction == CHANGE_DIRECTION_COUNT || change_direction == 0) {
            switch(motor_direction) {
                case STOP:
                        motor_stop();
                        cout<<"Motor stop" << endl;
                        break;
                case FORWARD: 
                        motor_forward();
                        cout<<"Motor forward" << endl;
                        break;
                case REVERSE: 
                        motor_reverse();
                        cout<<"Motor reverse" << endl;
                        break;
                case RIGHT: motor_right();
                        cout<<"Motor right" << endl;
                        break;
                case LEFT: 
                        motor_left();
                        cout<<"Motor left" << endl;
                        break;
                default : break;
            //printf("Time-stamp ultrasonic sensor release %llu @ sec=%d, msec=%d\n", S2Cnt, (int)(current_time_val.tv_sec-start_time_val.tv_sec), (int)current_time_val.tv_usec/USEC_PER_MSEC);
            }
        }
        
        if(change_direction) {
            change_direction--;
        }
        
        stop_time = getTimeMsec() - start_time;
        
        timeElapsed = stop_time;
        avg_time += stop_time;
        if((stop_time) > worst_time)
        {
            worst_time = stop_time;	
        }
        
        jitterTime = MOTOR_DEADLINE - timeElapsed;
        if(jitterTime < 0) {
            positiveJitter -= jitterTime;
        }
    }
    printf("The WCET of motor thread:: %lf\n",worst_time);
    //printf("The AVCET of motor thread:: %lf\n", (avg_time/S1Cnt));
    printf("Cmera task jitter time = %lf\n", positiveJitter);
    motor_stop();
    pthread_exit((void *)0);
}
