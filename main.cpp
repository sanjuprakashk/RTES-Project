/* ========================================================================== */
/*                                                                            */
/*   seqgen2x.c                                                               */
// Sam Siewert, December 2017
//
// This is necessary for CPU affinity macros in Linux
//#define _GNU_SOURCE

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

#include <wiringPi.h>


#include <errno.h>

#include <opencv2/opencv.hpp>
#include <zbar.h>
#include <sstream>

#define USEC_PER_MSEC (1000)
#define NANOSEC_PER_SEC (1000000000)
#define NUM_CPU_CORES (2)


#define NUM_THREADS (3 + 1)
#define SEQUENCER_COUNT 600
#define TRIG 5
#define ECHO 6

// http://wiringpi.com/pins/
#define MOTOR_A_IN1 2
#define MOTOR_A_IN2 0

#define MOTOR_B_IN1 12
#define MOTOR_B_IN2 13

#define CHANGE_DIRECTION_COUNT 15

using namespace cv;
using namespace std;
using namespace zbar;
using namespace std;

CvCapture* capture;
IplImage* frame;

VideoCapture cap; 

int abortTest=FALSE;
int abortS1=FALSE, abortS2=FALSE, abortS3=FALSE;
sem_t semS1, semS2, semS3;
struct timeval start_time_val;

unsigned int motor_direction = 1;
unsigned int change_direction = 0;

enum directions {STOP = 0, FORWARD, REVERSE, RIGHT, LEFT};

typedef struct
{
    int threadIdx;
    unsigned long long sequencePeriods;
} threadParams_t;

typedef struct
{
  string type;
  string data;
  vector <Point> location;
} decodedObject;

// Find and decode barcodes and QR codes
ImageScanner scanner;
Mat imGray;

string payload;
void decode(Mat &im)
{

  // Create zbar scanner
  //ImageScanner scanner;

  // Configure scanner
  //scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 1);

  // Convert image to grayscale
  //Mat imGray;
  cvtColor(im, imGray,COLOR_BGR2GRAY);

  // Wrap image data in a zbar image
  Image image(im.cols, im.rows, "Y800", (uchar *)imGray.data, im.cols * im.rows);
  
  // Scan the image for barcodes and QRCodes
  int n = scanner.scan(image);

  // Print results
  for(Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol)
  {
    decodedObject obj;

    obj.type = symbol->get_type_name();
    obj.data = symbol->get_data();

    // Print type and data
    cout << "Type : " << obj.type << endl;
    cout << "Data : " << obj.data << endl << endl;

    stringstream ss(obj.data);

    string product, direction;

    while(ss >> product >> direction) {
            cout << "Product : " << product << "\t" << "Direction : " << direction << endl;
            if(product == payload && change_direction == 0) {
                change_direction = CHANGE_DIRECTION_COUNT;
                if(direction == "Forward") {
                    motor_direction = FORWARD;
                }
                else if(direction == "Left") {
                    motor_direction = LEFT;
                }
                else if(direction == "Right") {
                    motor_direction = RIGHT;
                }
                else if(direction == "Reverse") {
                    motor_direction = REVERSE;
                }
                break;
            }
    }
    // Obtain location
    /*
    for(int i = 0; i< symbol->get_location_size(); i++)
    {
      obj.location.push_back(Point(symbol->get_location_x(i),symbol->get_location_y(i)));
    }

    decodedObjects.push_back(obj);
    */
  }
}

// Display barcode and QR code location
void display(Mat &im, vector<decodedObject>&decodedObjects)
{
  // Loop over all decoded objects
  for(int i = 0; i < decodedObjects.size(); i++)
  {
    vector<Point> points = decodedObjects[i].location;
    vector<Point> hull;

    // If the points do not form a quad, find convex hull
    if(points.size() > 4)
      convexHull(points, hull);
    else
      hull = points;

    // Number of points in the convex hull
    int n = hull.size();

    for(int j = 0; j < n; j++)
    {
      line(im, hull[j], hull[ (j+1) % n], Scalar(255,0,0), 3);
    }

  }

  // Display results
  imshow("Results", im);
  waitKey(0);

}

long unsigned int time_stamp(void)
{
	struct timespec time_val;
	long unsigned int val;
	clock_gettime( CLOCK_REALTIME, &time_val);
	val = ((long unsigned int)time_val.tv_sec*1000000000)+((long unsigned int)((long unsigned int)time_val.tv_nsec));
		
	return val;
}



void ultrasonic_setup() {
        wiringPiSetup();
        pinMode(TRIG, OUTPUT);
        pinMode(ECHO, INPUT);
 
        //TRIG pin must start LOW
        digitalWrite(TRIG, LOW);
        delay(30);
}

void l293d_setup() {
    wiringPiSetup();
    pinMode(MOTOR_A_IN1, OUTPUT);
    pinMode(MOTOR_A_IN2, OUTPUT);
}

int video_setup() {
    cap.open(0); 
    // Check if camera opened successfully
    if(!cap.isOpened()){
        cout << "Error opening video stream or file" << endl;
        return -1;
    }
    return 0;
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


int getCM() {
        //Send trig pulse
        digitalWrite(TRIG, HIGH);
        delayMicroseconds(20);
        digitalWrite(TRIG, LOW);
 
        //Wait for echo start
        while(digitalRead(ECHO) == LOW);
 
        //Wait for echo end
        long startTime = micros();
        while(digitalRead(ECHO) == HIGH);
        long travelTime = micros() - startTime;
 
        //Get distance in cm
        int distance = travelTime / 58;
        printf("Obt dist in func = %d cm\n", distance);
        return distance;
}

void *Sequencer(void *threadp);

void *Service_1(void *threadp);
void *Service_2(void *threadp);
void *Service_3(void *threadp);
double getTimeMsec(void);
void print_scheduler(void);


int main()
{
    
    struct timeval current_time_val;
    int i, rc, scope;
    cpu_set_t threadcpu;
    pthread_t threads[NUM_THREADS];
    threadParams_t threadParams[NUM_THREADS];
    pthread_attr_t rt_sched_attr[NUM_THREADS];
    int rt_max_prio, rt_min_prio;
    struct sched_param rt_param[NUM_THREADS];
    struct sched_param main_param;
    pthread_attr_t main_attr;
    pid_t mainpid;
    cpu_set_t allcpuset;
    

    printf("Starting High Rate Sequencer Demo\n");
    gettimeofday(&start_time_val, (struct timezone *)0);
    gettimeofday(&current_time_val, (struct timezone *)0);
    syslog(LOG_CRIT, "START High Rate Sequencer @ sec=%d, msec=%d\n", (int)(current_time_val.tv_sec-start_time_val.tv_sec), (int)current_time_val.tv_usec/USEC_PER_MSEC);

   printf("System has %d processors configured and %d available.\n", get_nprocs_conf(), get_nprocs());

   CPU_ZERO(&allcpuset);

   for(i=0; i < NUM_CPU_CORES; i++)
       CPU_SET(i, &allcpuset);

   printf("Using CPUS=%d from total available.\n", CPU_COUNT(&allcpuset));
    
    // initialize the sequencer semaphores
    //
    if (sem_init (&semS1, 0, 0)) { printf ("Failed to initialize S1 semaphore\n"); exit (-1); }
    if (sem_init (&semS2, 0, 0)) { printf ("Failed to initialize S2 semaphore\n"); exit (-1); }
    if (sem_init (&semS3, 0, 0)) { printf ("Failed to initialize S3 semaphore\n"); exit (-1); }
    // if (sem_init (&semS4, 0, 0)) { printf ("Failed to initialize S4 semaphore\n"); exit (-1); }
    // if (sem_init (&semS5, 0, 0)) { printf ("Failed to initialize S5 semaphore\n"); exit (-1); }
    // if (sem_init (&semS6, 0, 0)) { printf ("Failed to initialize S6 semaphore\n"); exit (-1); }
    // if (sem_init (&semS7, 0, 0)) { printf ("Failed to initialize S7 semaphore\n"); exit (-1); }

    mainpid=getpid();

    rt_max_prio = sched_get_priority_max(SCHED_FIFO);
    rt_min_prio = sched_get_priority_min(SCHED_FIFO);

    rc=sched_getparam(mainpid, &main_param);
    main_param.sched_priority=rt_max_prio;
    rc=sched_setscheduler(getpid(), SCHED_FIFO, &main_param);
    if(rc < 0) perror("main_param");
    print_scheduler();


    pthread_attr_getscope(&main_attr, &scope);

    if(scope == PTHREAD_SCOPE_SYSTEM)
      printf("PTHREAD SCOPE SYSTEM\n");
    else if (scope == PTHREAD_SCOPE_PROCESS)
      printf("PTHREAD SCOPE PROCESS\n");
    else
      printf("PTHREAD SCOPE UNKNOWN\n");

    printf("rt_max_prio=%d\n", rt_max_prio);
    printf("rt_min_prio=%d\n", rt_min_prio);

    for(i=0; i < NUM_THREADS; i++)
    {

      CPU_ZERO(&threadcpu);
      CPU_SET(3, &threadcpu);

      rc=pthread_attr_init(&rt_sched_attr[i]);
      rc=pthread_attr_setinheritsched(&rt_sched_attr[i], PTHREAD_EXPLICIT_SCHED);
      rc=pthread_attr_setschedpolicy(&rt_sched_attr[i], SCHED_FIFO);
      //rc=pthread_attr_setaffinity_np(&rt_sched_attr[i], sizeof(cpu_set_t), &threadcpu);

      rt_param[i].sched_priority=rt_max_prio-i;
      pthread_attr_setschedparam(&rt_sched_attr[i], &rt_param[i]);

      threadParams[i].threadIdx=i;
    }
   
    printf("Service threads will run on %d CPU cores\n", CPU_COUNT(&threadcpu));
      
    
    // Create Service threads which will block awaiting release for:
    //

    // Servcie_1 = RT_MAX-1 @ 30 Hz
    //
    rt_param[1].sched_priority=rt_max_prio-1;
    pthread_attr_setschedparam(&rt_sched_attr[1], &rt_param[1]);

    rc=pthread_create(&threads[1],               // pointer to thread descriptor
                      &rt_sched_attr[1],         // use specific attributes
                      //(void *)0,               // default attributes
                      Service_1,                 // thread function entry point
                      (void *)&(threadParams[1]) // parameters to pass in
                     );
    if(rc < 0)
        perror("pthread_create for service 1");
    else
        printf("pthread_create successful for service 1\n");

 
    // Service_2 = RT_MAX-2 @ 10 Hz
    //
    rt_param[2].sched_priority=rt_max_prio-2;
    pthread_attr_setschedparam(&rt_sched_attr[2], &rt_param[2]);
    rc=pthread_create(&threads[2], &rt_sched_attr[2], Service_2, (void *)&(threadParams[2]));
    if(rc < 0)
        perror("pthread_create for service 2");
    else
        printf("pthread_create successful for service 2\n");
        
    // Service_3 = RT_MAX-2 @ 10 Hz
    //
    rt_param[3].sched_priority=rt_max_prio-3;
    pthread_attr_setschedparam(&rt_sched_attr[3], &rt_param[3]);
    rc=pthread_create(&threads[3], &rt_sched_attr[3], Service_3, (void *)&(threadParams[3]));
    if(rc < 0)
        perror("pthread_create for service 3");
    else
        printf("pthread_create successful for service 3\n");
        

    printf("Start sequencer\n");
    threadParams[0].sequencePeriods= SEQUENCER_COUNT;
    sem_post(&semS3);
    sleep(5); // Wait for camera init
    // Sequencer = RT_MAX   @ 100 Hz
    //
    rt_param[0].sched_priority=rt_max_prio;
    pthread_attr_setschedparam(&rt_sched_attr[0], &rt_param[0]);
    rc=pthread_create(&threads[0], &rt_sched_attr[0], Sequencer, (void *)&(threadParams[0]));
    if(rc < 0)
        perror("pthread_create for sequencer service 0");
    else
        printf("pthread_create successful for sequeencer service 0\n");


   for(i=0;i<NUM_THREADS;i++) {
       printf("Joining thread %d\n", i+1);
       pthread_join(threads[i], NULL);
   }
   printf("\nTEST COMPLETE\n");
   
   
   return 0;
}


void *Sequencer(void *threadp)
{
    struct timeval current_time_val;
    struct timespec delay_time = {0, 11111111}; // 100 Hz
    //{0,1666666};//{0, 33333333};  // delay for 16.67 msec, 60 Hz
    struct timespec remaining_time;
    double current_time;
    double residual;
    int rc, delay_cnt=0;
    unsigned long long seqCnt=0;
    threadParams_t *threadParams = (threadParams_t *)threadp;

    gettimeofday(&current_time_val, (struct timezone *)0);
    syslog(LOG_CRIT, "Sequencer thread @ sec=%d, msec=%d\n", (int)(current_time_val.tv_sec-start_time_val.tv_sec), (int)current_time_val.tv_usec/USEC_PER_MSEC);
    printf("Sequencer thread @ sec=%d, msec=%d\n", (int)(current_time_val.tv_sec-start_time_val.tv_sec), (int)current_time_val.tv_usec/USEC_PER_MSEC);

    do
    {
        delay_cnt=0; residual=0.0;
        //gettimeofday(&current_time_val, (struct timezone *)0);
        //syslog(LOG_CRIT, "Sequencer thread prior to delay @ sec=%d, msec=%d\n", (int)(current_time_val.tv_sec-start_time_val.tv_sec), (int)current_time_val.tv_usec/USEC_PER_MSEC);
        do
        {

            rc=nanosleep(&delay_time, &remaining_time);

            if(rc == EINTR)
            { 
                residual = remaining_time.tv_sec + ((double)remaining_time.tv_nsec / (double)NANOSEC_PER_SEC);

                if(residual > 0.0) printf("residual=%lf, sec=%d, nsec=%d\n", residual, (int)remaining_time.tv_sec, (int)remaining_time.tv_nsec);
 
                delay_cnt++;
            }
            else if(rc < 0)
            {
                perror("Sequencer nanosleep");
                exit(-1);
            }
           
        } while((residual > 0.0) && (delay_cnt < 100));

        seqCnt++;
        gettimeofday(&current_time_val, (struct timezone *)0);
        syslog(LOG_CRIT, "Sequencer cycle %llu @ sec=%d, msec=%d\n", seqCnt, (int)(current_time_val.tv_sec-start_time_val.tv_sec), (int)current_time_val.tv_usec/USEC_PER_MSEC);


        if(delay_cnt > 1) printf("Sequencer looping delay %d\n", delay_cnt);


        // Release each service at a sub-rate of the generic sequencer rate

        // Servcie_1 = RT_MAX-1 @ 50 Hz
        if((seqCnt % 2) == 0) sem_post(&semS1);

        // Service_2 = RT_MAX-2 @ 20 Hz
        if((seqCnt % 5) == 0) sem_post(&semS2);

        // Service_3 = RT_MAX-3 @ 5 Hz
        if((seqCnt % 20) == 0) sem_post(&semS3);

        // // Service_4 = RT_MAX-2  @ 10 Hz
        // if((seqCnt % 6) == 0) sem_post(&semS4);

        // // Service_5 = RT_MAX-3  @ 5 Hz
        // if((seqCnt % 12) == 0) sem_post(&semS5);

        // // Service_6 = RT_MAX-2  @ 10 Hz
        // if((seqCnt % 6) == 0) sem_post(&semS6);

        // // Service_7 = RT_MIN    1 Hz
        // if((seqCnt % 60) == 0) sem_post(&semS7);

        //gettimeofday(&current_time_val, (struct timezone *)0);
        //syslog(LOG_CRIT, "Sequencer release all sub-services @ sec=%d, msec=%d\n", (int)(current_time_val.tv_sec-start_time_val.tv_sec), (int)current_time_val.tv_usec/USEC_PER_MSEC);

    } while(!abortTest && (seqCnt < threadParams->sequencePeriods));

    sem_post(&semS1); sem_post(&semS2); 
    sem_post(&semS3);
    // sem_post(&semS4); sem_post(&semS5); sem_post(&semS6);
    // sem_post(&semS7);
    abortS1=TRUE; abortS2=TRUE; abortS3=TRUE;

    pthread_exit((void *)0);
}



void *Service_1(void *threadp)
{
    struct timeval current_time_val;
    double current_time;
    unsigned long long S1Cnt=0;
    threadParams_t *threadParams = (threadParams_t *)threadp;
    
    double start_time, worst_time, stop_time, avg_time = 0;
    
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
        avg_time += stop_time;
        if((stop_time) > worst_time)
        {
            worst_time = stop_time;	
        }    
    }
    printf("The WCET of motor thread:: %lf\n",worst_time);
    //printf("The AVCET of motor thread:: %lf\n", (avg_time/S1Cnt));
    motor_stop();
    pthread_exit((void *)0);
}


void *Service_2(void *threadp)
{
    struct timeval current_time_val;
    double current_time;
    unsigned long long S2Cnt=0;
    int distance = 0;
    threadParams_t *threadParams = (threadParams_t *)threadp;
    double start_time, worst_time, stop_time, avg_time = 0;
    
    ultrasonic_setup();
    

    gettimeofday(&current_time_val, (struct timezone *)0);
    syslog(LOG_CRIT, "Time-stamp with Image Analysis thread @ sec=%d, msec=%d\n", (int)(current_time_val.tv_sec-start_time_val.tv_sec), (int)current_time_val.tv_usec/USEC_PER_MSEC);
    printf("Ultrasonic thread started @ sec=%d, msec=%d\n", (int)(current_time_val.tv_sec-start_time_val.tv_sec), (int)current_time_val.tv_usec/USEC_PER_MSEC);
    
    while(!abortS2)
    {
        sem_wait(&semS2);
        start_time = getTimeMsec();
        distance = getCM();
        
        gettimeofday(&current_time_val, (struct timezone *)0);
        //syslog(LOG_CRIT, "Time-stamp with Image Analysis release %llu @ sec=%d, msec=%d\n", S2Cnt, (int)(current_time_val.tv_sec-start_time_val.tv_sec), (int)current_time_val.tv_usec/USEC_PER_MSEC);
        //printf("Time-stamp with Image Analysis release %llu @ sec=%d, msec=%d\n", S2Cnt, (int)(current_time_val.tv_sec-start_time_val.tv_sec), (int)current_time_val.tv_usec/USEC_PER_MSEC);
        syslog(LOG_CRIT, "Time-stamp ultrasonic sensor release %llu @ sec=%d, msec=%d\n", S2Cnt, (int)(current_time_val.tv_sec-start_time_val.tv_sec), (int)current_time_val.tv_usec/USEC_PER_MSEC);
        printf("Distance = %d\n", distance);
        
        if(distance < 10) {
            motor_direction = STOP;
        }
        else {
            motor_direction = FORWARD;
        }
        //printf("Time-stamp ultrasonic sensor release %llu @ sec=%d, msec=%d\n", S2Cnt, (int)(current_time_val.tv_sec-start_time_val.tv_sec), (int)current_time_val.tv_usec/USEC_PER_MSEC);
    
        stop_time = getTimeMsec() - start_time;
        avg_time += stop_time;
        
        if((stop_time) > worst_time)
        {
            worst_time = stop_time;	
        }  
    }
    printf("The WCET of ultrasonic thread:: %lf\n",worst_time);
    //printf("The AVCET of ultrasonic thread:: %lf\n", (avg_time/S2Cnt));

    pthread_exit((void *)0);
}



void *Service_3(void *threadp)
{
    struct timeval current_time_val;
    double current_time;
    unsigned long long S3Cnt=0;
    threadParams_t *threadParams = (threadParams_t *)threadp;
    double start_time, worst_time, stop_time, avg_time = 0;
    
    int dev=0;
        
    video_setup();
    
    payload = "Laptop";
    
    //VideoCapture cap(0);
    
    scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 1);

    gettimeofday(&current_time_val, (struct timezone *)0);
    syslog(LOG_CRIT, "Camera thread @ sec=%d, msec=%d\n", (int)(current_time_val.tv_sec-start_time_val.tv_sec), (int)current_time_val.tv_usec/USEC_PER_MSEC);
    printf("Camera thread started @ sec=%d, msec=%d\n", (int)(current_time_val.tv_sec-start_time_val.tv_sec), (int)current_time_val.tv_usec/USEC_PER_MSEC);

    while(!abortS3)
    {
        sem_wait(&semS3);
        start_time = getTimeMsec();
        gettimeofday(&current_time_val, (struct timezone *)0);
        syslog(LOG_CRIT, "Time-stamp motor service %llu @ sec=%d, msec=%d\n", S3Cnt, (int)(current_time_val.tv_sec-start_time_val.tv_sec), (int)current_time_val.tv_usec/USEC_PER_MSEC);
        S3Cnt++;
        printf("Camera thread count = %lld\n", S3Cnt);
        
        Mat frame;
        cap >> frame;
        
        if(frame.empty())  {
            break;
        }
        
        decode(frame);
        
        stop_time = getTimeMsec() - start_time;
        avg_time += stop_time;
        
        if(S3Cnt > 1) {
            if((stop_time) > worst_time)
            {
                worst_time = stop_time;	
            }
        }  
    }
    
    printf("Releasing video\n");
    // When everything done, release the video capture object
    cap.release();
    
    printf("The WCET of camera thread:: %lf\n",worst_time);
    //printf("The AVCET of camera thread:: %lf\n", (avg_time/(iteration-10)));
    pthread_exit((void *)0);
}


double getTimeMsec(void)
{
  struct timespec event_ts = {0, 0};

  clock_gettime(CLOCK_MONOTONIC, &event_ts);
  return ((event_ts.tv_sec)*1000.0) + ((event_ts.tv_nsec)/1000000.0);
}


void print_scheduler(void)
{
   int schedType;

   schedType = sched_getscheduler(getpid());

   switch(schedType)
   {
       case SCHED_FIFO:
           printf("Pthread Policy is SCHED_FIFO\n");
           break;
       case SCHED_OTHER:
           printf("Pthread Policy is SCHED_OTHER\n"); exit(-1);
         break;
       case SCHED_RR:
           printf("Pthread Policy is SCHED_RR\n"); exit(-1);
           break;
       default:
           printf("Pthread Policy is UNKNOWN\n"); exit(-1);
   }
}

