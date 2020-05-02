/**
 * @\file   camera.cpp
 * @\author Sanju Prakash Kannioth
 * @\brief  This files contains the function definitions QR Code detection
 *          using a camera
 * @\date   05/02/2020
 * References : https://www.learnopencv.com/barcode-and-qr-code-scanner-using-zbar-and-opencv/
 *              https://www.learnopencv.com/opencv-qr-code-scanner-c-and-python/
 *
 */

#include "camera.h"
#include "main.h"


extern unsigned int motor_direction;
extern unsigned int change_direction;


extern int abortTest;
extern int abortS1, abortS2, abortS3;
extern sem_t semS1, semS2, semS3;
extern struct timeval start_time_val;

// Camera image frame object
IplImage* frame;

// Camera video capture object
VideoCapture cap; 

// QR Code scanner object
ImageScanner scanner;
Mat imGray;

// String that depicts the payload information for the robot
string payload;



/* Function to start the video capture */
int video_setup() {
    cap.open(0); 
    
    if(!cap.isOpened()){
        cout << "Error opening video stream or file" << endl;
        return -1;
    }
    return 0;
}


/* Function to scan the image for QR Codes and decode if QR code is present */
void decode(Mat &im)
{

  cvtColor(im, imGray,COLOR_BGR2GRAY);

  // Wrap image data in a zbar image
  Image image(im.cols, im.rows, "Y800", (uchar *)imGray.data, im.cols * im.rows);
  
  // Scan the image for QR Codes
  int n = scanner.scan(image);

  // Gather the decoded data from the QR Code
  for(Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol)
  {
    decodedObject obj;

    obj.type = symbol->get_type_name();
    obj.data = symbol->get_data();

    cout << "Type : " << obj.type << endl;
    cout << "Data : " << obj.data << endl << endl;

    // String stream object to parse the decoded data
    stringstream ss(obj.data);

    // Strings to store the payload and direction data
    string product, direction;

    while(ss >> product >> direction) {
            // cout << "Product : " << product << "\t" << "Direction : " << direction << endl;
            
            // Set the motor direction according to the payload
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
  }
}

/* Thread callback function for the camera task */
void *Service_3(void *threadp)
{
    struct timeval current_time_val;
    double current_time;
    unsigned long long S3Cnt=0;
    threadParams_t *threadParams = (threadParams_t *)threadp;
    double start_time, worst_time, stop_time, avg_time = 0;
    
    double timeElapsed;
    double positiveJitter = 0;  
    double jitterTime = 0;
    
    int dev=0;
        
    video_setup();
    
    payload = "Laptop";
    
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
        
        timeElapsed = stop_time;
        
        if(S3Cnt > 1) {
            if((stop_time) > worst_time)
            {
                worst_time = stop_time;	
            }
            
            jitterTime = CAMERA_DEADLINE - timeElapsed;
            if(jitterTime < 0) {
                positiveJitter -= jitterTime;
            }
        }  
    }
    
    // printf("Releasing video\n");
    // Release the video capture object
    cap.release();
    
    printf("The WCET of camera thread:: %lf\n",worst_time);
    //printf("The AVCET of camera thread:: %lf\n", (avg_time/(iteration-10)));
    printf("Cmera task jitter time = %lf\n", positiveJitter);
    pthread_exit((void *)0);
}
