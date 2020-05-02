#include "camera.h"
#include "main.h"

extern unsigned int motor_direction;
extern unsigned int change_direction;

int video_setup() {
    cap.open(0); 
    // Check if camera opened successfully
    if(!cap.isOpened()){
        cout << "Error opening video stream or file" << endl;
        return -1;
    }
    return 0;
}



void decode(Mat &im)
{

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