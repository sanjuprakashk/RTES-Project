#ifndef _CAMERA_H
#define _CAMERA_H

#include <opencv2/opencv.hpp>
#include <zbar.h>
#include <sstream>

using namespace cv;
using namespace std;
using namespace zbar;

extern CvCapture* capture;
extern IplImage* frame;

extern VideoCapture cap; 


typedef struct
{
  string type;
  string data;
  vector <Point> location;
} decodedObject;

// Find and decode barcodes and QR codes
extern ImageScanner scanner;
extern Mat imGray;

extern string payload;

void display(Mat &im, vector<decodedObject>&decodedObjects);

void decode(Mat &im);

int video_setup();

void *Service_3(void *threadp);
#endif
