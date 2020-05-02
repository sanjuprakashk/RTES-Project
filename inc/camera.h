#ifndef CAMERA_H_
#define CAMERA_H_

#include <opencv2/opencv.hpp>
#include <zbar.h>
#include <sstream>

CvCapture* capture;
IplImage* frame;

VideoCapture cap; 


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

void display(Mat &im, vector<decodedObject>&decodedObjects);

void decode(Mat &im);

int video_setup();


void *Service_3(void *threadp);
#endif