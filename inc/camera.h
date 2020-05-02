/**
 * @\file   camera.h
 * @\author Sanju Prakash Kannioth
 * @\brief  This files contains the function declarations for QR Code detection
 *          using a camera
 * @\date   05/02/2020
 * References : https://www.learnopencv.com/barcode-and-qr-code-scanner-using-zbar-and-opencv/
 *              https://www.learnopencv.com/opencv-qr-code-scanner-c-and-python/
 *
 */


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


/**
--------------------------------------------------------------------------------------------
decode
--------------------------------------------------------------------------------------------
*   This function is used to get the latest frame and decode the QR Code if one is detected
*
*   @\param         im 					
*
*   @\return        void
*
*/
void decode(Mat &im);

/**
--------------------------------------------------------------------------------------------
video_setup
--------------------------------------------------------------------------------------------
*   This function is used to setup the video capture
*
*   @\param         void 					
*
*   @\return        void
*
*/
int video_setup();


/**
--------------------------------------------------------------------------------------------
Service_3
--------------------------------------------------------------------------------------------
*   Thread callback function for the camera task
*
*   @\param         threadp 					
*
*   @\return        void
*
*/
void *Service_3(void *threadp);
#endif
