#include <cvaux.h>
#include <highgui.h>
#include <cxcore.h>
#include <stdio.h>
#include<iostream>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <math.h>
#include <float.h>
#include <limits.h>
#include <time.h>
#include <ctype.h>
using namespace std;
int main(int argc, char* argv[])
 {
IplImage* frame = NULL;

     CvCapture* capture = cvCaptureFromCAM( 0 );

     if( !capture )
     {
             fprintf( stderr, "ERROR: capture is NULL \n" );
             getchar();
             return -1;
     }

    cvNamedWindow("video", CV_WINDOW_AUTOSIZE);
    cvNamedWindow("colordetect", CV_WINDOW_AUTOSIZE);

    CvScalar hsv_min = cvScalar(0,130,200,0);
    CvScalar hsv_max =  cvScalar(22,210,230,0);

    frame = cvQueryFrame( capture );
    CvSize size = cvGetSize(frame);

     IplImage*  hsv_frame    = cvCreateImage(size, IPL_DEPTH_8U, 3);

     IplImage*  thresholded   = cvCreateImage(size, IPL_DEPTH_8U, 1);


 while(1)
     {

        //capture image from cam
         frame = cvQueryFrame( capture );

            if( (cvWaitKey(5) & 255) == 27 ) break;

         if( !frame )
         {
                 fprintf( stderr, "ERROR: frame is null...\n" );
                 getchar();
                 break;
         }

        //conversion of color model from rgb to hsv
        cvCvtColor(frame, hsv_frame, CV_BGR2HSV);

        //color detection
        cvInRangeS(hsv_frame, hsv_min, hsv_max, thresholded);

         // Memory for hough circles
         CvMemStorage* storage = cvCreateMemStorage(0);

        //Smoothing of image as cvHoughCircles works better on smoothed image
         cvSmooth( thresholded, thresholded, CV_GAUSSIAN, 11, 11 );

        //shape detection-circle shape
         CvSeq* circles = cvHoughCircles(thresholded, storage, CV_HOUGH_GRADIENT, 2,thresholded->height/6, 100, 50, 10, 400);

         cout<<"no. of circles="<<circles->total;

         //drawing circle around the detected circle
         for (int i = 0; i < circles->total; i++)
         {
             float* p = (float*)cvGetSeqElem( circles, i );
             cout<<"x= "<<p[0]<<"y= "<<p[1]<<"r= "<<p[2]<<endl;
             cvCircle( frame, cvPoint(cvRound(p[0]),cvRound(p[1])),cvRound(p[2]), CV_RGB(0,255,0), 3, 8, 0 );
         }

    //display image showing ball detected
 cvShowImage("video", frame);

    //color detected image
    cvShowImage("colordetect", thresholded);

    //release memry storage
    cvReleaseMemStorage(&storage);

    //wait for 5 sec and continue or break if "ESC"is pressed
    if( (cvWaitKey(5) & 255) == 27 ) break;
     }
      //release capture
      cvReleaseCapture( &capture );

      cvDestroyWindow( "mywindow" );
      return 0;
    }

