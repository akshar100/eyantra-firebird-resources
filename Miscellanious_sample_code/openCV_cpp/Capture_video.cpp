/*Include all required Header Files*/

#include<opencv/cvaux.h>
#include<opencv/highgui.h>
#include<stdio.h>
#include<iostream.h>
using namespace std;

/*Start Main Function*/
int main()
{

/*Create a structure using CvCapture which is used only as a parameter for video capturing functions.
  Here capture structure has been created using CvCapture*/

    CvCapture* capture=NULL;

/*Create a Null image type pointer using IplImage function.Here  image pointer has been created having Null value */

    IplImage* image = NULL;

/*Take an interrupt flag to terminate the capture and display of image frames on raising the flag
  Here interrupt is a variable used as a flag*/

    long int interrupt=0;

/*Num is declared to indicate no of webcams connected to the system com ports*/

    int num = 0;

/*Cam_no indicate which cam is to be initialized.
  -1 signifies initially it uses the default cam*/

    int cam_no=0;

/*Finished is a flag used to terminate the count process when it comes to an end in subsequent while loop */

    int finished = 0;

/*While loop continues till there is capture from the webcams and ends when captures is NULL*/

        while( !finished )
     {
         //CvCapture creates structure and allocates memory to store the frames/picture objects.
        /*CvCreateCameraCapture(num) :->takes a frame/snapshot in form of an object
            (here declared as Capture) from a camera with camera id defined by num */
        CvCapture* captures = cvCreateCameraCapture( num );
        /*When there is no capture from a camera the output is 0 ie is there is no camera.
            If a no of cameras are present the capture ends after the last camera.
            After the last capture there is no further capture.So capture becomes NULL making finished equals to 1 and  while loop ends.*/
        if( captures==NULL )
            finished = 1;
        else
                 {
                    num++;
        //CvReleaseCapture releases the memory(for storing the frame) allocated to the Capture structure by CvCapture.
                    cvReleaseCapture( &captures );
                 }
     }

     fprintf(stdout,"There are %d webcams found\n",num);

/*End of while loop i.e. counting no of cams present*/
/*If no webcam is found then it shows an error message else it asks for option to getinitialized with the desired input cam */
    if(num==0)
        {
            fprintf(stderr,"ERROR:No Webcams Found\n");
            return 1;
        }
    if(num==1)
        {
            fprintf(stdout,"You have no choice other than to use default cam \n");
            cam_no=0;
        }
    if(num>1)
        {

            fprintf(stdout,"Enter the webcam no you choose to be initialized \n");
            fscanf(stdin,"%d",&cam_no);
            if((cam_no-1)>num)
                {
                    fprintf(stderr,"ERROR:No Webcams with such cam_no exist\n");
                    return 1;
                }

            fprintf(stdout,"You have choosen the webcam with cam_no %d\n",cam_no);
        }


/*cvCaptureFromCAM(n) initializes the cam with cam id/ index n.
  CvCapture* cvCaptureFromCAM( int index );*/

   capture = cvCaptureFromCAM(cam_no);

   if(!capture)
    {
        fprintf(stderr,"ERROR:Cannot Open Specified Webcam \n");
        return 1;
    }
    cvNamedWindow("video",CV_WINDOW_AUTOSIZE);
   while(1)
    {
        image = cvQueryFrame(capture);
        if(!image)
        break;
        cvShowImage("video",image);
        fprintf(stdout,"wow\n ");
        interrupt = cvWaitKey(1);
        fprintf(stdout,"set\n");
        if(interrupt == 27 )
            {
                fprintf(stdout,"goodluck %d \n",interrupt);
                break;
            }
        else
            fprintf(stdout,"badluck\n %d",interrupt);
    }
    cvReleaseImage( &image );
    cvDestroyWindow("video");
    cvReleaseCapture(&capture);

    return 0;
}
/*End Main Function*/



