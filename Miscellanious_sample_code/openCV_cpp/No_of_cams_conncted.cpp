#include "cv.h"
#include "highgui.h"
#include "cvaux.h"

int main( void )
{
    int num = 0;
    int finished = 0;
    while( !finished )
     {
         //CvCapture creates structure and allocates memory to store the frames/picture objects.
        /*CvCreateCameraCapture(num) :->takes a frame/snapshot in form of an object
            (here declared as Capture) from a camera with camera id defined by num */
        CvCapture* capture = cvCreateCameraCapture( num );
        /*When there is no capture from a camera the output is 0 ie is there is no camera.
            If a no of cameras are present the capture ends after the last camera.
            After the last capture there is no further capture.So capture becomes NULL making finished equals to 1 and  while loop ends.*/
        if( capture==NULL )
            finished = 1;
        else
                 {
                    num++;
        //CvReleaseCapture releases the memory(for storing the frame) allocated to the Capture structure by CvCapture.
                    cvReleaseCapture( &capture );
                 }
     }
    return num;
}
