/*Include all required Header Files*/

#include<opencv/cvaux.h>
#include<opencv/highgui.h>
#include<stdio.h>
#include<iostream.h>
using namespace std;

/*Start Main Function*/
int main()
{
    IplImage* image = NULL;

    /*      This is defined in Highgui.h
            #define CV_LOAD_IMAGE_COLOR       1
            #define CV_LOAD_IMAGE_GRAYSCALE   0
            #define CV_LOAD_IMAGE_UNCHANGED  -1

                    IplImage* cvLoadImage( const char* filename, int iscolor=CV_LOAD_IMAGE_COLOR );
                    IplImage* cvLoadImage( "give image file location", either write 0 for CV_LOAD_IMAGE_GRAYSCALE or write 0  );

                    Similarly for colour image 1 and for unchanged -1 in place of 0.

    */

    image = cvLoadImage("D:/Users/gWaDZillA/Pictures/images.jpg",CV_LOAD_IMAGE_GRAYSCALE );

    /*              int cvNamedWindow(const char* name, int flags);
                    int cvNamedWindow("give Window name", 1 or CV_WINDOW_AUTOSIZE );

                    The user cannot size the window manually.
    */

    cvNamedWindow( "image_window", CV_WINDOW_AUTOSIZE );

    /*              void cvShowImage( const char* name, const CvArr* image );
                    void cvShowImage( "give window name", give image name to be shown );
    */

    cvShowImage("image_window", image );

    /*              int cvWaitKey( int delay=0 );
                    The function cvWaitKey waits for key event infinitely until it gets a ascii value through any key pressed.
                    cvWaitKey(n) doesnot stop the program rather delays the execution of next instruction by n milliseconds if any key is not pressed.
                    cvWaitKey() function automatically executes next insruction after specified delay.In case of cvWaitKey(0) it indefinitely waits for a key to be pressed.
                    When a key is pressed during the execution of delay the next instruction is executed immediately without any further delay.
    */

    cvWaitKey(10000);

    /*

                        void cvReleaseImage(IplImage** image)
                    {
                        cvReleaseData(*image);
                        cvReleaseImageHeader(image);
                    }

                    It deallocates the image header and the image data.
                    cvReleaseImage( address of the window assigned image );

    */
    cvReleaseImage( &image );

    /*
                    void cvDestroyWindow( const char* name );
                    i.e. cvDestroyWindow( window name to be destroyed );
    */
    cvDestroyWindow( "image_window" );
}
/*End Main Function*/
