#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <list>
#include <iterator>
#include <iostream>


/// Detection de pieces de monnaie
void detectionCoins()
{

}



int main(int argc, char** argv)
{
    cv::Mat im, im_gray;

    std::cout<<"main"<<std::endl;
    // Read the image
    im = cv::imread( argv[1], 1 );


    if(!im.data )
    { return -1; }

    // Convert it to gray
    cvtColor( im, im_gray, CV_BGR2GRAY );

    // Reduce the noise so we avoid false circle detection
    //GaussianBlur( im_gray, im_gray, cv::Size(9, 9), 2, 2 );

    cv::vector<cv::Vec3f> circles;

    // Apply the Hough Transform to find the circles
    // im_gray: Input image (grayscale)
    // circles: A vector that stores sets of 3 values: x_{c}, y_{c}, r for each detected circle.
    // CV_HOUGH_GRADIENT: Define the detection method. Currently this is the only one available in OpenCV
    // dp = 1: The inverse ratio of resolution
    // min_dist = im_gray.rows/8: Minimum distance between detected centers
    // param_1 = 200: Upper threshold for the internal Canny edge detector
    // param_2 = 100*: Threshold for center detection.
    // min_radius = 0: Minimum radio to be detected. If unknown, put zero as default.
    // max_radius = 0: Maximum radius to be detected. If unknown, put zero as default.


    HoughCircles( im_gray, circles, CV_HOUGH_GRADIENT, 1, im_gray.rows/16, 200, 100, 0, 0 );
    std::cout<<" houghcircles "<<circles.size()<<std::endl;

    // Draw the circles detected
    for( size_t i = 0; i < circles.size(); i++ )
    {
        cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);
        // circle center
        circle( im, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );
        // circle outline
        circle( im, center, radius, cv::Scalar(0,0,255), 3, 8, 0 );
    }

    imshow( "Hough Circle Transform Demo", im );
    cv::waitKey();

    return 0;
}

