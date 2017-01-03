#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <list>
#include <iterator>
#include <iostream>
#include <string>

struct coin{
    cv::Point center;
    int radius;
};

int main(int argc, char** argv)
{
    cv::Mat im, im_gray, im2;

    // Read the image
    im = cv::imread( argv[1], 1 );
    im2 = im.clone();

    if(!im.data )
    { return -1; }

    // Convert it to gray
    cvtColor( im, im_gray, CV_BGR2GRAY );

    // Reduce the noise so we avoid false circle detection
    //GaussianBlur( im_gray, im_gray, cv::Size(9, 9), 2, 2 );

    /** **************************** FIND CIRCLES - METHOD 1 **************************** **/
//    cv::vector<cv::Vec3f> circles;

//    /// Apply the Hough Transform to find the circles
//    // im_gray: Input image (grayscale)
//    // circles: A vector that stores sets of 3 values: x_{c}, y_{c}, r for each detected circle.
//    // CV_HOUGH_GRADIENT: Define the detection method. Currently this is the only one available in OpenCV
//    // dp = 1: The inverse ratio of resolution
//    // min_dist = im_gray.rows/8: Minimum distance between detected centers
//    // param_1 = 200: Upper threshold for the internal Canny edge detector
//    // param_2 = 65*: Threshold for center detection.
//    // min_radius = 0: Minimum radio to be detected. If unknown, put zero as default.
//    // max_radius = 0: Maximum radius to be detected. If unknown, put zero as default.

//    cv::HoughCircles( im_gray, circles, CV_HOUGH_GRADIENT, 1, im_gray.rows/8, 200, 40, 10, 0);

//    // Vector of coins
//    std::vector<coin> vector_coins;

//    /// Treatement of the found circles
//    for( size_t i = 0; i < circles.size(); i++ )
//    {
//        bool draw_circle = true;
//        cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
//        int radius = cvRound(circles[i][2]);

//        for ( unsigned int j = 0; j < vector_coins.size(); j++)

//        {
//            cv::Point center_temp = vector_coins[j].center;
//            int radius_temp = vector_coins[j].radius;

//            // Removing circles drawn on other circles
//            if(std::sqrt((center.x-center_temp.x)*(center.x-center_temp.x) + (center.y-center_temp.y)*(center.y-center_temp.y)) < (radius+radius_temp))
//            {
//                draw_circle = false;
//            }

//        }

//        // Draw the circles
//        if(draw_circle)
//        {
//            coin coin_detected;
//            coin_detected.center = center;
//            coin_detected.radius = radius;
//            // circle center
//            circle( im, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );
//            // circle outline
//            circle( im, center, radius, cv::Scalar(0,0,255), 3, 8, 0 );
//            vector_coins.push_back(coin_detected);
//        }
//    }

    /** ********************************************************************************* **/


    /** **************************** FIND CIRCLES - METHOD 2 **************************** **/
    cv::Mat threshold_output;
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    int thresh = 200;

    cv::blur( im_gray, im_gray, cv::Size(3,3) );

    /// Detect edges using Threshold
    cv::threshold( im_gray, threshold_output, thresh, 255, cv::THRESH_BINARY );
    /// Find contours
    cv::findContours( threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

    /// Find the rotated ellipses for each contour
    std::vector<cv::RotatedRect> minEllipse;
    for( unsigned int i = 0; i < contours.size(); i++ )
    {
        if( contours[i].size() > 20 )
            minEllipse.push_back(fitEllipse( cv::Mat(contours[i]) ));
    }

    /// Draw ellipses and save them
    // Vector of coins
    std::vector<coin> vector_coins;
    for( unsigned int i = 0; i < minEllipse.size(); i++ )
    {
        cv::Scalar color = cv::Scalar(255,0,0);
        cv::Size2f sizeEllipse = minEllipse[i].size;
        if(std::fabs(sizeEllipse.width - sizeEllipse.height) < (sizeEllipse.width + sizeEllipse.height)/20)
        {
            ellipse( im, minEllipse[i], color, 2, 8 );
            // save the coins
            coin coin_detected;
            coin_detected.center = minEllipse[i].center;
            coin_detected.radius = (sizeEllipse.width + sizeEllipse.height)/4;
            vector_coins.push_back(coin_detected);
        }
    }

    /** ********************************************************************************* **/

    /// Extract each coins
    for( unsigned int i = 0; i < vector_coins.size(); i++ )
    {
        cv::Mat extrated_coin;
        extrated_coin = im2(cv::Rect(vector_coins[i].center.x-vector_coins[i].radius,vector_coins[i].center.y-vector_coins[i].radius,vector_coins[i].radius*2,vector_coins[i].radius*2));
        std::string name_extrated_coin = "output/image_test" + std::to_string(i) + ".jpg";
        cv::imwrite( name_extrated_coin, extrated_coin);
    }

    imshow( "Hough Circle Transform Demo", im );
    cv::waitKey(0);
    return 0;
}

