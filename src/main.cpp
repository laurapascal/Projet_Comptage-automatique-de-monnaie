#include <opencv2/opencv.hpp>
#include <list>
#include <iterator>
#include <iostream>
#include <string>
#include <QFileDialog>
#include <QFileInfo>
#include <QFile>
#include <QString>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include <opencv2/nonfree/features2d.hpp>
#include "database.h"
#include "registration.h"

struct coin{
    cv::Point center;
    int radius;
};
int registration_coin(QString repertory_database, QString repertory_extracted_coins);

int main(int argc, char** argv)
{
    cv::Mat im, im_gray, im2, im_m1, im_m2;

    // Read the image
    im = cv::imread( argv[1], 1 );
    im2 = im.clone();
    im_m1 = im.clone();
    im_m2 = im.clone();

    if(!im.data )
    { return -1; }

    // Convert it to gray
    cvtColor( im, im_gray, CV_BGR2GRAY );

    // Reduce the noise so we avoid false circle detection
    //GaussianBlur( im_gray, im_gray, cv::Size(9, 9), 2, 2 );

    /** **************************** FIND CIRCLES - METHOD 1 **************************** **/
    cv::vector<cv::Vec3f> circles;

    /// Apply the Hough Transform to find the circles
    // im_gray: Input image (grayscale)
    // circles: A vector that stores sets of 3 values: x_{c}, y_{c}, r for each detected circle.
    // CV_HOUGH_GRADIENT: Define the detection method. Currently this is the only one available in OpenCV
    // dp = 1: The inverse ratio of resolution
    // min_dist = im_gray.rows/8: Minimum distance between detected centers
    // param_1 = 200: Upper threshold for the internal Canny edge detector
    // param_2 = 65*: Threshold for center detection.
    // min_radius = 0: Minimum radio to be detected. If unknown, put zero as default.
    // max_radius = 0: Maximum radius to be detected. If unknown, put zero as default.

    cv::HoughCircles( im_gray, circles, CV_HOUGH_GRADIENT, 1, im_gray.rows/8, 200, 40, 10, 0);

    // Vector of coins
    std::vector<coin> vector_coins_m1;

    /// Treatement of the found circles
    for( size_t i = 0; i < circles.size(); i++ )
    {
        bool draw_circle = true;
        cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);

        for ( unsigned int j = 0; j < vector_coins_m1.size(); j++)

        {
            cv::Point center_temp = vector_coins_m1[j].center;
            int radius_temp = vector_coins_m1[j].radius;

            // Removing circles drawn on other circles
            if(std::sqrt((center.x-center_temp.x)*(center.x-center_temp.x) + (center.y-center_temp.y)*(center.y-center_temp.y)) < (radius+radius_temp))
            {
                draw_circle = false;
            }

        }

        // Draw the circles
        if(draw_circle)
        {
            coin coin_detected;
            coin_detected.center = center;
            coin_detected.radius = radius;
            // circle center
            circle( im_m1, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );
            // circle outline
            circle( im_m1, center, radius, cv::Scalar(0,0,255), 3, 8, 0 );
            vector_coins_m1.push_back(coin_detected);
        }
    }

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

    /// Treatment on the found ellipses to keep only the circle and the bigger ellipses
    // IF the ellipse is not a circle THEN the ellipse is deleted
    for( unsigned int i = 0; i < minEllipse.size(); i++ )
    {
        cv::RotatedRect ellipse = minEllipse[i];
        if(std::fabs(ellipse.size.width - ellipse.size.height) > (ellipse.size.width + ellipse.size.height)/20)
        {
            minEllipse.erase(minEllipse.begin() + i);
            i--;
        }

    }
    // IF the ellipse is in an other ellipse THEN the ellipse is deleted
    for( unsigned int i = 0; i < minEllipse.size(); i++ )
    {
        cv::RotatedRect ellipse = minEllipse[i];
        float radius_ellipse = (ellipse.size.width + ellipse.size.height)/4;
        for( unsigned int j = 0; j < minEllipse.size(); j++ )
        {
            cv::RotatedRect ellipse_temp = minEllipse[j];
            float radius_ellipse_temp = (ellipse_temp.size.width + ellipse_temp.size.height)/4;
            if(i!=j)
            {
                if(std::sqrt((ellipse.center.x-ellipse_temp.center.x)*(ellipse.center.x-ellipse_temp.center.x) + (ellipse.center.y-ellipse_temp.center.y)*(ellipse.center.y-ellipse_temp.center.y)) < (radius_ellipse+radius_ellipse_temp))
                {
                    if(radius_ellipse > radius_ellipse_temp)
                    {
                        minEllipse.erase(minEllipse.begin() + j);
                        j--;
                    }

                    else
                    {
                        minEllipse.erase(minEllipse.begin() + i);
                        i--;
                    }
                }
            }
        }
    }

    /// Draw  and save the ellipses
    std::vector<coin> vector_coins_m2; // Vector of coins
    cv::Scalar color = cv::Scalar(255,0,0);
    for( unsigned int i = 0; i < minEllipse.size(); i++)
    {
        cv::RotatedRect ellipse = minEllipse[i];
        // draw the ellipse
        cv::ellipse( im_m2, minEllipse[i], color, 2, 8 );
        // save the coins
        coin coin_detected;
        coin_detected.center = ellipse.center;
        coin_detected.radius = (ellipse.size.width + ellipse.size.height)/4;
        vector_coins_m2.push_back(coin_detected);
    }

    /** ********************************************************************************* **/

    /** ******************************* EXTRACT EACH COINS ****************************** **/
    // Delete each file in the folder "output"
    QDir Dir_extracted_coins("output");
    QFileInfoList fileList_extracted_coins;
    fileList_extracted_coins.append(Dir_extracted_coins.entryInfoList());

    for( int j = 2; j < fileList_extracted_coins.size(); j++)
    {
        Dir_extracted_coins.remove(fileList_extracted_coins[j].absoluteFilePath());
    }
    // Extract each coins in a floder "output"
    for( unsigned int i = 0; i < vector_coins_m1.size(); i++ )
    {
        cv::Mat extracted_coin;
        extracted_coin = im2(cv::Rect(vector_coins_m1[i].center.x-vector_coins_m1[i].radius,vector_coins_m1[i].center.y-vector_coins_m1[i].radius,vector_coins_m1[i].radius*2,vector_coins_m1[i].radius*2));
        std::string name_extracted_coin = "output/coin" + std::to_string(i) + ".jpg";
        cv::imwrite( name_extracted_coin, extracted_coin);
    }

    /** ********************************************************************************* **/

    // Display of the two methods to find the coins
    imshow( "Method1", im_m1 );
    imshow( "Method2", im_m2 );


    /** ********************************** REGISTRATION ********************************** **/

    registration_coin("database","output");

    /** ********************************************************************************* **/

    cv::waitKey(0);
    return 0;
}

int registration_coin(QString repertory_database, QString repertory_extracted_coins)
{
    database db(repertory_database);
    registration rg("sift","flann");
    QDir Dir_extracted_coins(repertory_extracted_coins);
    QFileInfoList fileList_extracted_coins;
    fileList_extracted_coins.append(Dir_extracted_coins.entryInfoList());
    for( int i = 2; i < fileList_extracted_coins.size(); i++)
    {
        rg.creation_image_extracted_coin(fileList_extracted_coins[i].absoluteFilePath());
        std::cout<<fileList_extracted_coins[i].absoluteFilePath().toStdString()<<std::endl;

        rg.creation_keypoints_extracted_coin();
        rg.creation_descriptors_extracted_coin();


        /// Comparison with our data
        for(std::map<QString,std::string>::iterator it=db.map_data.begin() ; it!=db.map_data.end() ; ++it)
        {
            float score = 0;

            rg.creation_image_data(it->first);
            QString path = it->first;
            std::cout<<path.toStdString()<<std::endl;

            rg.creation_keypoints_data();
            rg.creation_descriptors_data();
            rg.compute_hypothetical_matches();
            cv::Mat mask = rg.findTransformation();

            // Compute the score
            for(int r = 0; r < mask.rows; r++)
            {
                for(int c = 0; c < mask.cols; c++)
                {
                    if((unsigned int)mask.at<uchar>(r,c) == 1)
                    {
                        score += 1;
                    }
                }

            }

            // Percentage
            score = 100*(score / (float)mask.rows);
            std::cout<<"score: "<<score<<std::endl;
        }
    }

    cv::waitKey(0);
    return 0;
}
