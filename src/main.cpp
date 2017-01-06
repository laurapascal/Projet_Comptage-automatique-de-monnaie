#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
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
#include "data.h"
#include "database.h"

struct coin{
    cv::Point center;
    int radius;
};
int comparaison(QString repertory_database, QString repertory_extracted_coins);

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


    /** ********************************** COMPARAISON ********************************** **/

    comparaison("database","output");

    /** ********************************************************************************* **/

    cv::waitKey(0);
    return 0;
}

int comparaison(QString repertory_database, QString repertory_extracted_coins)
{
    database db(repertory_database);
    QDir Dir_extracted_coins(repertory_extracted_coins);
    QFileInfoList fileList_extracted_coins;
    fileList_extracted_coins.append(Dir_extracted_coins.entryInfoList());
    for( int j = 2; j < fileList_extracted_coins.size(); j++)
    {
        cv::Mat img_extracted_coin = cv::imread( fileList_extracted_coins[j].absoluteFilePath().toStdString(), CV_LOAD_IMAGE_GRAYSCALE );
        std::cout<<fileList_extracted_coins[j].absoluteFilePath().toStdString()<<std::endl;

        double max_dist = 0;
        double min_dist = 100;

        /// Comparison with our data
        for( unsigned int i = 0; i < db.vector_data.size(); i++)
        {
            cv::Mat img_database = db.vector_data[i].get_image();

            //-- Step 1: Detect the keypoints using SIFT Detector
            // SIFT( int nfeatures=0, int nOctaveLayers=3,double contrastThreshold=0.04, double edgeThreshold=10,double sigma=1.6);
            cv::SiftFeatureDetector detector( 4000, 3, 0.04, 10, 1.0 );

            // SURF(double hessianThreshold, int nOctaves=4, int nOctaveLayers=2,bool extended=true, bool upright=false);
            //            int minHessian = 100000;
            //            cv::SurfFeatureDetector detector( minHessian );

            //ORB
            //cv::OrbFeatureDetector detector;

            std::vector<cv::KeyPoint> keypoints_object, keypoints_scene;

            detector.detect( img_extracted_coin, keypoints_object );
            detector.detect( img_database, keypoints_scene );

            //-- Step 2: Calculate descriptors (feature vectors)
            cv::SiftDescriptorExtractor extractor;

            cv::Mat descriptors_object, descriptors_scene;

            extractor.compute( img_extracted_coin, keypoints_object, descriptors_object );
            extractor.compute( img_database, keypoints_scene, descriptors_scene );

            //-- Step 3: Matching descriptor vectors
            //   using FLANN matcher
            cv::FlannBasedMatcher matcher;
            //   using Brut Force matcher
//            cv::BFMatcher matcher(cv::NORM_L2);
            std::vector< cv::DMatch > matches;
            matcher.match( descriptors_object, descriptors_scene, matches );

            // Debug
//            std::cout<<"key_points_object: "<<keypoints_object.size()<<std::endl;
//            std::cout<<"descriptors_object: "<<descriptors_object.size()<<std::endl;
//            std::cout<<"keypoints_scene: "<<keypoints_scene.size()<<std::endl;
//            std::cout<<"descriptors_scene: "<<descriptors_scene.size()<<std::endl;
//            std::cout<<"matches: "<<matches.size()<<std::endl;

            //-- Quick calculation of max and min distances between keypoints
            for( int i = 0; i < descriptors_object.rows; i++ )
            { double dist = matches[i].distance;
                if( dist < min_dist ) min_dist = dist;
                if( dist > max_dist ) max_dist = dist;
            }
        }

        printf("-- Max dist : %f \n", max_dist );
        printf("-- Min dist : %f \n", min_dist );

        /// Comparison with our data
        for( unsigned int i = 0; i < db.vector_data.size(); i++)
        {
            cv::Mat img_database = db.vector_data[i].get_image();
            std::cout<<db.vector_data[i].data_path.toStdString()<<std::endl;

            //-- Step 1: Detect the keypoints using SIFT Detector
            // SIFT( int nfeatures=0, int nOctaveLayers=3,double contrastThreshold=0.04, double edgeThreshold=10,double sigma=1.6);
            cv::SiftFeatureDetector detector( 1000, 3, 0.04, 10, 1.0 );


            // SURF(double hessianThreshold, int nOctaves=4, int nOctaveLayers=2,bool extended=true, bool upright=false);
            //            int minHessian = 100000;
            //            cv::SurfFeatureDetector detector( minHessian );

            // ORB
            // cv::OrbFeatureDetector detector;

            std::vector<cv::KeyPoint> keypoints_object, keypoints_scene;

            detector.detect( img_extracted_coin, keypoints_object );
            detector.detect( img_database, keypoints_scene );

            //-- Step 2: Calculate descriptors (feature vectors)
            cv::SiftDescriptorExtractor extractor;

            cv::Mat descriptors_object, descriptors_scene;

            extractor.compute( img_extracted_coin, keypoints_object, descriptors_object );
            extractor.compute( img_database, keypoints_scene, descriptors_scene );

            // Affichage des feautures points
//            cv::Mat img_KPO;
//            cv::drawKeypoints(img_extracted_coin,keypoints_object,img_KPO, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
//            cv::imshow( "Object Key Points", img_KPO );

//            cv::Mat img_KPS;
//            cv::drawKeypoints(img_database,keypoints_scene,img_KPS, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
//            cv::imshow( "Scene Key Points", img_KPS );

            //-- Step 3: Matching descriptor vectors
            //   using FLANN matcher
            cv::FlannBasedMatcher matcher;
            //   using Brut Force matcher
//            cv::BFMatcher matcher(cv::NORM_L2);
            std::vector< cv::DMatch > matches;
            matcher.match( descriptors_object, descriptors_scene, matches );

            //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
            std::vector< cv::DMatch > good_matches;

            for( int i = 0; i < descriptors_object.rows; i++ )
            { if( matches[i].distance < 3*min_dist )
                { good_matches.push_back( matches[i]); }
            }

            std::cout<<"Good matches:"<<good_matches.size()<<std::endl;

            cv::Mat img_matches;
            cv::drawMatches( img_extracted_coin, keypoints_object, img_database, keypoints_scene,
                             good_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
                             std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

            //-- Show detected matches
            imshow( "Good Matches & Object detection", img_matches );

            cv::waitKey(0);
        }
    }

    cv::waitKey(0);
    return 0;
}
