#include "circleDetection.hpp"

circleDetection::circleDetection(QString path_initial_image_for_detection)
{
    initial_image_for_detection = cv::imread( path_initial_image_for_detection.toStdString(), 1 );
    assert(initial_image_for_detection.data);
    curent_image_for_detection = initial_image_for_detection.clone();
}

/** ********************************************************************************* **/
/** ****************************** Pre-treatment  *********************************** **/
/** ********************************************************************************* **/

void circleDetection::preTreatment(bool blur)
{
    // Convert it to gray
    cvtColor( curent_image_for_detection, curent_image_for_detection, CV_BGR2GRAY );

    // Reduce the noise so we avoid false circle detection
    if(blur)
        GaussianBlur( curent_image_for_detection, curent_image_for_detection, cv::Size(9, 9), 2, 2 );
}

void circleDetection::backgroundSegmantation(bool display)
{
    // define bounding rectangle
    int col = curent_image_for_detection.cols;
    col *= 0.01;
    cv::Rect rectangle(col,col,curent_image_for_detection.cols-col*2,curent_image_for_detection.rows-col*2);

    cv::Mat result; // segmentation result (4 possible values)
    cv::Mat bgModel,fgModel; // the models (internally used)

    // GrabCut segmentation
    cv::grabCut(curent_image_for_detection,    // input image
                result,   // segmentation result
                rectangle,// rectangle containing foreground
                bgModel,fgModel, // models
                2,        // number of iterations
                cv::GC_INIT_WITH_RECT); // use rectangle

    // Get the pixels marked as likely foreground
    cv::compare(result,cv::GC_PR_FGD,result,cv::CMP_EQ);
    // Generate output image
    cv::Mat foreground(curent_image_for_detection.size(),CV_8UC3,cv::Scalar(255,0,255));
    cv::Mat background(curent_image_for_detection.size(),CV_8UC3,cv::Scalar(255,0,255));
    curent_image_for_detection.copyTo(background,~result);
    curent_image_for_detection.copyTo(foreground,result);

    if(display)
    {
        //Saving the result for debugging
        cv::Mat rect = curent_image_for_detection.clone();
        cv::rectangle(rect, rectangle, cv::Scalar(255,0,0),1);
        cv::imshow("initial rectangle",rect);
        cv::imshow("Foreground",foreground);
        cv::imshow("Background",background);
    }
    cv::Mat zero(curent_image_for_detection.size(),CV_8UC3,cv::Scalar(0,0,0));
    cv::Mat one(curent_image_for_detection.size(),CV_8UC3,cv::Scalar(255,255,255));
    one.copyTo(zero,result);
    curent_image_for_detection = zero.clone();
}

/** ********************************************************************************* **/
/** ****************** Detection with method 1 or method 2  ************************* **/
/** ********************************************************************************* **/

cv::vector<cv::Vec3f> circleDetection::technique1()
{
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

    cv::HoughCircles( curent_image_for_detection, circles, CV_HOUGH_GRADIENT, 1, curent_image_for_detection.rows/8, 200, 40, 10, 0);
    return circles;
}

std::vector<cv::RotatedRect> circleDetection::technique2()
{
    cv::Mat threshold_output;
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    int thresh = 125;

    // Detect edges using Threshold
    cv::threshold( curent_image_for_detection, threshold_output, thresh, 255, cv::THRESH_BINARY );
    //Affichage de seuillage pour debug
    imshow( "Seuillage", threshold_output );
    // Find contours
    cv::findContours( threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

    // Find the rotated ellipses for each contour
    std::vector<cv::RotatedRect> ellipses;
    for( unsigned int i = 0; i < contours.size(); i++ )
    {
        if( contours[i].size() > 20 )
            ellipses.push_back(fitEllipse( cv::Mat(contours[i]) ));
    }

    return ellipses;
}

/** ********************************************************************************* **/
/** ***************************** Post-treatment  *********************************** **/
/** ********************************************************************************* **/


void circleDetection::post_treatment_technique1(cv::vector<cv::Vec3f> circles)
{
    /// Treatement of the found circles
    for( size_t i = 0; i < circles.size(); i++ )
    {
        bool keep_circle = true;
        cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);

        for ( unsigned int j = 0; j < vector_coins.size(); j++)

        {
            cv::Point center_temp = vector_coins[j].center;
            int radius_temp = vector_coins[j].radius;

            // Removing circles drawn on other circles
            if(std::sqrt((center.x-center_temp.x)*(center.x-center_temp.x) + (center.y-center_temp.y)*(center.y-center_temp.y)) < (radius+radius_temp))
            {
                keep_circle = false;
            }

        }
        if(keep_circle)
        {
            coin coin_detected;
            coin_detected.center = center;
            coin_detected.radius = radius;
            vector_coins.push_back(coin_detected);
        }
    }
}

void circleDetection::post_treatment_technique2(std::vector<cv::RotatedRect> ellipses)
{
    /// Treatment on the found ellipses to keep only the circle and the bigger ellipses
    // IF the ellipse is not a circle THEN the ellipse is deleted
    for( unsigned int i = 0; i < ellipses.size(); i++ )
    {
        cv::RotatedRect ellipse = ellipses[i];
        if(std::fabs(ellipse.size.width - ellipse.size.height) > (ellipse.size.width + ellipse.size.height)/20)
        {
            ellipses.erase(ellipses.begin() + i);
            i--;
        }

    }
    // IF the ellipse is in an other ellipse THEN the ellipse is deleted
    for( unsigned int i = 0; i < ellipses.size(); i++ )
    {
        cv::RotatedRect ellipse = ellipses[i];
        float radius_ellipse = (ellipse.size.width + ellipse.size.height)/4;
        for( unsigned int j = 0; j < ellipses.size(); j++ )
        {
            cv::RotatedRect ellipse_temp = ellipses[j];
            float radius_ellipse_temp = (ellipse_temp.size.width + ellipse_temp.size.height)/4;
            if(i!=j)
            {
                if(std::sqrt((ellipse.center.x-ellipse_temp.center.x)*(ellipse.center.x-ellipse_temp.center.x) + (ellipse.center.y-ellipse_temp.center.y)*(ellipse.center.y-ellipse_temp.center.y)) < (radius_ellipse+radius_ellipse_temp))
                {
                    if(radius_ellipse > radius_ellipse_temp)
                    {
                        ellipses.erase(ellipses.begin() + j);
                        j--;
                    }

                    else
                    {
                        ellipses.erase(ellipses.begin() + i);
                        i--;
                    }
                }
            }
        }
    }

    for( unsigned int i = 0; i < ellipses.size(); i++)
    {
        cv::RotatedRect ellipse = ellipses[i];
        // save the coins
        coin coin_detected;
        coin_detected.center = ellipse.center;
        coin_detected.radius = (ellipse.size.width + ellipse.size.height)/4;
        vector_coins.push_back(coin_detected);
    }
}

/** ********************************************************************************* **/
/** ************************* Exxtraction des pieces  ******************************* **/
/** ********************************************************************************* **/

void circleDetection::extraction(QDir Dir_extracted_coins)
{
    // Clearing the output floder
    QFileInfoList fileList_extracted_coins;
    fileList_extracted_coins.append(Dir_extracted_coins.entryInfoList());
    for( int j = 2; j < fileList_extracted_coins.size(); j++)
    {
        Dir_extracted_coins.remove(fileList_extracted_coins[j].absoluteFilePath());
    }

    // Extract each coins in an output floder
    for( unsigned int i = 0; i < vector_coins.size(); i++ )
    {
        cv::Mat extracted_coin;
        extracted_coin = initial_image_for_detection(cv::Rect(vector_coins[i].center.x-vector_coins[i].radius,vector_coins[i].center.y-vector_coins[i].radius,vector_coins[i].radius*2,vector_coins[i].radius*2));
        std::string name_extracted_coin = "output/coin" + std::to_string(i) + ".jpg";
        cv::imwrite( name_extracted_coin, extracted_coin);
    }
}

/** ********************************************************************************* **/
/** ************************** Debug to draw circles  ******************************* **/
/** ********************************************************************************* **/

void circleDetection::draw_circles()
{
    cv::Mat im;
    im=initial_image_for_detection.clone();
    for( size_t i = 0; i < vector_coins.size(); i++ )
    {
        // Draw the circles
        // circle center
        circle( im, vector_coins[i].center, 3, cv::Scalar(0,255,0), -1, 8, 0 );
        // circle outline
        circle( im, vector_coins[i].center, vector_coins[i].radius, cv::Scalar(0,0,255), 3, 8, 0 );
    }
    imshow( "Circle detection", im );
    cv::waitKey(0);
}

void circleDetection::detection(bool backGroundSeg, bool blur, char *methode, bool draw, QDir Dir_extracted_coins)
{
    if(backGroundSeg)
        backgroundSegmantation(true);
    preTreatment(blur);
    std::string choix(methode);
    if(choix == "methode1")
    {
        cv::vector<cv::Vec3f> circles;
        circles = technique1();
        post_treatment_technique1(circles);
    }
    else if(choix == "methode2")
    {
        std::vector<cv::RotatedRect> ellipses;
        ellipses = technique2();
        post_treatment_technique2(ellipses);
    }
    if(draw)
        draw_circles();
    extraction(Dir_extracted_coins);
}
