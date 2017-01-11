#include "circleDetection.hpp"

circleDetection::circleDetection(QString path_initial_image_for_detection, int method_param, bool debug_param)
    :method(method_param), debug(debug_param)
{
    initial_image_for_detection = cv::imread( path_initial_image_for_detection.toStdString(), 1 );
    assert(initial_image_for_detection.data);
    curent_image_for_detection = initial_image_for_detection.clone();
}

/** ********************************************************************************* **/
/** ****************************** Pre-treatment  *********************************** **/
/** ********************************************************************************* **/

void circleDetection::preTreatment()
{
    if(method == 1)
    {
        addBlur();
        gray_conversion();
    }
    if(method == 2)
    {
        thresholding();
        gray_conversion();
    }

}

void circleDetection::addBlur()
{
    // Reduce the noise so we avoid false circle detection
    cv::GaussianBlur( curent_image_for_detection, curent_image_for_detection, cv::Size(9, 9), 3.5, 3.5 );

    if(debug)
    {
        imshow( "Add a blur to the image", curent_image_for_detection );
        cv::waitKey(0);
    }
}

void circleDetection::thresholding()
{
    int sum = 0;
    for(int r = 0; r < curent_image_for_detection.rows; r++)
    {
        for(int c = 0; c < curent_image_for_detection.cols; c++)
        {
            sum += (unsigned int)curent_image_for_detection.at<uchar>(r,c);
        }
    }
    int thresh = 0.99*(sum / (curent_image_for_detection.rows * curent_image_for_detection.cols));

    // Detect edges using Threshold
    cv::threshold( curent_image_for_detection, curent_image_for_detection, thresh, 255, cv::THRESH_BINARY );

    //Display of Thresholding for debug
    if(debug)
    {
        imshow( "Thresholding before applying the 2nd circles detection method", curent_image_for_detection );
        cv::waitKey(0);
    }
}

void circleDetection::gray_conversion()
{
    // Convert it to gray
    cvtColor( curent_image_for_detection, curent_image_for_detection, CV_BGR2GRAY );
    if(debug)
    {
        imshow( "Convert the image in gray", curent_image_for_detection );
        cv::waitKey(0);
    }
}

/** ********************************************************************************* **/
/** **************************** Detection of circle ******************************** **/
/** ********************************************************************************* **/

void circleDetection::HoughDetection()
{

    /// Apply the Hough Transform to find the circles
    // im_gray: Input image (grayscale)
    // circles: A vector that stores sets of 3 values: x_{c}, y_{c}, r for each detected circle.
    // CV_HOUGH_GRADIENT: Define the detection method. Currently this is the only one available in OpenCV
    // dp = 1: The inverse ratio of resolution
    // min_dist = im_gray.rows/8: Minimum distance between detected centers
    // param_1 = 80: Upper threshold for the internal Canny edge detector
    // param_2 = 60*: Threshold for center detection.
    // min_radius = 0: Minimum radio to be detected. If unknown, put zero as default.
    // max_radius = 0: Maximum radius to be detected. If unknown, put zero as default.

    cv::HoughCircles( curent_image_for_detection, circles, CV_HOUGH_GRADIENT, 1, curent_image_for_detection.rows/20, 80, 60, 0, 0);
}

void circleDetection::ContourDetection()
{
    // Find contours
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;

    cv::findContours( curent_image_for_detection, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

    // Find the rotated ellipses for each contour
    for( unsigned int i = 0; i < contours.size(); i++ )
    {
        if( contours[i].size() > 20 )
            ellipses.push_back(fitEllipse( cv::Mat(contours[i]) ));
    }
}

/** ********************************************************************************* **/
/** ***************************** Post-treatment  *********************************** **/
/** ********************************************************************************* **/
void circleDetection::post_treatment()
{
    if(method == 1)
    {
        circle_deletion();
    }
    else
    {
        ellipses_deletion();
    }
}

void circleDetection::circle_deletion()
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

void circleDetection::ellipses_deletion()
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
                if(std::sqrt((ellipse.center.x-ellipse_temp.center.x)*(ellipse.center.x-ellipse_temp.center.x) + (ellipse.center.y-ellipse_temp.center.y)*(ellipse.center.y-ellipse_temp.center.y)) < (radius_ellipse + radius_ellipse_temp))
                {
                    if(radius_ellipse > radius_ellipse_temp)
                    {
                        ellipses.erase(ellipses.begin() + j);
                        j--;
                    }

                    else if(radius_ellipse < radius_ellipse_temp)
                    {
                        ellipses.erase(ellipses.begin() + i);
                        i--;
                        break;
                    }

                }
            }
        }
    }
    vector_coins.clear();

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
/** ************************* Extraction des pieces  ******************************** **/
/** ********************************************************************************* **/

void circleDetection::clear_output(QDir Dir_extracted_coins)
{
    // Clearing the output floder
    QFileInfoList fileList_extracted_coins;
    fileList_extracted_coins.append(Dir_extracted_coins.entryInfoList());
    for( int j = 2; j < fileList_extracted_coins.size(); j++)
    {
        Dir_extracted_coins.remove(fileList_extracted_coins[j].absoluteFilePath());
    }
}

void circleDetection::extract_one_coin(cv::Mat coin_image, unsigned int coin_number)
{
    // Extract one coins in an output floder
    cv::Mat extracted_coin;
    extracted_coin = coin_image(cv::Rect(vector_coins[coin_number].center.x-vector_coins[coin_number].radius,
                                         vector_coins[coin_number].center.y-vector_coins[coin_number].radius,
                                         vector_coins[coin_number].radius*2,vector_coins[coin_number].radius*2));
    std::string name_extracted_coin = "output/coin" + std::to_string(coin_number) + ".jpg";
    cv::imwrite( name_extracted_coin, extracted_coin);
}

void circleDetection::extraction_square(QDir Dir_extracted_coins)
{
    clear_output(Dir_extracted_coins);

    // Extract each coins in an output floder
    for( unsigned int i = 0; i < vector_coins.size(); i++ )
    {
        extract_one_coin(initial_image_for_detection,i);
    }
}

void circleDetection::extraction_circle(QDir Dir_extracted_coins)
{
    clear_output(Dir_extracted_coins);

    // Extract each coins in an output floder
    for( unsigned int i = 0; i < vector_coins.size(); i++ )
    {
        // Segmentation of the coin thanks to a mask
        cv::Mat binary_mask(initial_image_for_detection.size(), CV_8UC3, cv::Scalar(0,0,0));
        cv::circle(binary_mask, vector_coins[i].center, vector_coins[i].radius, cv::Scalar(255,255,255), CV_FILLED);
        cv::Mat unique_coin_image(initial_image_for_detection.size(), CV_8UC3, cv::Scalar(0,0,0));
        initial_image_for_detection.copyTo(unique_coin_image, binary_mask);

        extract_one_coin(unique_coin_image,i);
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

/** ********************************************************************************* **/
/** **************************** General Functions  ********************************* **/
/** ********************************************************************************* **/

void circleDetection::detection()
{
    preTreatment();
    if(method == 1)
        HoughDetection();
    else if(method == 2)
        ContourDetection();
    post_treatment();

    if(debug)
        draw_circles();
}

void circleDetection::extraction(QDir Dir_extracted_coins, int score_method)
{
    if(score_method == 3)
    {
        extraction_circle(Dir_extracted_coins);
    }
    else
    {
        extraction_square(Dir_extracted_coins);
    }
}
