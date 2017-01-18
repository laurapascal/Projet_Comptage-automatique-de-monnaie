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
        gray_conversion();
        thresholding();
        opening();
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
    int thresh = 0.85*(sum / (curent_image_for_detection.rows * curent_image_for_detection.cols));

    // Detect edges using Threshold
    cv::threshold( curent_image_for_detection, curent_image_for_detection, thresh, 255, cv::THRESH_BINARY );

    //Display of Thresholding for debug
    if(debug)
    {
        imshow( "Thresholding before applying the 2nd circles detection method", curent_image_for_detection );
        cv::waitKey(0);
    }
}

void circleDetection::opening()
{
    int morph_size = 5;
    cv::Mat element = cv::getStructuringElement( cv::MORPH_ELLIPSE, cv::Size( 2*morph_size + 1, 2*morph_size+1 ), cv::Point( morph_size, morph_size ) );
    cv::morphologyEx( curent_image_for_detection, curent_image_for_detection, cv::MORPH_OPEN,  element);

    //Display of opening for debug
    if(debug)
    {
        imshow( "Opening before applying the 2nd circles detection method", curent_image_for_detection );
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
    cv::HoughCircles( curent_image_for_detection, hough_circles, CV_HOUGH_GRADIENT, 1, curent_image_for_detection.rows/20, 80, 60, 0, 0);
}

void circleDetection::ContourDetection()
{
    // Find contours
    cv::findContours( curent_image_for_detection, contours, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

    if(debug)
        draw_contours();
}

/** ********************************************************************************* **/
/** ***************************** Post-treatment  *********************************** **/
/** ********************************************************************************* **/
void circleDetection::post_treatment()
{
    if(method == 1)
    {
        circle_store();
        deleting_circles_outside_the_image();
        circle_deletion();
        nb_detected_coin = circles.size();
        if(debug)
            draw_circles();
    }
    else // method = 2
    {
        ellipse_store();
        deleting_ellipses_outside_the_image();
        ellipses_deletion();       
        nb_detected_coin = ellipses.size();
        if(debug)
            draw_ellipses();
    }
}

void circleDetection::circle_store()
{
    for( size_t i = 0; i < hough_circles.size(); i++ )
    {
        cv::Point center(cvRound(hough_circles[i][0]), cvRound(hough_circles[i][1]));
        int radius = cvRound(hough_circles[i][2]);
        circle detected_circle;
        detected_circle.center = center;
        detected_circle.radius = radius;
        circles.push_back(detected_circle);
    }
}
void circleDetection::ellipse_store()
{
    // Find the rotated ellipses for each contour
    for( unsigned int i = 0; i < contours.size(); i++ )
    {
        if( contours[i].size() > (curent_image_for_detection.rows + curent_image_for_detection.cols)/40 )
        {
            ellipses.push_back(fitEllipse( cv::Mat(contours[i]) ));
        }
    }
}

// Deletion of circles outside the image
void circleDetection::deleting_circles_outside_the_image()
{
    for( unsigned int i = 0; i < circles.size(); i++ )
    {
        circle circle_detected = circles[i];

        if(circle_detected.radius > circle_detected.center.x || circle_detected.center.x > (initial_image_for_detection.cols - circle_detected.radius))
        {
            circles.erase(circles.begin() + i);
            i--;
        }
        else if(circle_detected.radius > circle_detected.center.y || circle_detected.center.y > (initial_image_for_detection.rows - circle_detected.radius))
        {
            circles.erase(circles.begin() + i);
            i--;
        }
    }
}

// Deletion of ellipses outside the image
void circleDetection::deleting_ellipses_outside_the_image()
{
    for( size_t i = 0; i < ellipses.size(); i++ )
    {
        cv::RotatedRect ellipse = ellipses[i];
        std::vector<cv::Point> ellipse_points;
        cv::Size2f size_ellipse(ellipse.size.width/2,ellipse.size.height/2);
        cv::ellipse2Poly(ellipse.center, size_ellipse, ellipse.angle, 0, 360, 1, ellipse_points);
        for(unsigned int k = 0; k < ellipse_points.size(); k++)
        {
            if(0 > ellipse_points[k].x || ellipse_points[k].x > initial_image_for_detection.cols || 0 > ellipse_points[k].y || ellipse_points[k].y > initial_image_for_detection.rows)
            {
                ellipses.erase(ellipses.begin() + i);
                i--;
                break;
            }
        }
    }
}

// Removing circles drawn on other circles
void circleDetection::circle_deletion()
{
    /// Treatement of the found circles
    for( size_t i = 0; i < circles.size(); i++ )
    {
        cv::Point center = circles[i].center;
        int radius = circles[i].radius;

        for ( unsigned int j = 0; j < i; j++)
        {
            cv::Point center_temp = circles[j].center;
            int radius_temp = circles[j].radius;

            if(std::sqrt((center.x-center_temp.x)*(center.x-center_temp.x) + (center.y-center_temp.y)*(center.y-center_temp.y)) < (radius+radius_temp))
            {
                circles.erase(circles.begin() + i);
                i--;
                break;
            }
        }
    }
}

// IF a ellipse is in an other ellipse THEN the ellipse is deleted
void circleDetection::ellipses_deletion()
{
    for( size_t i = 0; i < ellipses.size(); i++ )
    {
        cv::RotatedRect ellipse = ellipses[i];

        for( size_t j = 0; j < ellipses.size(); j++ )
        {
//            cv::Mat im_step = initial_image_for_detection.clone();

            cv::RotatedRect ellipse_temp = ellipses[j];
            std::vector<cv::Point> ellipse_points;
            cv::Size2f size_ellipse(ellipse_temp.size.width/2,ellipse_temp.size.height/2);
            cv::ellipse2Poly(ellipse_temp.center, size_ellipse, ellipse_temp.angle, 0, 360, 1, ellipse_points);

            bool is_included = true;
            if( ellipse_temp.center != ellipse.center &&   ellipse_temp.angle != ellipse.angle && ellipse_temp.size != ellipse.size)
            {
                for(unsigned int k = 0; k < ellipse_points.size(); k++)
                {
                    if((std::pow(((ellipse_points[k].x - ellipse.center.x )/ellipse.size.width),2) + std::pow(((ellipse_points[k].y - ellipse.center.y )/ellipse.size.height),2)) > 1)
                    {
                        is_included = false;
                    }
                }
                if(is_included)
                {
//                    std::cout<<"supression"<<std::endl;
                    ellipses.erase(ellipses.begin() + j);
                    j = 0;
                    i = 0;
                    break;
                }
            }
//            cv::ellipse( im_step, ellipse, cv::Scalar(255,255,0), 3, 8 );
//            cv::ellipse( im_step, ellipse_temp, cv::Scalar(0,255,255), 3, 8 );
//            cv::rectangle(im_step, ellipse.boundingRect(), cv::Scalar(255,0,255), 3, 8 );
//            cv::rectangle(im_step, ellipse_temp.boundingRect(), cv::Scalar(255,255,255), 3, 8 );
//            imshow( "Ellipses Deletion Steps", im_step );
//            cv::waitKey(0);
        }
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

void circleDetection::extract_one_coin(cv::Mat coin_image, unsigned int coin_number, int size, cv::Rect rect)
{
    // Extract one coins in an output floder
    cv::Mat extracted_coin;
    extracted_coin = coin_image(rect);
    std::string name_extracted_coin = "output/coin" + std::to_string(coin_number) + ".jpg";
    cv::Size newsize;
    newsize.height = size;
    newsize.width = size;
    cv::resize(extracted_coin,extracted_coin,newsize);
    cv::imwrite( name_extracted_coin, extracted_coin);
}

void circleDetection::extraction_circle_in_square(int size)
{
    // Extract each coins in an output floder
    for( unsigned int i = 0; i < circles.size(); i++ )
    {
        cv::Rect rect(circles[i].center.x-circles[i].radius,
                      circles[i].center.y-circles[i].radius,
                      circles[i].radius*2,circles[i].radius*2);

        extract_one_coin(initial_image_for_detection, i, size, rect);
    }
}

void circleDetection::extraction_ellipse_in_square(int size)
{
    // Extract each coins in an output floder
    for( unsigned int i = 0; i < ellipses.size(); i++ )
    {
        cv::Rect rect = ellipses[i].boundingRect();

        extract_one_coin(initial_image_for_detection, i, size, rect);
    }
}

void circleDetection::extraction_circle(int size)
{

    // Extract each coins in an output floder
    for( unsigned int i = 0; i < circles.size(); i++ )
    {
        // Segmentation of the coin thanks to a mask
        cv::Mat binary_mask(initial_image_for_detection.size(), CV_8UC3, cv::Scalar(0,0,0));
        cv::circle(binary_mask, circles[i].center, circles[i].radius, cv::Scalar(255,255,255), CV_FILLED);
        cv::Mat unique_coin_image(initial_image_for_detection.size(), CV_8UC3, cv::Scalar(0,0,0));
        initial_image_for_detection.copyTo(unique_coin_image, binary_mask);

        cv::Rect rect(circles[i].center.x-circles[i].radius,
                      circles[i].center.y-circles[i].radius,
                      circles[i].radius*2,circles[i].radius*2);

        extract_one_coin(unique_coin_image, i, size, rect);
    }
}

void circleDetection::extraction_ellipse(int size)
{

    // Extract each coins in an output floder
    for( unsigned int i = 0; i < ellipses.size(); i++ )
    {
        // Segmentation of the coin thanks to a mask
        cv::Mat binary_mask(initial_image_for_detection.size(), CV_8UC3, cv::Scalar(0,0,0));
        cv::ellipse(binary_mask, ellipses[i], cv::Scalar(255,255,255), CV_FILLED);
        cv::Mat unique_coin_image(initial_image_for_detection.size(), CV_8UC3, cv::Scalar(0,0,0));
        initial_image_for_detection.copyTo(unique_coin_image, binary_mask);

        cv::Rect rect = ellipses[i].boundingRect();

        extract_one_coin(unique_coin_image, i, size, rect);
    }
}

/** ********************************************************************************* **/
/** ***************** Debug to draw circles, ellipses and contours ****************** **/
/** ********************************************************************************* **/

void circleDetection::draw_circles()
{
    cv::Mat im;
    im = initial_image_for_detection.clone();

    for( size_t i = 0; i < circles.size(); i++ )
    {
        // Draw the circles
        // circle center
        cv::circle( im, circles[i].center, 3, cv::Scalar(0,255,0), -1, 8, 0 );
        // circle outline
        cv::circle( im, circles[i].center, circles[i].radius, cv::Scalar(0,0,255), 3, 8, 0 );
    }

    imshow( "Circle detection", im );
    cv::waitKey(0);
}

void circleDetection::draw_ellipses()
{
    cv::Mat im;
    im = initial_image_for_detection.clone();
    for( size_t i = 0; i< ellipses.size(); i++ )
    {
        cv::ellipse( im, ellipses[i], cv::Scalar(255,0,0), 3, 8 );
        cv::rectangle(im, ellipses[i].boundingRect(), cv::Scalar(255,0,255), 3, 8 );
    }
    imshow( "Ellipses detection", im );
    cv::waitKey(0);
}

void circleDetection::draw_contours()
{

    cv::Mat im;
    im = initial_image_for_detection.clone();
    for( size_t i = 0; i< contours.size(); i++ )
    {
        cv::drawContours( im, contours, (int)i, cv::Scalar(0,255,255), 3, 8);
    }
    imshow( "Contours detection", im );
    cv::waitKey(0);
}

/** ********************************************************************************* **/
/** **************************** General Functions  ********************************* **/
/** ********************************************************************************* **/

void circleDetection::detection()
{
    preTreatment();
    if(method == 1)
    {
        HoughDetection();
    }
    else // method = 2
    {
        ContourDetection();
    }
    post_treatment();
}

void circleDetection::extraction(QDir Dir_extracted_coins, int score_method, int size)
{
    clear_output(Dir_extracted_coins);

    if(score_method == 3)
    {
        if(method == 1)
            extraction_circle(size);
        else // 2
            extraction_ellipse(size);
    }
    else // score method = 1 or 2
    {
        if(method == 1)
            extraction_circle_in_square(size);
        else
            extraction_ellipse_in_square(size);
    }
}
