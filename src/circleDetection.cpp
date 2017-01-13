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

void circleDetection::backgroundSegmantation()
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

    if(debug)
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

    if(debug)
        draw_contours(contours);


    // Find the rotated ellipses for each contour
    for( unsigned int i = 0; i < contours.size(); i++ )
    {
        if( contours[i].size() > (curent_image_for_detection.rows + curent_image_for_detection.cols)/40 )
            ellipses.push_back(fitEllipse( cv::Mat(contours[i]) ));
    }

    if(debug)
        draw_ellipses();
}

/** ********************************************************************************* **/
/** ***************************** Post-treatment  *********************************** **/
/** ********************************************************************************* **/
void circleDetection::post_treatment()
{
    if(method == 1)
        circle_store();
    else // 2
        ellipse_store();
    deleting_circles_outside_the_image();
    if(method == 1)
        circle_deletion();
    else // 2
        ellipses_deletion();
}

void circleDetection::circle_store()
{
    for( size_t i = 0; i < circles.size(); i++ )
    {
        cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);
        coin coin_detected;
        coin_detected.center = center;
        coin_detected.radius = radius;
        vector_coins.push_back(coin_detected);
    }
}

void circleDetection::ellipse_store()
{
    // IF the ellipse is a circle THEN the ellipse is stored
    for( unsigned int i = 0; i < ellipses.size(); i++ )
    {
        cv::RotatedRect ellipse = ellipses[i];
        if(std::fabs(ellipse.size.width - ellipse.size.height) < (ellipse.size.width + ellipse.size.height)/20)
        {
            coin coin_detected;
            coin_detected.center = ellipse.center;
            coin_detected.radius = (ellipse.size.width + ellipse.size.height)/4;
            vector_coins.push_back(coin_detected);
        }

    }
}

void circleDetection::deleting_circles_outside_the_image()
{
    for( unsigned int i = 0; i < vector_coins.size(); i++ )
    {
        coin coin_detected = vector_coins[i];

        if(coin_detected.radius > coin_detected.center.x || coin_detected.center.x > (initial_image_for_detection.cols - coin_detected.radius))
        {
            vector_coins.erase(vector_coins.begin() + i);
            i--;
        }
        else if(coin_detected.radius > coin_detected.center.y || coin_detected.center.y > (initial_image_for_detection.rows - coin_detected.radius))
        {
            vector_coins.erase(vector_coins.begin() + i);
            i--;
        }
    }
}

// Removing circles drawn on other circles
void circleDetection::circle_deletion()
{
    /// Treatement of the found circles
    for( size_t i = 0; i < vector_coins.size(); i++ )
    {
        cv::Point center = vector_coins[i].center;
        int radius = vector_coins[i].radius;

        for ( unsigned int j = 0; j < i; j++)
        {
            cv::Point center_temp = vector_coins[j].center;
            int radius_temp = vector_coins[j].radius;

            if(std::sqrt((center.x-center_temp.x)*(center.x-center_temp.x) + (center.y-center_temp.y)*(center.y-center_temp.y)) < (radius+radius_temp))
            {
                vector_coins.erase(vector_coins.begin() + i);
                i--;
                break;
            }
        }
    }
}

// IF the circle is in an other circle THEN the circle is deleted
void circleDetection::ellipses_deletion()
{
    for( unsigned int i = 0; i < vector_coins.size(); i++ )
    {
        float radius = vector_coins[i].radius;
        cv::Point center = vector_coins[i].center;
        for( unsigned int j = 0; j < vector_coins.size(); j++ )
        {

            float radius_temp = vector_coins[j].radius;
            cv::Point center_temp = vector_coins[j].center;
            if(i!=j)
            {
                if(std::sqrt((center.x-center_temp.x)*(center.x-center_temp.x) + (center.y-center_temp.y)*(center.y-center_temp.y)) < (radius + radius_temp))
                {
                    if(radius > radius_temp)
                    {
                        vector_coins.erase(vector_coins.begin() + j);
                        j--;
                        break;
                    }

                    else
                    {
                        vector_coins.erase(vector_coins.begin() + i);
                        i--;
                        break;
                    }

                }
            }
        }
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
/** ***************** Debug to draw circles, ellipses and contours ****************** **/
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

void circleDetection::draw_ellipses()
{
    cv::Mat im;
    im=initial_image_for_detection.clone();
    for( size_t i = 0; i< ellipses.size(); i++ )
    {
        cv::ellipse( im, ellipses[i], cv::Scalar(0,0,255), 3, 8 );
    }
    imshow( "Ellipses detection", im );
    cv::waitKey(0);
}

void circleDetection::draw_contours(std::vector<std::vector<cv::Point> > contours)
{

    cv::Mat im;
    im=initial_image_for_detection.clone();
    for( size_t i = 0; i< contours.size(); i++ )
    {
        cv::drawContours( im, contours, (int)i, cv::Scalar(0,0,255), 3, 8);
    }
    imshow( "Contours detection", im );
    cv::waitKey(0);
}

/** ********************************************************************************* **/
/** ***************************** General Function  ********************************* **/
/** ********************************************************************************* **/

void circleDetection::detection(bool backGroundSeg, QDir Dir_extracted_coins)
{
    if(backGroundSeg)
        backgroundSegmantation();
    preTreatment();
    if(method == 1)
        HoughDetection();
    else if(method == 2)
        ContourDetection();
    if(method == 3)
    {
        method = 1;
        preTreatment();
        HoughDetection();
        post_treatment();
        curent_image_for_detection = initial_image_for_detection.clone();
        method = 2;
        preTreatment();
        ContourDetection();
        post_treatment();
        method = 3;
    }
    post_treatment();

    if(debug)
        draw_circles();

    extraction(Dir_extracted_coins);
}
