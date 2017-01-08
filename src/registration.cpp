#include "registration.hpp"

registration::registration(char* method_keypoint_descriptor_param, char* method_matches_param)
:method_keypoint_descriptor(method_keypoint_descriptor_param),method_matches(method_matches_param)
{

}

/** ********************************************************************************* **/
/** ********************* Creation of an image form the given path ****************** **/
/** ********************************************************************************* **/

void registration::creation_image_extracted_coin(QString path)
{
    img_ectracted_coin = get_image(path);
}

void registration::creation_image_data(QString path)
{
    img_data = get_image(path);
}

cv::Mat registration::get_image(QString path)
{
    return cv::imread( path.toStdString(), CV_LOAD_IMAGE_GRAYSCALE );
}

/** ********************************************************************************* **/
/** ***************************** Pre Treatment of images *************************** **/
/** ********************************************************************************* **/

void registration::preTreatment_images()
{
    cv::Mat mean_ectracted_coin;
    cv::Mat stddev_ectracted_coin;
    cv::meanStdDev(img_ectracted_coin, mean_ectracted_coin, stddev_ectracted_coin);
    std::cout<<mean_ectracted_coin.rows <<std::endl;

    for(int r = 0; r < mean_ectracted_coin.rows; r++ )
    {
        for(int l = 0; l < mean_ectracted_coin.cols; l++ )
        {
            std::cout<<"before: "<<(unsigned int)mean_ectracted_coin.at<uchar>(r,l)<<std::endl;

            mean_ectracted_coin.at<uchar>(r,l) = mean_ectracted_coin.at<uchar>(r,l) + 125;
            std::cout<<"after: "<<(unsigned int)mean_ectracted_coin.at<uchar>(r,l)<<std::endl;
        }
    }
    for(int r = 0; r < 10; r++ )
    {
        for(int l = 0; l < 10; l++ )
        {
            std::cout<<"before: "<<(unsigned int)img_ectracted_coin.at<uchar>(r,l)<<std::endl;
        }
    }
    img_ectracted_coin = (img_ectracted_coin - mean_ectracted_coin) ;

    for(int r = 0; r < 10; r++ )
    {
        for(int l = 0; l < 10; l++ )
        {
            std::cout<<"after: "<<(unsigned int)img_ectracted_coin.at<uchar>(r,l)<<std::endl;
        }
    }

    cv::Mat mean_data;
    mean_data.create(img_data.rows, img_data.cols, img_data.type());
    cv::Mat stddev_data;
    stddev_data.create(img_data.rows, img_data.cols, img_data.type());
    cv::meanStdDev(img_data, mean_data, stddev_data);
    img_data = (img_data - mean_data) ;

    imshow( "Img extrated coin normalized", img_ectracted_coin );
    cv::waitKey(0);

    imshow( "Img data normalized", img_data );
    cv::waitKey(0);


}

/** ********************************************************************************* **/
/** **************************** Creation of the keypoints ************************** **/
/** ********************************************************************************* **/
void registration::creation_keypoints_extracted_coin()
{
    std::vector<cv::KeyPoint> keypoints;
    if( method_keypoint_descriptor == "sift" )
        keypoints = get_keypoints_Sift(img_ectracted_coin);
    else if( method_keypoint_descriptor == "surf" )
        keypoints = get_keypoints_Surf(img_ectracted_coin);

    keypoints_extracted_coin = keypoints;
}

void registration::creation_keypoints_data()
{
    std::vector<cv::KeyPoint> keypoints;
    if( method_keypoint_descriptor == "sift" )
       keypoints =  get_keypoints_Sift(img_data);
    else if( method_keypoint_descriptor == "surf" )
        keypoints = get_keypoints_Surf(img_data);

    keypoints_data = keypoints;
}

std::vector<cv::KeyPoint> registration::get_keypoints_Sift(cv::Mat img)
{
    std::vector<cv::KeyPoint> keypoints;

    //-- Detect the keypoints using SIFT Detector
    int nfeatures = 4000;
    int nOctaveLayers = 3;
    double contrastThreshold = 0.04;
    double edgeThreshold = 10;
    double sigma = 1.0;
    cv::SiftFeatureDetector detector(nfeatures, nOctaveLayers, contrastThreshold, edgeThreshold, sigma);

    detector.detect(img, keypoints);

    return keypoints;
}

std::vector<cv::KeyPoint> registration::get_keypoints_Surf(cv::Mat img)
{
    std::vector<cv::KeyPoint> keypoints;

    int minHessian = 300;
    int nOctaves = 4;
    int nOctaveLayers = 2;
    bool extended = true;
    bool upright = false;
    cv::SurfFeatureDetector detector(minHessian, nOctaves, nOctaveLayers, extended, upright);

    detector.detect(img, keypoints);

    return keypoints;
}

/** ********************************************************************************* **/
/** *************************** Creation of the descriptors ************************* **/
/** ********************************************************************************* **/

void registration::creation_descriptors_extracted_coin()
{
    cv::Mat descriptors;
    if( method_keypoint_descriptor == "sift" )
        descriptors = get_descriptors_Sift(img_ectracted_coin, keypoints_extracted_coin);
    else if( method_keypoint_descriptor == "surf" )
        descriptors = get_descriptors_Surf(img_ectracted_coin, keypoints_extracted_coin);

     descriptors_extracted_coin = descriptors;
}

void registration::creation_descriptors_data()
{
    cv::Mat descriptors;
    if( method_keypoint_descriptor == "sift" )
       descriptors =  get_descriptors_Sift(img_data, keypoints_data);
    else if( method_keypoint_descriptor == "surf" )
        descriptors = get_descriptors_Surf(img_data, keypoints_data);

    descriptors_data = descriptors;
}

cv::Mat registration::get_descriptors_Sift(cv::Mat img, std::vector<cv::KeyPoint> keypoints)
{
    cv::Mat descriptors;

    // Calculate descriptors (feature vectors)
    cv::SiftDescriptorExtractor extractor;

    extractor.compute( img, keypoints, descriptors);

    return descriptors;
}

cv::Mat registration::get_descriptors_Surf(cv::Mat img, std::vector<cv::KeyPoint> keypoints)
{
    cv::Mat descriptors;

    // Calculate descriptors (feature vectors)
    cv::SurfDescriptorExtractor extractor;

    extractor.compute( img, keypoints, descriptors);

    return descriptors;
}

/** ********************************************************************************* **/
/** ******************************* Compute matches  ******************************** **/
/** ********************************************************************************* **/

void registration::compute_matches()
{
    if( method_matches == "flann")
        matches = get_matches_FLANN();
    else
        matches = get_matches_BF();
}


std::vector<cv::DMatch> registration::get_matches_FLANN()
{
    // Matching descriptor vectors using FLANN matcher
    cv::FlannBasedMatcher matcher;

    std::vector< cv::DMatch > matches;
    matcher.match( descriptors_extracted_coin, descriptors_data, matches );
    return matches;
}

std::vector<cv::DMatch> registration::get_matches_BF()
{
    // Matching descriptor vectors using Brut Force matcher
    cv::BFMatcher matcher(cv::NORM_L2);

    std::vector< cv::DMatch > matches;
    matcher.match( descriptors_extracted_coin, descriptors_data, matches );

    return matches;
}

/** ********************************************************************************* **/
/** **************************** Compute good matches  ****************************** **/
/** ********************************************************************************* **/

void registration::compute_good_matches()
{
    good_matches.clear();

    //-- Quick calculation of max and min distances between keypoints
    double max_dist= 0;
    double min_dist = 100;
    for( int i = 0; i < descriptors_extracted_coin.rows; i++ )
    { double dist = matches[i].distance;
        if( dist < min_dist ) min_dist = dist;
        if( dist > max_dist ) max_dist = dist;
    }

    for( int i = 0; i < descriptors_extracted_coin.rows; i++ )
    {
        if( matches[i].distance < 3*min_dist )
        {
            good_matches.push_back( matches[i]);
        }
    }
}

/** ********************************************************************************* **/
/** ************************* Compute hypothetical matches  ************************* **/
/** ********************************************************************************* **/
void registration::compute_hypothetical_matches()
{
    compute_matches();
    compute_good_matches();
    display_good_matches();
}

/** ********************************************************************************* **/
/** ********************** Debug: Display of good matches  *********************** **/
/** ********************************************************************************* **/
void registration::display_good_matches()
{
    std::cout<<"keypoints extracted coin: "<<keypoints_extracted_coin.size()<<std::endl;
    std::cout<<"descriptors extracted coin: "<<descriptors_extracted_coin.size()<<std::endl;
    std::cout<<"keypoints data: "<<keypoints_data.size()<<std::endl;
    std::cout<<"descriptors data: "<<descriptors_data.size()<<std::endl;
    std::cout<<"matches: "<<matches.size()<<std::endl;
    std::cout<<"good matches: "<<good_matches.size()<<std::endl;

    cv::Mat img_matches;
    cv::drawMatches( img_ectracted_coin, keypoints_extracted_coin, img_data, keypoints_data,
                     good_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
                     std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

    //-- Show detected matches
    imshow( "Good Matches & Object detection", img_matches );
    cv::waitKey(0);
}

/** ********************************************************************************* **/
/** ************ Compute the better transformation thanks to RanSaC  **************** **/
/** ********************************************************************************* **/
std::vector<cv::Mat> registration::findTransformation()
{
    // Get the keypoints from the good matches

    std::vector<cv::Point2f> good_keypoints_extracted_coin;
    std::vector<cv::Point2f> good_keypoints_data;

    for( unsigned int i = 0; i < good_matches.size(); i++ )
    {
        good_keypoints_extracted_coin.push_back( keypoints_extracted_coin[ good_matches[i].queryIdx ].pt );
        good_keypoints_data.push_back( keypoints_data[ good_matches[i].trainIdx ].pt );
    }

    // Compute the better transformation
    double ransacReprojThreshold = 3.0;
    H = cv::findHomography( good_keypoints_extracted_coin, good_keypoints_data, CV_RANSAC, ransacReprojThreshold, mask);

    // Debug
    display_inliers();

    std::vector<cv::Mat> result;

    result.push_back(H);
    result.push_back(mask);
    return result;
}

/** ********************************************************************************* **/
/** *************************** Debug: Display of inliers  ************************** **/
/** ********************************************************************************* **/
void registration::display_inliers()
{
    // Display of inliers
    std::vector< cv::DMatch > inliers;
    for(int r = 0; r < mask.rows; r++)
    {
        if((unsigned int)mask.at<uchar>(r,0) == 1)
        {
            inliers.push_back(good_matches[r]);
        }
    }

    cv::Mat img_inliers;
    cv::drawMatches( img_ectracted_coin, keypoints_extracted_coin, img_data, keypoints_data,
                     inliers, img_inliers, cv::Scalar::all(-1), cv::Scalar::all(-1),
                     std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

    //-- Show detected matches
    imshow( "Inliers", img_inliers );
    cv::waitKey(0);

    // Apply Homography found
    cv::Mat img_H;
    std::cout<<"homography: ["<<H.rows<<","<<H.cols<<"]"<<std::endl;
    cv::warpPerspective(img_ectracted_coin, img_H, H, img_data.size());

    imshow( "Apply H on the image", img_H );
    cv::waitKey(0);


}
