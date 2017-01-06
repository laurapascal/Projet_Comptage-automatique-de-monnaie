#include "registration.h"

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

    int minHessian = 100000;
    cv::SurfFeatureDetector detector(minHessian);

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
//    affichage_good_matches();
}

/** ********************************************************************************* **/
/** ********************** Debug: Affichage des good matches  *********************** **/
/** ********************************************************************************* **/
void registration::affichage_good_matches()
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


