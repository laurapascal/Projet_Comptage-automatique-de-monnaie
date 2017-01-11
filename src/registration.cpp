#include "registration.hpp"

registration::registration(std::string method_keypoint_descriptor_param, std::string method_matches_param, bool debug_param)
    :method_keypoint_descriptor(method_keypoint_descriptor_param), method_matches(method_matches_param), debug(debug_param)
{

}

/** ********************************************************************************* **/
/** ********************* Creation of an image form the given path ****************** **/
/** ********************************************************************************* **/

void registration::creation_image_extracted_coin(QString path)
{
    img_extracted_coin = get_image(path);
}

void registration::creation_image_data(QString path)
{
    img_data = get_image(path);
}

cv::Mat registration::get_image(QString path)
{
    return cv::imread( path.toStdString() );
}

/** ********************************************************************************* **/
/** **************************** Creation of the keypoints ************************** **/
/** ********************************************************************************* **/
void registration::creation_keypoints_extracted_coin()
{
    std::vector<cv::KeyPoint> keypoints;
    if( method_keypoint_descriptor == "sift" )
        keypoints = get_keypoints_Sift(img_extracted_coin);
    else if( method_keypoint_descriptor == "surf" )
        keypoints = get_keypoints_Surf(img_extracted_coin);
    else if( method_keypoint_descriptor == "orb" )
        keypoints = get_keypoints_ORB(img_extracted_coin);

    keypoints_extracted_coin = keypoints;
}

void registration::creation_keypoints_data()
{
    std::vector<cv::KeyPoint> keypoints;
    if( method_keypoint_descriptor == "sift" )
        keypoints =  get_keypoints_Sift(img_data);
    else if( method_keypoint_descriptor == "surf" )
        keypoints = get_keypoints_Surf(img_data);
    else if( method_keypoint_descriptor == "orb" )
        keypoints = get_keypoints_ORB(img_data);

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

std::vector<cv::KeyPoint> registration::get_keypoints_ORB(cv::Mat img)
{
    std::vector<cv::KeyPoint> keypoints;

    int nfeatures = 4000;
    float scaleFactor = 1.2f;
    int nlevels = 8;
    int edgeThreshold = 31;
    int firstLevel = 0;
    int WTA_K = 4;
    int scoreType = cv::ORB::HARRIS_SCORE;
    int patchSize = 50;
    cv::OrbFeatureDetector detector(nfeatures, scaleFactor, nlevels, edgeThreshold, firstLevel, WTA_K, scoreType, patchSize);

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
        descriptors = get_descriptors_Sift(img_extracted_coin, keypoints_extracted_coin);
    else if( method_keypoint_descriptor == "surf" )
        descriptors = get_descriptors_Surf(img_extracted_coin, keypoints_extracted_coin);
    else if( method_keypoint_descriptor == "orb" )
        descriptors = get_descriptors_ORB(img_extracted_coin, keypoints_extracted_coin);

    descriptors_extracted_coin = descriptors;
}

void registration::creation_descriptors_data()
{
    cv::Mat descriptors;
    if( method_keypoint_descriptor == "sift" )
        descriptors =  get_descriptors_Sift(img_data, keypoints_data);
    else if( method_keypoint_descriptor == "surf" )
        descriptors = get_descriptors_Surf(img_data, keypoints_data);
    else if( method_keypoint_descriptor == "orb" )
        descriptors = get_descriptors_ORB(img_data, keypoints_data);

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

cv::Mat registration::get_descriptors_ORB(cv::Mat img, std::vector<cv::KeyPoint> keypoints)
{
    cv::Mat descriptors;

    // Calculate descriptors (feature vectors)
    cv::OrbDescriptorExtractor extractor;

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
    else if( method_matches == "BF" )
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
    int normType;
    if(method_keypoint_descriptor == "surf" || method_keypoint_descriptor == "sift")
        normType = cv::NORM_L2;
    else // orb
        normType = cv::NORM_HAMMING2;
    cv::BFMatcher matcher(normType, true);

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

    // Debug
    if(debug)
    {
        display_features();
        display_information();
        display_good_matches();
    }
}

/** ********************************************************************************* **/
/** ********************** Debug: Display of good matches  *********************** **/
/** ********************************************************************************* **/

void registration::display_features()
{
    cv::Mat img_KPO;
    cv::drawKeypoints(img_extracted_coin,keypoints_extracted_coin,img_KPO, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    cv::imshow( "KeyPoints on Extracted Coin", img_KPO );

    cv::Mat img_KPS;
    cv::drawKeypoints(img_data,keypoints_data,img_KPS, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    cv::imshow( "KeyPoints on DataBase", img_KPS );
}

void registration::display_information()
{
    std::cout<<"keypoints extracted coin: "<<keypoints_extracted_coin.size()<<std::endl;
    std::cout<<"descriptors extracted coin: "<<descriptors_extracted_coin.size()<<std::endl;
    std::cout<<"keypoints data: "<<keypoints_data.size()<<std::endl;
    std::cout<<"descriptors data: "<<descriptors_data.size()<<std::endl;
    std::cout<<"matches: "<<matches.size()<<std::endl;
    std::cout<<"good matches: "<<good_matches.size()<<std::endl;
}


void registration::display_good_matches()
{
    cv::Mat img_matches;
    cv::drawMatches( img_extracted_coin, keypoints_extracted_coin, img_data, keypoints_data,
                     good_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
                     std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

    //-- Show detected matches
    imshow( "Good Matches & Object detection", img_matches );
    cv::waitKey(0);
}

/** ********************************************************************************* **/
/** ************ Compute the better transformation thanks to RanSaC  **************** **/
/** ********************************************************************************* **/
std::vector<cv::Mat> registration::findTransformation_m1()
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
    double ransacReprojThreshold = 6.0;
    H = cv::findHomography( good_keypoints_extracted_coin, good_keypoints_data, CV_RANSAC, ransacReprojThreshold, mask);

    // Debug
    if(debug)
    {
        display_inliers();
        apply_homography();
    }

    std::vector<cv::Mat> result;

    result.push_back(H);
    result.push_back(mask);
    return result;
}

std::vector<cv::Mat> registration::findTransformation_m2()
{

    int nb_inliers = 0;

    // RANSAC
    for( int i = 0; i < 10000; i++)
    {
        // Get the keypoints from the good matches
        std::vector<cv::Point2f> good_keypoints_extracted_coin;
        std::vector<cv::Point2f> good_keypoints_data;
        for( unsigned int j = 0; j < good_matches.size(); j++ )
        {
            good_keypoints_extracted_coin.push_back( keypoints_extracted_coin[ good_matches[j].queryIdx ].pt );
            good_keypoints_data.push_back( keypoints_data[ good_matches[j].trainIdx ].pt );
        }

        // Choose the 3 pairs
        std::vector<cv::Point2f> random_keypoints_extracted_coin;
        std::vector<cv::Point2f> ramdom_keypoints_data;
        std::vector<int> k;
        for( unsigned int j = 0; j < 3; j++ )
        {
            k.push_back(rand() % good_keypoints_data.size());
            random_keypoints_extracted_coin.push_back( good_keypoints_extracted_coin[ k[j] ] );
            ramdom_keypoints_data.push_back( good_keypoints_data[ k[j] ]  );
        }

        // Compute the better transformation
        cv::Mat R = cv::estimateRigidTransform(random_keypoints_extracted_coin, ramdom_keypoints_data, false);

        // Apply the found transformation
        if(R.cols != 0 && R.rows != 0)
        {
            cv::Mat H_temp = cv::Mat(3,3,R.type());
            H_temp.at<double>(0,0) = R.at<double>(0,0);
            H_temp.at<double>(0,1) = R.at<double>(0,1);
            H_temp.at<double>(0,2) = R.at<double>(0,2);

            H_temp.at<double>(1,0) = R.at<double>(1,0);
            H_temp.at<double>(1,1) = R.at<double>(1,1);
            H_temp.at<double>(1,2) = R.at<double>(1,2);

            H_temp.at<double>(2,0) = 0.0;
            H_temp.at<double>(2,1) = 0.0;
            H_temp.at<double>(2,2) = 1.0;

            // Debug
//            cv::Mat img_H;
//            cv::warpPerspective(img_extracted_coin, img_H, H_temp, img_data.size());
//            imshow( "Apply H on the image", img_H );
//            cv::waitKey(0);

            // Apply Homography found
            cv::transform(good_keypoints_extracted_coin, good_keypoints_extracted_coin, R);

            // Count the number of inliers
            int nb_inliers_temp = 0 ;
            cv::Mat mask_temp;
            mask_temp.create(good_matches.size(), 1, mask.type());
            for( unsigned int j = 0; j < good_keypoints_data.size(); j++ )
            {
                // Inlier
                if( std::fabs(good_keypoints_extracted_coin[j].x - good_keypoints_data[j].x) < 11  && std::fabs(good_keypoints_extracted_coin[j].y - good_keypoints_data[j].y) < 11 )
                {
                    nb_inliers_temp += 1;
                    mask_temp.at<uchar>(j,0) = 1;
                }
                // Outlier
                else
                    mask_temp.at<uchar>(j,0) = 0;
            }
            if (nb_inliers < nb_inliers_temp)
            {
                nb_inliers =  nb_inliers_temp;
                H = H_temp.clone();
                mask = mask_temp.clone();
            }
        }
    }

    // Debug
    display_inliers();

    std::vector<cv::Mat> result;

    result.push_back(H);
    result.push_back(mask);
    return result;
}

/** ********************************************************************************* **/
/** ************** Debug: Display of inliers and apply homography  ****************** **/
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
    cv::drawMatches( img_extracted_coin, keypoints_extracted_coin, img_data, keypoints_data,
                     inliers, img_inliers, cv::Scalar::all(-1), cv::Scalar::all(-1),
                     std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

    //-- Show detected matches
    imshow( "Inliers", img_inliers );
    cv::waitKey(0);
}

void registration::apply_homography()
{
    // Apply Homography found
    std::cout<<"Transformation: "<<H<<std::endl;
    cv::Mat img_H;
    cv::warpPerspective(img_extracted_coin, img_H, H, img_data.size());

    imshow( "Apply H on the image", img_H );
    cv::waitKey(0);
}
