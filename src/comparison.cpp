#include "comparison.hpp"

comparison::comparison(QString img_ectracted_coin_path, QString img_data_path, cv::Mat homographie_param, cv::Mat mask_param, int method_param, bool debug_param)
    :homographie(homographie_param), mask(mask_param), method(method_param), debug(debug_param)
{
    img_ectracted_coin = cv::imread( img_ectracted_coin_path.toStdString(), 1 );
    assert(img_ectracted_coin.data);
    img_data = cv::imread( img_data_path.toStdString(), 1 );
    assert(img_data.data);
}

float comparison::compute_score(std::vector<cv::KeyPoint> keypoints, std::vector< cv::DMatch > matches)
{
    if(method == 1)
        return get_inlierScore();
    else if(method == 2)
        return get_inlierScore()*get_inlier_repartition(keypoints, matches);
    else // 3
        return get_templateMatching_score();
}

/** ********************************************************************************* **/
/** ************ Compute the score with the number of inliers found  **************** **/
/** ********************************************************************************* **/
float comparison::get_inlierScore()
{
    float score = 0;

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
    return score;
}

/** ********************************************************************************* **/
/** *************************** Cheking inlier repatition  ************************** **/
/** ********************************************************************************* **/
float comparison::get_inlier_repartition(std::vector<cv::KeyPoint> keypoints, std::vector< cv::DMatch > matches)
{
    cv::Mat repartitionImage(img_ectracted_coin.rows, img_ectracted_coin.cols, CV_8UC1,  cv::Scalar(0));
    for(int r = 0; r < mask.rows; r++)
    {
        if((unsigned int)mask.at<uchar>(r,0) == 1)
        {
            cv::Point point= keypoints[ matches[r].queryIdx ].pt;
            repartitionImage.at<uchar>(point.y,point.x) = 255;
        }
    }
    cv::dilate(repartitionImage, repartitionImage, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(30, 30)));

    if(debug)
    {
        imshow( "Inlier repartition", repartitionImage );
        cv::waitKey(0);
    }

    float weighting = 0;

    for(int r = 0; r < img_ectracted_coin.rows; r++ )
    {
        for(int l = 0; l < img_ectracted_coin.cols; l++ )
        {
            if( repartitionImage.at<uchar>(r,l) == 255)
            {
                weighting += 1;
            }
        }
    }

    weighting /= (img_ectracted_coin.rows*img_ectracted_coin.cols);
    return weighting;
}


/** ********************************************************************************* **/
/** ***************** Compute score thanks to template matching ********************* **/
/** ********************************************************************************* **/
float comparison::get_templateMatching_score()
{
    /// Source image to display
    cv::Mat img_display;
    img_data.copyTo( img_display );

    int match_method = CV_TM_CCOEFF_NORMED; /** CV_TM_SQDIFF / CV_TM_SQDIFF_NORMED /
                                                 CV_TM_CCORR / CV_TM_CCORR_NORMED /
                                                 CV_TM_CCOEFF / CV_TM_CCOEFF_NORMED **/

    /// Create the result matrix
    int result_cols =  img_data.cols - img_ectracted_coin.cols + 1;
    int result_rows = img_data.rows - img_ectracted_coin.rows + 1;

    cv::Mat result;
    result.create( result_rows, result_cols, CV_32FC1 );

    /// Do the Matching and Normalize
    cv::matchTemplate( img_data, img_ectracted_coin, result, match_method );
    cv::normalize( result, result, 0, 1, cv::NORM_MINMAX, -1, cv::Mat() );

    /// Localizing the best match with minMaxLoc
    double minVal; double maxVal; cv::Point minLoc; cv::Point maxLoc;
    cv::Point matchLoc;

    cv::minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat() );

    /// For SQDIFF and SQDIFF_NORMED, the best matches are lower values. For all the other methods, the higher the better
    if( match_method  == CV_TM_SQDIFF || match_method == CV_TM_SQDIFF_NORMED )
    { matchLoc = minLoc; }
    else
    { matchLoc = maxLoc; }

    /// display for debug
    if(debug)
    {
        cv::rectangle( img_display, matchLoc, cv::Point( matchLoc.x + img_ectracted_coin.cols , matchLoc.y + img_ectracted_coin.rows ), cv::Scalar::all(0), 2, 8, 0 );
        cv::rectangle( result, matchLoc, cv::Point( matchLoc.x + img_ectracted_coin.cols , matchLoc.y + img_ectracted_coin.rows ), cv::Scalar::all(0), 2, 8, 0 );

        cv::imshow( "image détectée", img_display );
        cv::imshow( "resultat du template matching", result );
        cv::waitKey(0);
    }

    return mask.at<uchar>(matchLoc.x,matchLoc.y);
}
