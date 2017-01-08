#include "comparison.hpp"

comparison::comparison(QString img_ectracted_coin_path, QString img_data_path, cv::Mat homographie_param, cv::Mat mask_param)
    :homographie(homographie_param), mask(mask_param)
{
    img_ectracted_coin = cv::imread( img_ectracted_coin_path.toStdString(), 1 );
    assert(img_ectracted_coin.data);
    img_data = cv::imread( img_data_path.toStdString(), 1 );
    assert(img_data.data);
}

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

    // Percentage
    score = 100*(score / (float)mask.rows);
    return score;
}

float comparison::get_templateMatching_score(bool debug)
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
