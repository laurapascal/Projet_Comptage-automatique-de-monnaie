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

