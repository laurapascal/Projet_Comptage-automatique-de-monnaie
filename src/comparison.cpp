#include "comparison.hpp"

comparison::comparison(QString img_extracted_coin_path, QString img_data_path, cv::Mat H_param, cv::Mat mask_param, int method_param,int size, bool debug_param)
    :H(H_param), mask(mask_param), method(method_param), debug(debug_param)
{
    img_extracted_coin = cv::imread( img_extracted_coin_path.toStdString(), 1 );
    assert(img_extracted_coin.data);
    img_data = cv::imread( img_data_path.toStdString(), 1 );
    assert(img_data.data);
    cv::Size newsize;
    newsize.height = size;
    newsize.width = size;
    cv::resize(img_data,img_data,newsize);
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
    cv::Mat repartitionImage(img_extracted_coin.rows, img_extracted_coin.cols, CV_8UC1,  cv::Scalar(0));
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

    for(int r = 0; r < img_extracted_coin.rows; r++ )
    {
        for(int l = 0; l < img_extracted_coin.cols; l++ )
        {
            if( repartitionImage.at<uchar>(r,l) == 255)
            {
                weighting += 1;
            }
        }
    }

    weighting /= (img_extracted_coin.rows*img_extracted_coin.cols);
    return weighting;
}


/** ********************************************************************************* **/
/** ***************** Compute score thanks to template matching ********************* **/
/** ********************************************************************************* **/
float comparison::get_templateMatching_score()
{
    cv::Mat img_H;
    cv::warpPerspective(img_extracted_coin, img_H, H, img_data.size());

    // Source image to display
    cv::Mat img_treated_data = erase_background(img_data);


    int match_method = CV_TM_CCOEFF_NORMED; /** CV_TM_SQDIFF / CV_TM_SQDIFF_NORMED /
                                                 CV_TM_CCORR / CV_TM_CCORR_NORMED /
                                                 CV_TM_CCOEFF / CV_TM_CCOEFF_NORMED **/

    // Create the result matrix (the size is 1*1 because the size of the two images is the same)
    cv::Mat result;
    result.create( 1, 1, CV_32FC1 );

    // Compare the to images pixels one by one and return a result
    cv::matchTemplate( img_treated_data, img_H, result, match_method );
    if( match_method  == CV_TM_SQDIFF || match_method == CV_TM_SQDIFF_NORMED )
        return 1000.0 - result.at<float>( 0, 0 );  //in those cases the best match is the lowest score
    else
        return result.at<float>( 0, 0 );  //in the other cases the best match is the highest score
}

// Deletion of the background thanks to grab-cut
cv::Mat comparison::erase_background(cv::Mat input_image)
{

    // define bounding rectangle
    int col = input_image.cols;
    col *= 0.01;
    cv::Rect rectangle(col,col,input_image.cols-col*2,input_image.rows-col*2);

    cv::Mat result_seg; // segmentation result (4 possible values)
    cv::Mat bgModel,fgModel; // the models (internally used)

    // GrabCut segmentation
    cv::grabCut(input_image,    // input image
                result_seg,   // segmentation result
                rectangle,// rectangle containing foreground
                bgModel,fgModel, // models
                2,        // number of iterations
                cv::GC_INIT_WITH_RECT); // use rectangle

    // Get the pixels marked as likely foreground
    cv::compare(result_seg,cv::GC_PR_FGD,result_seg,cv::CMP_EQ);
    // Generate output image
    cv::Mat foreground(input_image.size(),CV_8UC3,cv::Scalar(0,0,0));
    cv::Mat background(input_image.size(),CV_8UC3,cv::Scalar(0,0,0));
    input_image.copyTo(background,~result_seg);
    input_image.copyTo(foreground,result_seg);

    if(debug)
    {
        //Saving the result for debugging
        cv::Mat rect = input_image.clone();
        cv::rectangle(rect, rectangle, cv::Scalar(255,0,0),1);
        cv::imshow("initial rectangle",rect);
        cv::imshow("Foreground",foreground);
        cv::imshow("Background",background);
    }
    cv::Mat zero(input_image.size(),CV_8UC3,cv::Scalar(0,0,0));
    cv::Mat one(input_image.size(),CV_8UC3,cv::Scalar(255,255,255));
    one.copyTo(zero,result_seg);

    cv::Mat output_image;
    output_image = zero.clone();

    return output_image;
}
