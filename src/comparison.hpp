#ifndef COMPARISON_HPP
#define COMPARISON_HPP

#include <list>
#include <iterator>
#include <iostream>
#include <string>

#include <QFileDialog>
#include <QFileInfo>
#include <QFile>
#include <QString>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/nonfree/features2d.hpp"

class comparison
{
public:
    comparison(QString img_ectracted_coin_path, QString img_data_path, cv::Mat homographie_param, cv::Mat mask_param, int method_param, bool debug_param);

    float compute_score(std::vector<cv::KeyPoint> keypoints, std::vector< cv::DMatch > matches);

private:
    cv::Mat img_ectracted_coin;
    cv::Mat img_data;
    cv::Mat homographie;
    cv::Mat mask;
    int method;
    bool debug;

    /** Compute score equal to the number of inliers found **/
    float get_inlierScore();

    /** Compute a weighting thank to the repartition of the inliers in the image **/
    float get_inlier_repartition(std::vector<cv::KeyPoint> keypoints, std::vector< cv::DMatch > matches);

    /** Compute score thanks to template matching  **/
    float get_templateMatching_score();
};

#endif // COMPARISON_HPP

