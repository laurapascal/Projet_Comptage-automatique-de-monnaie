#ifndef REGISTRATION_H
#define REGISTRATION_H

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

class registration
{
public:
    registration(char* method_keypoint_param, char* method_matches_param);

    std::string method_keypoint_descriptor;
    std::string method_matches;

    cv::Mat img_ectracted_coin;
    cv::Mat img_data;
    void creation_image_extracted_coin(QString path);
    void creation_image_data(QString path);

    std::vector<cv::KeyPoint> keypoints_extracted_coin;
    std::vector<cv::KeyPoint> keypoints_data;
    void creation_keypoints_extracted_coin();
    void creation_keypoints_data();

    cv::Mat descriptors_extracted_coin;
    cv::Mat descriptors_data;
    void creation_descriptors_extracted_coin();
    void creation_descriptors_data();

    std::vector<cv::DMatch> matches;
    std::vector< cv::DMatch > good_matches;
    void compute_hypothetical_matches();

    std::vector<cv::Mat> findTransformation();

private:
    cv::Mat get_image(QString path);

    std::vector<cv::KeyPoint> get_keypoints_Sift(cv::Mat img);
    std::vector<cv::KeyPoint> get_keypoints_Surf(cv::Mat img);

    cv::Mat get_descriptors_Sift(cv::Mat img, std::vector<cv::KeyPoint> keypoints);
    cv::Mat get_descriptors_Surf(cv::Mat img, std::vector<cv::KeyPoint> keypoints);

    std::vector<cv::DMatch> get_matches_FLANN();
    std::vector<cv::DMatch> get_matches_BF();

    void compute_matches();
    void compute_good_matches();

    void affichage_good_matches();
};

#endif // REGISTRATION_H
