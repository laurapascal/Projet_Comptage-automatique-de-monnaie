#ifndef CIRCLEDETECTION_HPP
#define CIRCLEDETECTION_HPP

#include <vector>
#include <iostream>
#include <map>
#include <list>
#include <iterator>
#include <string>
#include <QFileDialog>
#include <QFileInfo>
#include <QDir>
#include <QDirIterator>
#include <QFile>
#include <QString>
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"

struct coin{
    cv::Point center;
    int radius;
};

class circleDetection
{
public:
    circleDetection(QString path_initial_image_for_detection);

    /** detection of circles **/
    void detection(bool backGroundSeg, bool blur, char *method, bool draw, QDir Dir_extracted_coins);

private:
    cv::Mat initial_image_for_detection;    // Image in which the coins will be detected
    cv::Mat curent_image_for_detection;     // Modified image
    std::vector<coin> vector_coins;         // Vector containing the coins detected in the image

    /** Pre-treatment **/
    void preTreatment(bool blur);
    void backgroundSegmantation(bool display);

    /** Detection with method 1 or method 2 **/
    cv::vector<cv::Vec3f> method1();
    std::vector<cv::RotatedRect> method2();

    /** Post-treatment **/
    void post_treatment_method1(cv::vector<cv::Vec3f> circles);
    void post_treatment_method2(std::vector<cv::RotatedRect> ellipses);

    /** Coin Extraction **/
    void extraction(QDir Dir_extracted_coins);

    /** Debug to draw circle **/
    void draw_circles();

};

#endif // CIRCLEDETECTION_HPP

