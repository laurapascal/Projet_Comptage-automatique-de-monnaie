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
    circleDetection(QString path_initial_image_for_detection, int method_param, bool debug_param);

    std::vector<coin> vector_coins;         // Vector containing the coins detected in the image

    /** detection of circles **/
    void detection(bool backGroundSeg, QDir Dir_extracted_coins);

private:
    int method;
    bool debug;

    cv::Mat initial_image_for_detection;    // Image in which the coins will be detected
    cv::Mat curent_image_for_detection;     // Modified image

    /** Pre-treatment **/
    void preTreatment();
    void addBlur();
    void thresholding();
    void gray_conversion();
    void backgroundSegmantation();

    /** Detection with method 1 or method 2 **/
    cv::vector<cv::Vec3f> circles;
    std::vector<cv::RotatedRect> ellipses;
    void HoughDetection();
    void ContourDetection();

    /** Post-treatment **/
    void post_treatment();
    void circle_store();
    void ellipse_store();
    void deleting_circles_outside_the_image();
    void circle_deletion();
    void ellipses_deletion();

    /** Coin Extraction **/
    void extraction(QDir Dir_extracted_coins);

    /** Debug to draw circle **/
    void draw_circles();

};

#endif // CIRCLEDETECTION_HPP

