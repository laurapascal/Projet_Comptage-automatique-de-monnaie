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

struct circle{
    cv::Point center;
    int radius;
};

class circleDetection
{
public:
    circleDetection(QString path_initial_image_for_detection, int method_param, bool debug_param);

    /** Number of detected coin **/
    int nb_detected_coin;

    /** detection of circles **/
    void detection();

    /** extraction of the found circles **/
    void extraction(QDir Dir_extracted_coins, int score_method, int size);

private:
    int method;
    bool debug;

    cv::Mat initial_image_for_detection;    // Image in which the coins will be detected
    cv::Mat curent_image_for_detection;     // Modified image

    /** Pre-treatment **/
    void preTreatment();
    void addBlur();
    void thresholding();
    void opening();
    void gray_conversion();

    /** Detection with method 1 or method 2 **/
    cv::vector<cv::Vec3f> hough_circles;
    std::vector<circle> circles;
    void HoughDetection();
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::RotatedRect> ellipses;
    void ContourDetection();

    /** Post-treatment **/
    void post_treatment();
    void circle_store();
    void ellipse_store();
    void ellipse_to_circle_store();
    void deleting_circles_outside_the_image();
    void deleting_ellipses_outside_the_image();
    void circle_deletion();
    void ellipses_deletion();

    /** Coin Extraction **/
    void clear_output(QDir Dir_extracted_coins);
    void extract_one_coin(cv::Mat coin_image, unsigned int coin_number, int size);
    void extraction_square(int size);
    void extraction_circle(int size);

    /** Debug to draw circle **/
    void draw_circles();

    /** Debug to draw ellipses **/
    void draw_ellipses();

    /** Debug to draw ellipses **/
    void draw_contours();
};

#endif // CIRCLEDETECTION_HPP

