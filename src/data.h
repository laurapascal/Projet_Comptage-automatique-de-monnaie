#ifndef DATA_H
#define DATA_H

#include <QString>
#include <opencv2/opencv.hpp>


class data
{
public:
    data(std::string value_param, QString data_path_param);
    std::string value;
    QString data_path;

    cv::Mat get_image();

};

#endif // DATA_H
