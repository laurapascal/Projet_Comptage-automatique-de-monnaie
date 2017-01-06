#include "data.h"

data::data(std::string value_param, QString data_path_param)
    :value(value_param),data_path(data_path_param)
{

}

cv::Mat data::get_image()
{
    cv::Mat img = cv::imread( data_path.toStdString(), CV_LOAD_IMAGE_GRAYSCALE );
    return img;
}
