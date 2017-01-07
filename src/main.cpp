#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <list>
#include <iterator>
#include <iostream>
#include <string>
#include <QFileDialog>
#include <QFileInfo>
#include <QString>
#include "circleDetection.hpp"

int comparaison(QString repertory_database, QString repertory_extracted_coins);

int main(int argc, char** argv)
{
    /** ********************************** DETECTION  ********************************** **/

    QDir Dir_extracted_coins("output");
    circleDetection detection(argv[1]);
    detection.detection(true, false, argv[2],true, Dir_extracted_coins);

    /** ********************************** COMPARAISON ********************************** **/

    comparaison("database","output");

    /** ********************************************************************************* **/

    cv::waitKey(0);
    return 0;
}

int comparaison(QString repertory_database, QString repertory_extracted_coins)
{

    QDir Dir_extracted_coins(repertory_extracted_coins);
    QFileInfoList fileList_extracted_coins;
    fileList_extracted_coins.append(Dir_extracted_coins.entryInfoList());
    for( int j = 2; j < fileList_extracted_coins.size(); j++)
    {
        cv::Mat img_extracted_coin = cv::imread( fileList_extracted_coins[j].absoluteFilePath().toStdString(), CV_LOAD_IMAGE_GRAYSCALE );

        /// Comparison with our data
        QDir Dir_database(repertory_database);
        QFileInfoList fileList_database;
        fileList_database.append(Dir_database.entryInfoList());
        for( int i = 2; i < fileList_database.size(); i++)
        {
            cv::Mat img_database = cv::imread( fileList_database[i].absoluteFilePath().toStdString(), CV_LOAD_IMAGE_GRAYSCALE );

        }
    }

    cv::waitKey(0);
    return 0;
}
