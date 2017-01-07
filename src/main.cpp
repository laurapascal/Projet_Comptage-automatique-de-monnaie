#include <list>
#include <iterator>
#include <iostream>
#include <string>
#include <QFileDialog>
#include <QFileInfo>
#include <QFile>
#include <QString>
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "database.hpp"
#include "registration.hpp"
#include "circleDetection.hpp"
#include "comparison.hpp"


int coin_value_detection(QString repertory_database, QString repertory_extracted_coins);

int main(int argc, char** argv)
{
    /** ********************************** DETECTION  ********************************** **/

    QDir Dir_extracted_coins("output");
    circleDetection detection(argv[1]);
    detection.detection(false, true, argv[2],true, Dir_extracted_coins);

    /** ********************************** REGISTRATION ********************************** **/

    coin_value_detection("database","output");

    /** ********************************************************************************* **/

    cv::waitKey(0);
    return 0;
}

int coin_value_detection(QString repertory_database, QString repertory_extracted_coins)
{
    database db(repertory_database);
    registration rg("sift","flann");
    QDir Dir_extracted_coins(repertory_extracted_coins);
    QFileInfoList fileList_extracted_coins;
    fileList_extracted_coins.append(Dir_extracted_coins.entryInfoList());
    for( int i = 2; i < fileList_extracted_coins.size(); i++)
    {
        rg.creation_image_extracted_coin(fileList_extracted_coins[i].absoluteFilePath());
        std::cout<<fileList_extracted_coins[i].absoluteFilePath().toStdString()<<std::endl;

        rg.creation_keypoints_extracted_coin();
        rg.creation_descriptors_extracted_coin();


        /// Comparison with our data
        for(std::map<QString,std::string>::iterator it=db.map_data.begin() ; it!=db.map_data.end() ; ++it)
        {
            rg.creation_image_data(it->first);
            QString path = it->first;
            std::cout<<path.toStdString()<<std::endl;

            rg.creation_keypoints_data();
            rg.creation_descriptors_data();
            rg.compute_hypothetical_matches();

            std::vector<cv::Mat> registrationResult = rg.findTransformation(); // registrationResult[0]: homography registrationResult[1]: inliers matrix

            comparison cmp(fileList_extracted_coins[i].absoluteFilePath(), it->first, registrationResult[0], registrationResult[1]);

            float score = cmp.get_inlierScore();

            std::cout<<"score: "<<score<<std::endl;
        }
    }

    cv::waitKey(0);
    return 0;
}
