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

int detection_method = 1;
QString database_folder_path = "Test_DataBase";
QString extracted_coin_folder_path = "output";
std::string algorithm_features_detection = "sift";
std::string algorithm_matcher = "BF";
int score_method = 1;
bool debug = false;

int coin_value_detection();
void usage_executable(char** argv);

int main(int argc, char** argv)
{
    std::srand(time(NULL));

    /** ********************************** ARGUMENT ************************************ **/
    if( argc < 2)
    {
        usage_executable(argv); exit(0);
    }
    else
    {
        QFile file(argv[1]);
        if(!file.exists())
        {
            std::cout<<"The selected file "<<argv[1]<<" doesn't exist"<<std::endl;
            exit(0);
        }
    }
    if( argc % 2 != 0)
    {
        usage_executable(argv); exit(0);
    }
    for(int i = 2; i < argc; i += 2)
    {
        if(!strcmp(argv[i],"--database"))
        {
            QDir dir(argv[i + 1]);
            if(!dir.exists())
            {
                std::cout<<"The selected folder "<<argv[i + 1]<<" doesn't exist"<<std::endl;
                exit(0);
            }
            database_folder_path = argv[i + 1];
        }
        else if(!strcmp(argv[i],"--detection"))
        {
            if(std::atoi(argv[i + 1]) != 1 && std::atoi(argv[i + 1]) != 2)
            {
                usage_executable(argv); exit(0);
            }
            else
                detection_method = std::atoi(argv[i + 1]);
        }
        else if(!strcmp(argv[i],"--features_detection"))
        {
            if(strcmp(argv[i + 1], "sift") && strcmp(argv[i + 1], "surf") && strcmp(argv[i + 1], "orb"))
            {
                usage_executable(argv); exit(0);
            }
            else
                algorithm_features_detection = argv[i + 1];
        }
        else if(!strcmp(argv[i],"--matcher"))
        {
            if(strcmp(argv[i + 1], "flann") && strcmp(argv[i + 1], "BF"))
            {
                usage_executable(argv); exit(0);
            }
            else
                algorithm_matcher = argv[i + 1];
        }
        else if(!strcmp(argv[i],"--debug"))
        {
            if(strcmp(argv[i + 1], "true") && strcmp(argv[i + 1], "false"))
            {
                usage_executable(argv); exit(0);
            }
            else if(!strcmp(argv[i + 1], "true"))
                debug = true;
            else if(!strcmp(argv[i + 1], "false"))
                debug = false;
        }
        else if(!strcmp(argv[i],"--score"))
        {
            if(std::atoi(argv[i + 1]) != 1 && std::atoi(argv[i + 1]) != 2)
            {
                usage_executable(argv); exit(0);
            }
            else
                score_method = std::atoi(argv[i + 1]);
        }
        else
        {
            usage_executable(argv); exit(0);
        }
    }

    /** ********************************************************************************* **/


    /** **************************** DETECTION OF CIRCLES ******************************* **/

    QDir Dir_extracted_coins(extracted_coin_folder_path);
    circleDetection detection(argv[1], debug);
    detection.detection(false, detection_method, Dir_extracted_coins);
    std::cout<<"Number of detected coin: "<<detection.vector_coins.size()<<std::endl;


    /** ********************************************************************************* **/


    /** ************************** REGISTRATION & COMPARISON **************************** **/

    coin_value_detection();

    /** ********************************************************************************* **/

    cv::waitKey(0);
    return 0;
}

void usage_executable(char **argv)
{
    std::cout<<"Usage: "<<argv[0]<<" std::string> image path"<<std::endl;
    std::cout<<"Option:"<<std::endl;
    std::cout<<"\t--database: <std::string> database folder path"<<std::endl;
    std::cout<<"\t--detection: <int> detection method number: 1 or 2"<<std::endl;
    std::cout<<"\t--features_detection: <std::string> features detection algorithm: 'sift' ,'surf' or 'orb'"<<std::endl;
    std::cout<<"\t--matcher: <std::string> matcher algorithm: 'flann' or 'BF'"<<std::endl;
    std::cout<<"\t--debug: <std::string> display debug images: 'true' or 'false'"<<std::endl;
    std::cout<<"\t--score: <int> score computing method: 1 or 2"<<std::endl;
    std::cout<<"\t\t 1: compute the score with the number of inliers found"<<std::endl;
    std::cout<<"\t\t 2: add of a weighting compute with the repartition of the inliers found"<<std::endl;
    std::cout<<"\nOption Values by default: --database Test_DataBase --detection 1 --features_detection sift --matcher BF --debug false --score 1"<<std::endl;

}

int coin_value_detection()
{
    database db(database_folder_path);
    registration rg(algorithm_features_detection, algorithm_matcher, debug);
    QDir Dir_extracted_coins(extracted_coin_folder_path);
    QFileInfoList fileList_extracted_coins;
    fileList_extracted_coins.append(Dir_extracted_coins.entryInfoList());
    for( int i = 2; i < fileList_extracted_coins.size(); i++)
    {
        rg.creation_image_extracted_coin(fileList_extracted_coins[i].absoluteFilePath());
        std::cout<<fileList_extracted_coins[i].absoluteFilePath().toStdString()<<std::endl;
        // Show the detected coin:
        if(!debug)
        {
            imshow("Detected coin", rg.img_ectracted_coin);
            cv::waitKey(0);
        }

        rg.creation_keypoints_extracted_coin();
        rg.creation_descriptors_extracted_coin();


        /// Comparison with our data
        std::string result_value_coin;
        float score = 0;
        for(std::map<QString,std::string>::iterator it=db.map_data.begin() ; it!=db.map_data.end() ; ++it)
        {
            rg.creation_image_data(it->first);
            QString path = it->first;
            if (debug)
                std::cout<<path.toStdString()<<std::endl;

            rg.creation_keypoints_data();
            rg.creation_descriptors_data();
            rg.compute_hypothetical_matches();

            std::vector<cv::Mat> registrationResult = rg.findTransformation(); // registrationResult[0]: homography registrationResult[1]: inliers matrix

            comparison cmp(fileList_extracted_coins[i].absoluteFilePath(), it->first, registrationResult[0], registrationResult[1], debug);

            float score_temp = cmp.compute_score(score_method, rg.keypoints_extracted_coin, rg.good_matches);

            if(score < score_temp)
            {
                score = score_temp;
                result_value_coin = it->second;
            }
            if (debug)
               std::cout<<"The score is: "<<score_temp<<std::endl;
        }
        std::cout<<"The value of the detected coin is: "<<result_value_coin<<" with a score of "<<score<<std::endl;
    }

    cv::waitKey(0);
    return 0;
}
