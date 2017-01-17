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
std::string algorithm_features_detection = "orb";
std::string algorithm_matcher = "BF";
int score_method = 1;
bool debug_circleDetection = false;
bool debug_registration = false;
bool debug_comparison = false;
int size = 250;

int coin_value_detection();
void usage_executable(char** argv);

int main(int argc, char** argv)
{
    std::srand(time(NULL));

    /** ********************************** ARGUMENT ************************************ **/
    if( argc % 2 != 0)
    {
        usage_executable(argv); exit(0);
    }
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
            if(strcmp(argv[i + 1], "all") && strcmp(argv[i + 1], "none") && std::atoi(argv[i + 1]) != 1 && std::atoi(argv[i + 1]) != 2 && std::atoi(argv[i + 1]) != 3)
            {
                usage_executable(argv); exit(0);
            }
            else if(!strcmp(argv[i + 1], "all"))
            {
                debug_circleDetection = true;
                debug_registration = true;
                debug_comparison = true;
            }
            else if(std::atoi(argv[i + 1]) == 1)
                debug_circleDetection = true;
            else if(std::atoi(argv[i + 1]) == 2)
                debug_registration = true;
            else if(std::atoi(argv[i + 1]) == 3)
                debug_comparison = true;
        }
        else if(!strcmp(argv[i],"--score"))
        {
            if(std::atoi(argv[i + 1]) != 1 && std::atoi(argv[i + 1]) != 2 && std::atoi(argv[i + 1]) != 3)
            {
                usage_executable(argv); exit(0);
            }
            else
                score_method = std::atoi(argv[i + 1]);
        }
        else if(!strcmp(argv[i],"--size"))
        {
            if(std::atoi(argv[i + 1]) < 250 || std::atoi(argv[i + 1]) > 1000)
            {
                usage_executable(argv); exit(0);
            }
            else
                size = std::atoi(argv[i + 1]);
        }
        else
        {
            usage_executable(argv); exit(0);
        }
    }
    if(algorithm_features_detection == "orb" && algorithm_matcher == "flann")
    {
        std::cout<<"You can't use the descriptor ORB with the matcher FLANN!"<<std::endl;
        exit(0);
    }

    /** ********************************************************************************* **/


    /** **************************** DETECTION OF CIRCLES ******************************* **/

    QDir Dir_extracted_coins(extracted_coin_folder_path);
    if(!Dir_extracted_coins.exists())
    {
        std::cout<<"The folder '"<<extracted_coin_folder_path.toStdString()<<"' to store your extracted coins doesn't exist!"<<std::endl;
        exit(0);
    }
    circleDetection detection(argv[1], detection_method, debug_circleDetection);
    detection.detection();
    detection.extraction(Dir_extracted_coins, score_method, size);
    std::cout<<"Number of detected coin: "<<detection.nb_detected_coin<<std::endl;


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
    std::cout<<"\t--detection: <int> detection method number: 1, 2 or 3"<<std::endl;
    std::cout<<"\t\t 1: detection of circles thanks to Hough transform"<<std::endl;
    std::cout<<"\t\t 2: detection of circles or ellipses thanks to threshold and contour detection"<<std::endl;
    std::cout<<"\t--features_detection: <std::string> features detection algorithm: 'sift' ,'surf' or 'orb'"<<std::endl;
    std::cout<<"\t--matcher: <std::string> matcher algorithm: 'flann' or 'BF'"<<std::endl;
    std::cout<<"\t--debug: <std::string> display debug images: 'all', 'none', '1', '2', or '3'"<<std::endl;
    std::cout<<"\t\t all: display debug for all steps"<<std::endl;
    std::cout<<"\t\t none: doesn't display any debug"<<std::endl;
    std::cout<<"\t\t 1: display debug for circle destection step"<<std::endl;
    std::cout<<"\t\t 2: display debug for registration step"<<std::endl;
    std::cout<<"\t\t 3: display debug for score computation step"<<std::endl;
    std::cout<<"\t--score: <int> score computing method: 1 or 2"<<std::endl;
    std::cout<<"\t\t 1: compute the score with the number of inliers found"<<std::endl;
    std::cout<<"\t\t 2: add of a weighting compute with the repartition of the inliers found"<<std::endl;
    std::cout<<"\t\t 3: compute the score with template matching"<<std::endl;
    std::cout<<"\t--size: <int> size used for the resize (of the database and the extracted coins) before the comparison: between 250 and 1000"<<std::endl;
    std::cout<<"\nOption Values by default: --database Test_DataBase --detection 1 --features_detection orb --matcher BF --debug none --score 1 --size 250"<<std::endl;

}

int coin_value_detection()
{
    double amount = 0;
    database db(database_folder_path);
    registration rg(algorithm_features_detection, algorithm_matcher, debug_registration);
    QDir Dir_extracted_coins(extracted_coin_folder_path);
    QFileInfoList fileList_extracted_coins;
    fileList_extracted_coins.append(Dir_extracted_coins.entryInfoList());
    for( int i = 2; i < fileList_extracted_coins.size(); i++)
    {
        rg.creation_image_extracted_coin(fileList_extracted_coins[i].absoluteFilePath());
        std::cout<<fileList_extracted_coins[i].absoluteFilePath().toStdString()<<std::endl;

        rg.creation_keypoints_extracted_coin();
        rg.creation_descriptors_extracted_coin();


        /// Comparison with our data
        std::string result_value_coin;
        float score = 0;
        for(std::map<QString,std::string>::iterator it=db.map_data.begin() ; it!=db.map_data.end() ; ++it)
        {
            rg.creation_image_data(it->first, size);
            QString path = it->first;
            if (debug_registration || debug_comparison)
                std::cout<<path.toStdString()<<std::endl;

            rg.creation_keypoints_data();
            rg.creation_descriptors_data();
            rg.compute_hypothetical_matches();
            float score_temp = 0;
            if(rg.matches.size() != 0)
            {
                std::vector<cv::Mat> registrationResult = rg.findTransformation(); // registrationResult[0]: homography registrationResult[1]: inliers matrix
                comparison cmp(fileList_extracted_coins[i].absoluteFilePath(), it->first, registrationResult[0], registrationResult[1], score_method, size, debug_comparison);

                score_temp = cmp.compute_score(rg.keypoints_extracted_coin, rg.matches);
            }

            if(score < score_temp)
            {
                score = score_temp;
                result_value_coin = it->second;
            }
            if (debug_comparison)
               std::cout<<"The score is: "<<score_temp<<std::endl;
        }
        if(!result_value_coin.empty())
        {
            std::cout<<"The value of the detected coin is: "<<result_value_coin<<" with a score of "<<score<<std::endl;
            amount += db.get_value_data(result_value_coin);
        }
        else
        {
            std::cout<<"The value of the detected coin wasn't found! There is maybe something wrong with your database! "<<std::endl;
            exit(0);
        }
    }
    std::cout<<"The total amount is: "<<amount<<"€"<<std::endl;

    cv::waitKey(0);
    return 0;
}
