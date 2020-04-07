#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "utils.h"

int main(int argc, char**argv){
    if (argc < 9){
        std::cerr << "Usage: ./undistortRectify calibL calibR calibStereo calibRectify imgL imgR imgFilenameOutL imgFilenameOutR \n";
        exit(1); 
    }
    cv::FileStorage fsL(argv[1], cv::FileStorage::READ);
    cv::FileStorage fsR(argv[2], cv::FileStorage::READ);
    cv::FileStorage fsS(argv[3], cv::FileStorage::READ);
    cv::FileStorage fsRect(argv[4], cv::FileStorage::READ);
    std::string imgPathL = argv[5];
    std::string imgPathR = argv[6]; 
    std::string imgFilenameOutL = argv[7];
    std::string imgFilenameOutR = argv[8]; 

    // Variables to load 
    cv::Mat KL, DL, KR, DR;         // Single camera calibrations
    cv::Mat R;                      // --
    cv::Vec3d T;                    // Stereo calibration
    cv::Mat RL, RR, PL, PR, Q;      // Stereo rectification

    // Load variables
    std::cout << "Loading data\n";
    fsL["K"] >> KL;
    fsL["D"] >> DL;
    fsR["K"] >> KR;
    fsR["D"] >> DR;
    fsS["R"] >> R;
    fsS["T"] >> T;
    fsRect["R1"] >> RL;
    fsRect["R2"] >> RR;
    fsRect["P1"] >> PL;
    fsRect["P2"] >> PR;
    fsRect["Q"] >> Q;
    std::cout << "\tDone\n\n";

    // Load images
    cv::Mat imgL, imgR;
    std::cout << "Loading images\n";
    imgL = cv::imread(imgPathL, cv::IMREAD_COLOR);
    imgR = cv::imread(imgPathR, cv::IMREAD_COLOR);
    if (imgL.size() != imgR.size()){
        std::cerr << "Left and right images must have the same resolution\n";
        exit(1);
    }
    std::cout << "\tDone\n\n";

    // Compute mappping transformations
    cv::Mat mapXL, mapYL, mapXR, mapYR;
    std::cout << "Computing mapping transformations\n";
    cv::initUndistortRectifyMap(KL, DL, RL, PL, imgL.size(), CV_32F, mapXL, mapYL); 
    cv::initUndistortRectifyMap(KR, DR, RR, PR, imgR.size(), CV_32F, mapXR, mapYR);
    std::cout << "\tDone\n\n";
    
    // Apply rectification
    cv::Mat imgRectL, imgRectR;
    std::cout << "Remapping images\n";
    cv::remap(imgL, imgRectL, mapXL, mapYL, cv::INTER_LINEAR);
    cv::remap(imgR, imgRectR, mapXR, mapYR, cv::INTER_LINEAR);
    std::cout << "\tDone\n\n";

    // Save rectified images
    std::cout << "Saving rectified images\n";
    cv::imwrite(imgFilenameOutL, imgRectL);
    cv::imwrite(imgFilenameOutR, imgRectR);
    std::cout << "\t" << imgFilenameOutL << " and " << 
        imgFilenameOutR << " saved\n";
}