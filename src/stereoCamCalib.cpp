#include <iostream>
#include <experimental/filesystem>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include "utils.h"

int main(int argc, char** argv){
    if (argc < 6){
        std::cerr << "Usage: ./stereoCamCalib calibL calibR imgFolderL imgFolderR extension";
        exit(1);
    }
    const std::string calibL = argv[1];
    const std::string calibR = argv[2];
    const std::string imgFolderL = argv[3];
    const std::string imgFolderR = argv[4];
    const std::string extension = argv[5];
    std::cout << "Input arguments:\n\tCalibL: " << calibL
        << "\n\tCalibR: " << calibR << "\n\tImgFolderL size: " << imgFolderL 
        << "\n\tImgFolderR: " << imgFolderR << "\n\tExtension: " << extension << "\n\n";
    
    // Get chessboard info from the single camera calibrations. Check consistency.
    cv::FileStorage fsL(calibL, cv::FileStorage::READ);
    cv::FileStorage fsR(calibR, cv::FileStorage::READ);
    int boardWidth, boardHeight;
    float cellSize;
    if (!utils::loadAndCheckChessboardData(fsL, fsR, boardWidth, boardHeight, cellSize)){
        std::cerr << "Chessboard data inconsistencies between left and right calibrations. Check your data\n";
        exit(1);
    }
    
    // Load left and right image filepaths and sort them alphabetically
    std::vector<std::string> imgPathsL, imgPathsR;
    utils::getImgPaths(imgFolderL, extension, imgPathsL);
    utils::getImgPaths(imgFolderR, extension, imgPathsR);
    std::sort(imgPathsL.begin(), imgPathsL.end());
    std::sort(imgPathsR.begin(), imgPathsR.end());
    if (imgFolderL.size() != imgFolderR.size()){
        std::cerr << "Error, left and right images must be of the same number\n";
        exit(1);
    }

    /* Check that:
        1)The images have the same resolution of the images used in the single cam calibrations
        2)Left and right images resolution are equal
    */ 
    cv::Size imgResL, imgResR;
    fsL["Img_res"] >> imgResL;
    fsR["Img_res"] >> imgResR;
    if(!utils::checkImgsResolution(imgPathsL, imgResL)){
        std::cerr << "Found inconsistencies in the resolution of the left images. Check your data\n";
        exit(1);
    }
    if(!utils::checkImgsResolution(imgPathsR, imgResR)){
        std::cerr << "Found inconsistencies in the resolution of the right images. Check your data\n";
        exit(1);
    }
    if (imgResL != imgResR){
        std::cerr << "Left and right images must have the same resolution. \n";
        exit(1);
    }

    // Find corner points and create the correspoding 3D points
    std::vector<std::vector<cv::Point2f>> imgPtsL, imgPtsR;         // Image (chess corners) points of left and right images
    std::vector<std::vector<cv::Point3f>> objPts;                   // Object (3D corner) points (same for left and right)
    std::cout << "Looking for chess corners\n";
    for (uint i=0; i<imgPathsL.size(); i++){
        // Convert images to grayscale
        cv::Mat imgL, imgR, imgGrayL, imgGrayR;
        imgL = cv::imread(imgPathsL[i], cv::IMREAD_COLOR);
        imgR = cv::imread(imgPathsR[i], cv::IMREAD_COLOR);
        cv::cvtColor(imgL, imgGrayL, cv::COLOR_BGR2GRAY);
        cv::cvtColor(imgR, imgGrayR, cv::COLOR_BGR2GRAY);

        // Find chess corners
        bool foundL, foundR;
        std::vector<cv::Point2f> _imgPtsL, _imgPtsR;
        foundL = utils::findChessCorners(imgGrayL, boardWidth, boardHeight, _imgPtsL);
        foundR = utils::findChessCorners(imgGrayR, boardWidth, boardHeight, _imgPtsR);
        std::cout << "\t" << imgPathsL[i] << " - " << imgPathsR[i] << ": ";
        if (!foundL || !foundR){
            std::cout << " Not found\n";
            continue;
        } else {
            std::cout << " Found\n";
        }
        imgPtsL.emplace_back(_imgPtsL);
        imgPtsR.emplace_back(_imgPtsR);

        // Set up 3D object points starting from the top-left corner of the chessboard
        std::vector<cv::Point3f> _objPts;
        for (int i = 0; i < boardHeight; i++){
            for (int j = 0; j < boardWidth; j++){
                _objPts.emplace_back(cv::Point3f((float)j * cellSize, (float)i * cellSize, 0));
            }
        }
        objPts.emplace_back(_objPts);
    }

    // Do stereo calibration
    std::cout << "Starting stereo calibration\n";
    cv::Mat KL, KR, DL, DR;                                             // Load single camera calibrations
    fsL["K"] >> KL;
    fsL["D"] >> DL;
    fsR["K"] >> KR;
    fsR["D"] >> DR;
    //--
    cv::Mat R, F, E;                                                    // Variables to be estimated
    cv::Vec3d T;
    int flag = 0;
    flag |= cv::CALIB_FIX_INTRINSIC;                                    // Fix the intrinsics (KL, KR, DL, DR) estimated in the single camera calibrations
    //--
    cv::stereoCalibrate(objPts, imgPtsL, imgPtsR, KL, DL, KR, DR, 
        imgResL, R, T, E, F, flag);
    
    // Write results
    std::string serialL, serialR;
    fsL["Serial"] >> serialL;
    fsR["Serial"] >> serialR;
    std::string fsOutName = "cam_stereo_" + serialL + "_to_" + serialR + ".yaml";
    //--
    cv::FileStorage fsOut(fsOutName, cv::FileStorage::WRITE);         
    fsOut << "Serial_left" << serialL;
    fsOut << "Serial_right" << serialR;
    fsOut << "Img_res" << imgResL;
    fsOut << "R" << R;
    fsOut << "T" << T;
    fsOut << "E" << E;
    fsOut << "F" << F;
    std::cout << "\tResults written to " << fsOutName << "\n\n";

    return 0;
}