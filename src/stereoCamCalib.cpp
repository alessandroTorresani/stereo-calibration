#include "utils.h"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <iostream>
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;


// Global variables
const std::string logFolder = "./logStereoCamCalib";           // Script output will be stored in this directory


int main(int argc, char** argv){
    if (argc < 6){
        std::cerr << "Usage: ./stereoCamCalib calibL calibR imgFolderL imgFolderR extension\n";
        exit(1);
    }
    const std::string calibL = argv[1];
    const std::string calibR = argv[2];
    const std::string imgFolderL = argv[3];
    const std::string imgFolderR = argv[4];
    const std::string extension = argv[5];
    std::cout << "Input arguments:\n\tCalibL: " << calibL
        << "\n\tCalibR: " << calibR << "\n\tImgFolderL size: " << imgFolderL 
        << "\n\tImgFolderR: " << imgFolderR << "\n\tExtension: " << extension << "\n";
    
    // Prepare file managers (cv::Filestorage)
    cv::FileStorage fs;
    cv::FileStorage fsL(calibL, cv::FileStorage::READ);                             // Read left camera configuration file
    cv::FileStorage fsR(calibR, cv::FileStorage::READ);                             // Read right camera configuration file
    
    // Get chessboard info from the single camera calibrations. Check consistency.
    int boardWidth, boardHeight;
    float cellSize;
    if (!utils::loadAndCheckChessboardData(fsL, fsR, boardWidth, boardHeight, cellSize)){
        std::cerr << "Chessboard data inconsistencies between left and right calibrations. Check your data\n";
        return 1;
    }
    
    // Load left and right image filepaths
    const std::vector<std::string> imgPathsL = utils::getImgPaths(imgFolderL, extension);
    const std::vector<std::string> imgPathsR = utils::getImgPaths(imgFolderR, extension);
    if (imgPathsL.size() != imgPathsR.size()){
        std::cerr << "Error, left and right images must be of the same number\n";
        return 1;
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

    // Read serial of the cameras
    std::string serialL, serialR;
    fsL["Serial"] >> serialL;
    fsR["Serial"] >> serialR;

    // Create log folders
    fs::create_directory(logFolder);
    fs::create_directory(logFolder + "/" + serialL);
    fs::create_directory(logFolder + "/" + serialR);

    /* 
    FIND CHESSBOARD CORNERS AND CREATE ASSOCIATED 3D POINTS
    */
    std::vector<std::vector<cv::Point2f>> chessCorners2DL, chessCorners2DR;     // Detected chess corners foreach image (left and right)
    std::vector<std::vector<cv::Point3f>> chessCorners3D;                       // Associated 3D points foreach corner foreach image (same left and right)
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
        std::vector<cv::Point2f> chessCornersL, chessCornersR;
        foundL = utils::findChessCorners(imgGrayL, boardWidth, boardHeight, chessCornersL);
        foundR = utils::findChessCorners(imgGrayR, boardWidth, boardHeight, chessCornersR);
        std::cout << "\t" << imgPathsL[i] << " - " << imgPathsR[i] << ": ";
        if (!foundL || !foundR){
            std::cout << " Not found\n";
            continue;
        } else {
            std::cout << " Found\n";
            chessCorners2DL.emplace_back(chessCornersL);
            chessCorners2DR.emplace_back(chessCornersR);
            
            // Save chessboard corners coordinates
            fs.open(logFolder + "/" + serialL + "/chesscorners.yml", cv::FileStorage::APPEND);
            fs << "Image_" + std::to_string(i) << chessCornersL;
            fs.release();
            fs.open(logFolder + "/" + serialR + "/chesscorners.yml", cv::FileStorage::APPEND);
            fs << "Image_" + std::to_string(i) << chessCornersR;
            fs.release();

            // Save chessboard corners as image
            utils::saveChessCornersAsImg(logFolder + "/" + serialL + "/" + std::to_string(i) + ".jpeg", 
                                    imgL, cv::Size(boardWidth, boardHeight), chessCornersL);
            utils::saveChessCornersAsImg(logFolder + "/" + serialR + "/" + std::to_string(i) + ".jpeg", 
                                    imgR, cv::Size(boardWidth, boardHeight), chessCornersR);
        }
        
        // Set up the 3D points starting from the top-left corner of the chessboard. 
        // Use cellSize to scale them. Assume they are on a plane (z=0)
        std::vector<cv::Point3f> chessObjPoints;
        for (int i = 0; i < boardHeight; i++){
            for (int j = 0; j < boardWidth; j++){
                chessObjPoints.emplace_back(cv::Point3f((float)j * cellSize, (float)i * cellSize, 0));
            }
        }
        chessCorners3D.emplace_back(chessObjPoints);
    }


    /* 
    START STEREO CALIBRATION
    */
    std::cout << "Starting stereo calibration\n";
    cv::Mat KL, KR, DL, DR;                                             // Load single camera calibrations
    fsL["K"] >> KL;
    fsL["D"] >> DL;
    fsR["K"] >> KR;
    fsR["D"] >> DR;
    
    //--
    cv::Mat R, F, E;                                                    // Rotation, fundamental and essential matrix (of the right image wrt the left image)
    cv::Vec3d T;                                                        // Translation vector (of the right image wrt the left image)
    cv::Mat perViewReprErr;                                             // Per-view reprojection error of the corners
    int flag = 0;
    flag |= cv::CALIB_FIX_INTRINSIC;                                    // Fix the intrinsics (KL, KR, DL, DR) estimated in the single camera calibrations
    //flag |= cv::CALIB_USE_INTRINSIC_GUESS;
    cv::TermCriteria termCrit(cv::TermCriteria::Type::EPS |             // Termination criteria
                    cv::TermCriteria::Type::MAX_ITER, 
                    30, 0.001);
    //--
    const double reprError = cv::stereoCalibrate(chessCorners3D, chessCorners2DL, chessCorners2DR, KL, DL, KR, DR, 
        imgResL, R, T, E, F, perViewReprErr, flag, termCrit);
    std::cout << "\tOverall reprojection error: " << reprError << "\n";
    
    // Write calibration results
    std::string calFilename = logFolder + "/calib_stereo_" + serialL + "_to_" + serialR + ".yml";
    //--
    fs.open(calFilename, cv::FileStorage::WRITE);         
    fs << "Serial_left" << serialL;
    fs << "Serial_right" << serialR;
    fs << "Img_res" << imgResL;
    fs << "R" << R;
    fs << "T" << T;
    fs << "E" << E;
    fs << "F" << F;
    fs.release();
    fsL.release();
    fsR.release();
    std::cout << "\tCalibration written to " << calFilename << "\n";

    // Write calibration info
    const std::string calInfoFilename = logFolder + "/info_stereo_" + serialL + "_to_" + serialR + ".yml";
    fs.open(calInfoFilename, cv::FileStorage::WRITE);
    fs << "perViewReprErr" << perViewReprErr;
    fs.release();
    std::cout << "\tCalibration statistics written to " << calInfoFilename << "\n";


    return 0;
}