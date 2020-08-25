#include "utils.h"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <iostream>
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;


// Global variables
const std::string logFolder = "./logSingleCamCalib";           // Script output will be stored in this directory


int main(int argc, char** argv) 
{
    if (argc < 7) 
    {
        std::cerr << "Usage ./singleCamChessCalib boardWidth boardHeight cellSize " 
            "imgFolder extension camSerial" << std::endl;
        return 1;
    }
    const int boardWidth = std::stoi(argv[1]);
    const int boardHeight = std::stoi(argv[2]);
    const float cellSize = std::stof(argv[3]);
    const std::string imgFolder = argv[4];
    const std::string extension = argv[5];
    const std::string camSerial = argv[6];
    std::cout << "Input arguments:\n\tboardWidth: " << boardWidth
        << "\n\tboardHeight: " << boardHeight << "\n\tcellSize: " << cellSize 
        << "\n\tcamSerial: " << camSerial << "\n";

    // Prepare file manager (cv::Filestorage)
    cv::FileStorage fs;

    // Load the filepaths of the images
    std::cout << "Loading image filepaths\n";
    const std::vector<std::string> imgPaths = utils::getImgPaths(imgFolder, extension);
    if (imgPaths.size() > 0)
    {   
        std::cout << "\tFound " << imgPaths.size() << " images\n";
    } 
    else 
    {
        std::cout << "\tNo images Found. Exiting\n";
        return 1;
    }

    // Check that all the images have the same resolution   
    cv::Size imgResolution;       
    if(!utils::checkImgsResolution(imgPaths, imgResolution)) {
        std::cerr << "Found inconsistencies in the image resolutions. Check your data\n";
        exit(1);
    }

    // Create log folders
    fs::create_directory(logFolder);
    fs::create_directory(logFolder + "/" + camSerial);


    /* 
    FIND CHESSBOARD CORNERS AND CREATE ASSOCIATED 3D POINTS
    */
    std::cout << "Looking for chess corners\n";
    std::vector<std::vector<cv::Point2f>> chessCorners2D;                   // Detected chess corners foreach image
    std::vector<std::vector<cv::Point3f>> chessCorners3D;                   // Associated 3D points foreach corner foreach image
    //
    for (uint i=0; i<imgPaths.size(); i++) 
    {
        cv::Mat img, imgGray;
        img = cv::imread(imgPaths[i], cv::IMREAD_COLOR);       
        cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

        // Look for chess corners
        std::vector<cv::Point2f> chessCorners;
        const bool found = utils::findChessCorners(imgGray, boardWidth, boardHeight, chessCorners);
        if (found) 
        {                                                
            std::cout << "\t" << imgPaths[i] << ": Found\n";
            
            // Save chessboard corners coordinates
            fs.open(logFolder + "/" + camSerial + "/chessCorners.yml", cv::FileStorage::APPEND);               
            fs << "image_" + std::to_string(i) << chessCorners; 
            fs.release();

            // Save chessboard corners as image
            utils::saveChessCornersAsImg(logFolder + "/" + camSerial + "/" + std::to_string(i) + ".jpeg", 
                                    img, cv::Size(boardWidth, boardHeight), chessCorners);

            chessCorners2D.emplace_back(chessCorners);
        }
        else 
        {                                                      
            std::cout << "\t" << imgPaths[i] << ": Not found\n";
            continue;
        }
        
        // Set up the 3D points starting from the top-left corner of the chessboard. 
        // Use cellSize to scale them. Assume they are on a plane (z=0)
        std::vector<cv::Point3f> chessObjPoints;
        for (int i = 0; i < boardHeight; i++)
        {
            for (int j = 0; j < boardWidth; j++)
            {
                chessObjPoints.emplace_back(cv::Point3f((float)j * cellSize, (float)i * cellSize, 0));
            }
        }
        chessCorners3D.emplace_back(chessObjPoints);
    }


    /* 
    START CALIBRATION
    */
    cv::Mat K, D;                                               // Camera matrix and distortion vector
    std::vector<cv::Mat> rVecs, tVecs;                          // Rotation and translation of each camera   
    std::vector<double> intrinsicStd, extrinsicStd;             // Standard deviation of intrinsic and extrinsic params
    std::vector<double> perViewReprErr;                         // Per-view reprojection error of the corners
    int flag = 0;                                               // Ignore K4 and K5
    flag |= cv::CALIB_FIX_K4;
    flag |= cv::CALIB_FIX_K5;
    cv::TermCriteria termCrit(cv::TermCriteria::Type::EPS |     // Termination criteria
                    cv::TermCriteria::Type::MAX_ITER, 
                    30, 0.001);
    std::cout << "Calibrating";
    const double reprError = cv::calibrateCamera(
                            chessCorners3D, 
                            chessCorners2D, imgResolution,
                            K, D, rVecs, tVecs, 
                            intrinsicStd, extrinsicStd,
                            perViewReprErr, flag, termCrit);
    std::cout << "\n\tOverall reprojection error: " << reprError << "\n";

    // Write calibration results
    const std::string calFilename = logFolder + "/calib_" + camSerial + ".yml";                  
    fs.open(calFilename, cv::FileStorage::WRITE);      
    fs << "Serial" << camSerial;
    fs << "Img_res" << imgResolution;
    fs << "K" << K;
    fs << "D" << D;
    fs << "Board_width" << boardWidth;
    fs << "Board_weight" << boardHeight;   
    fs << "Cell_size" << cellSize;
    fs.release();
    std::cout << "\tCalibration written to " << calFilename << "\n";

    // Write calibration info
    const std::string calInfoFilename = logFolder + "/info_" + camSerial + ".yml";
    fs.open(calInfoFilename, cv::FileStorage::WRITE);
    fs << "rVecs" << rVecs;
    fs << "tVecs" << tVecs;
    fs << "intrStd" << intrinsicStd;
    fs << "extrStd" << extrinsicStd;
    fs << "perViewReprErr" << perViewReprErr;
    fs.release();
    std::cout << "\tCalibration statistics written to " << calInfoFilename << "\n";

    return 0;
}