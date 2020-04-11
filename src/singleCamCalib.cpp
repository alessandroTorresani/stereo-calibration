#include <iostream>
#include <experimental/filesystem>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include "utils.h"

float computeRMSE(const std::vector<cv::Point3f> &objPoints, const std::vector<cv::Point2f> &imgPoints,
        const cv::Mat &rvec, const cv::Mat &tvec, const cv::Mat &K , const cv::Mat &D) {  
    // Project object points
    std::vector<cv::Point2f> projObj;
    cv::projectPoints(objPoints, rvec, tvec, K, D, projObj);
    
    //Compute RMSE
    float projErr = std::pow(cv::norm(imgPoints, projObj, cv::NORM_L2), 2);
    float RMSE = std::sqrt((projErr) / objPoints.size());

    return RMSE; 
}

int main(int argc, char** argv){
    if (argc < 7){
        std::cerr << "Usage ./singleCamChessCalib boardWidth boardHeight cellSize imgFolder extension camSerial" << std::endl;
        exit(1);
    }
    const int boardWidth = std::stoi(argv[1]);
    const int boardHeight = std::stoi(argv[2]);
    const float cellSize = std::stof(argv[3]);
    const std::string imgFolder = argv[4];
    const std::string extension = argv[5];
    const std::string camSerial = argv[6];
    std::cout << "Input arguments:\n\tboardWidth: " << boardWidth
        << "\n\tboardHeight: " << boardHeight << "\n\tcellSize: " << cellSize 
        << "\n\tcamSerial: " << camSerial << "\n\n";

    // Read the images
    std::cout << "Loading image filepaths\n";
    std::vector<std::string> imgPaths;
    utils::getImgPaths(imgFolder, extension, imgPaths);
    std::sort(imgPaths.begin(), imgPaths.end());
    if (imgPaths.size() > 0){
        std::cout << "\tFound " << imgPaths.size() << " images\n\n";
    } else {
        std::cout << "\tNo images Found. Exiting\n\n";
        exit(1);
    }

    // Check that all the images have the same resolution
    const cv::Size imgRes = cv::imread(imgPaths[0], cv::IMREAD_COLOR).size();               // Read the resolution of the first image of the dataset
    if(!utils::checkImgsResolution(imgPaths, imgRes)){
        std::cerr << "Found inconsistencies in the image resolutions. Check your data\n";
        exit(1);
    }

    // Prepare calibration
    std::vector<std::vector<cv::Point2f>> imgPoints;        // It contains all the chess corners foreach image
    std::vector<std::vector<cv::Point3f>> objPoints;        // It contains all the 3D object points foreach image
    std::cout << "Looking for chess corners\n";
    for (uint i=0; i<imgPaths.size(); i++){
        // Convert image to grayscale
        cv::Mat img, imgGray;
        img = cv::imread(imgPaths[i], cv::IMREAD_COLOR);       
        cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

        // Look for chess corners
        std::vector<cv::Point2f> chessCorners;
        bool found = utils::findChessCorners(imgGray, boardWidth, boardHeight, chessCorners);
        std::cout << "\t" << imgPaths[i] << ":";
        if (found){
            std::cout << " Found\n";
        } else {
            std::cout << " Not found\n";
            continue;
        }
        imgPoints.emplace_back(chessCorners);

        // Set up 3D object points starting from the top-left corner of the chessboard
        std::vector<cv::Point3f> chessObjPoints;
        for (int i = 0; i < boardHeight; i++){
            for (int j = 0; j < boardWidth; j++){
                chessObjPoints.emplace_back(cv::Point3f((float)j * cellSize, (float)i * cellSize, 0));
            }
        }
        objPoints.emplace_back(chessObjPoints);
    }
    std::cout << "\n";

    // Start calibration
    cv::Mat K, D;                                   // Camera matrix and distortion vector
    std::vector<cv::Mat> rVecs, tVecs;              // Rotation and translation of each camera   
    int flag = 0;                                   // Ignore K4 and K5
    flag |= cv::CALIB_FIX_K4;
    flag |= cv::CALIB_FIX_K5;
    std::cout << "Starting calibration";
    cv::calibrateCamera(objPoints, imgPoints, imgRes, K, D, rVecs, tVecs, flag);
    
    // Write calibration results
    const std::string fsOutName = "cam_" + camSerial + ".yml";
    cv::FileStorage fsOut(fsOutName, cv::FileStorage::WRITE);      
    fsOut << "Serial" << camSerial;
    fsOut << "Img_res" << imgRes;
    fsOut << "K" << K;
    fsOut << "D" << D;
    fsOut << "Board_width" << boardWidth;
    fsOut << "Board_weight" << boardHeight;   
    fsOut << "Cell_size" << cellSize;
    std::cout << "\n\tResults written to " << fsOutName << "\n\n";

    // Compute calibration RMSE
    const uint nImgWithCorners = objPoints.size();
    float totErr = 0;
    std::cout << "Computing calibration RMSE of the images with corners";
    for (uint i=0; i<nImgWithCorners; i++){
        float err = computeRMSE(objPoints[i], imgPoints[i], rVecs[i], tVecs[i], K, D);
        std::cout << "\n\t" << i << "-th image RMSE: " << err;
        totErr += err;
    }
    std::cout << "\n\tMean RMSE: " << totErr / (float) nImgWithCorners << "\n";
}