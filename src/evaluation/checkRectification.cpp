#include "../utils.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <experimental/filesystem>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <numeric>
#include <regex>
#include <math.h>
namespace fs = std::experimental::filesystem;


// Global variables
const std::string logFolder = "./logCheckRectification";           // Script output will be stored in this directory


/**
 * Save rectified image in the log folder
 * 
 * @param imgRect       Rectified image
 * @param refImgPath    Absolute filepath of the original image (from which the rectified image was generated).
 *                      This is used to name the rectified image as the original image.
 * @param serial        Serial of the camera that acquired the image. Used to name the rectified image
 */  
void saveRectifiedImage(const cv::Mat &imgRect, const std::string &origImgPath, const std::string &serial);

/**
 * Compute median, mean and standard deviation of a vector of float
 * 
 * @param vec           Input vector
 * @param median        Output median
 * @param mean          Output mean
 * @param std           Output standard deviation
 */
void computeStatistics(std::vector<float> vec, float &median, float &mean, float &std);


int main(int argc, char** argv) {
    if (argc < 8){
        std::cerr << "Usage: ./checkRectification calibL calibR calibStereo calibRectify imgFolderL imgFolderR extension\n";
        return 1;
    }
    cv::FileStorage fsL(argv[1], cv::FileStorage::READ);        // Reader of left camera calibration file
    cv::FileStorage fsR(argv[2], cv::FileStorage::READ);        // Reader of right camera calibration file
    cv::FileStorage fsS(argv[3], cv::FileStorage::READ);        // Reader of stereo calibration file
    cv::FileStorage fsRect(argv[4], cv::FileStorage::READ);     // Reader of the rectificaton file
    const std::string imgFolderL = argv[5];                     // Path to the left image folder
    const std::string imgFolderR = argv[6];                     // Path to the right image folder
    const std::string extension = argv[7];                      // Image extension/format
    //
    std::string serialL, serialR;                               // Serial of the left and right cameras
    //
    int boardWidth, boardHeight;                                // Chessboard width, height and cell size.
    float cellSize;                                             // Chessboard cell size
    //
    cv::Mat KL, DL, KR, DR;                                     // Instrinsic parameters of the cameras
    cv::Mat R;                                                  // Relative rotation of the left camera wrt the right camera                   
    cv::Vec3d T;                                                // Relative translation the left camera wrt the right camera
    cv::Mat RL, RR, PL, PR;                                     // Rectification projection and rotation m
    

    // Load left and right image paths
    const std::vector<std::string> imgPathsL = utils::getImgPaths(imgFolderL, extension);
    const std::vector<std::string> imgPathsR = utils::getImgPaths(imgFolderR, extension);
    if (imgPathsL.size() == 0 || imgPathsR.size() == 0) {
        std::cerr << "Empty image folders\n";
        return 1;
    }
    if (imgPathsL.size() != imgPathsR.size()){
        std::cerr << "Error, left and right images must be of the same number\n";
        return 1;
    }

    // Load camera serials
    fsS["Serial_left"] >> serialL;
    fsS["Serial_right"] >> serialR;

    // Load chessboard information
    if (!utils::loadAndCheckChessboardData(fsL, fsR, boardWidth, boardHeight, cellSize)){
        std::cerr << "Chessboard data inconsistencies between left and right calibrations. Check your data\n";
        return 1;
    }

    // Load intrinsic parameters of the cameras
    fsL["K"] >> KL;
    fsL["D"] >> DL;
    fsR["K"] >> KR;
    fsR["D"] >> DR;

    // Load stereo calibration (R and t between left and right cameras)
    fsS["R"] >> R;
    fsS["T"] >> T;

    // Load rectification matrices
    fsRect["R1"] >> RL;
    fsRect["R2"] >> RR;
    fsRect["P1"] >> PL;
    fsRect["P2"] >> PR;

    // Create output/log folder and folders for the rectified images
    fs::create_directory(logFolder);
    fs::create_directory(logFolder + "/" + serialL + "Rect");
    fs::create_directory(logFolder + "/" + serialR + "Rect");


    /* EVALUATION LOOP
    For each image pair (l,r), compute:
        1) Rectified images (lRect, rRect)
        2) Chess corners on lRect and rRect
        3) For each corner pair (cl,cr), compute yDisparity = abs(cl.y - cr.y)
    */
    int validPairs = 0;                                                                 // Number of pairs with detected chessboard corners in both images (left,right)
    float accumMedian = 0, accumMean = 0, accumStd = 0;                                 // Dataset Y disparity statistics
    //
    std::ofstream evalOut;                                                              // File logger                                                      
    evalOut.open(logFolder + "/yDisparities.txt", std::ios::app);
    evalOut << "#Median mean std\n";                                                    // Each line is an image in reading order (images sorted alphabetically)
    //
    for (unsigned int i=0; i<imgPathsL.size(); i++) {
        cv::Mat imgL, imgR, imgRectL, imgRectR;
        imgL = cv::imread(imgPathsL[i], cv::IMREAD_COLOR);
        imgR = cv::imread(imgPathsR[i], cv::IMREAD_COLOR);

        // Rectify the images
        cv::Mat mapXL, mapYL, mapXR, mapYR;
        cv::initUndistortRectifyMap(KL, DL, RL, PL, imgL.size(), CV_32F, mapXL, mapYL); 
        cv::initUndistortRectifyMap(KR, DR, RR, PR, imgR.size(), CV_32F, mapXR, mapYR);
        cv::remap(imgL, imgRectL, mapXL, mapYL, cv::INTER_LINEAR);
        cv::remap(imgR, imgRectR, mapXR, mapYR, cv::INTER_LINEAR);

        // Compute the chess corners and save them (on the rectified images)
        std::vector<cv::Point2f> chessCornersL, chessCornersR;
        cv::Mat imgRectGrayL, imgRectGrayR;
        cv::cvtColor(imgRectL, imgRectGrayL, cv::COLOR_BGR2GRAY);
        cv::cvtColor(imgRectR, imgRectGrayR, cv::COLOR_BGR2GRAY);
        const bool cornersFoundL = utils::findChessCorners(imgRectGrayL, boardWidth, boardHeight, chessCornersL);
        const bool cornersFoundR = utils::findChessCorners(imgRectGrayR, boardWidth, boardHeight, chessCornersR);
        //
        cv::drawChessboardCorners(imgRectL, cv::Size(boardWidth, boardHeight), chessCornersL, cornersFoundL);
        cv::drawChessboardCorners(imgRectR, cv::Size(boardWidth, boardHeight), chessCornersR, cornersFoundR);
        saveRectifiedImage(imgRectL, imgPathsL[i], serialL);
        saveRectifiedImage(imgRectR, imgPathsR[i], serialR);
        //
        if (!cornersFoundL || !cornersFoundR) {
            evalOut << "Nan Nan NaN\n"; 
            continue;
        }

        // Compute y disparities between left and right corners
        std::vector<float> yDisparities;
        yDisparities.reserve(chessCornersL.size());
        for (unsigned int i=0; i<chessCornersL.size(); i++) {
            yDisparities.emplace_back(sqrt(pow(chessCornersL[i].y - chessCornersR[i].y, 2)));
        }

        // Compute statistics 
        float median, mean, std;
        computeStatistics(yDisparities, mean, median, std);
        evalOut << median << " " << mean << " " << std << std::endl;

        // Update dataset statistics
        accumMedian += median;
        accumMean += mean;
        accumStd += std;
        validPairs++;
    }
    evalOut.close();
    std::cout << "\nEVALUATION TERMINATED***\n\tY disparity on the whole dataset (" <<
        validPairs << " valid image pairs) [median, mean, std]: [" <<
        accumMedian / (float) validPairs << ", " <<
        accumMean / (float) validPairs <<  ", " <<
        accumStd / (float) validPairs << "]" << std::endl;

    return 0;
}

void saveRectifiedImage(const cv::Mat &imgRect, const std::string &origImgPath, const std::string &serial) {
    // Get the original image name from the absolute path
    std::regex reg("([^\\/]+)+$");
    auto words_begin = std::regex_iterator(origImgPath.begin(), origImgPath.end(), reg);
    auto words_end = std::sregex_iterator();
    std::string origImgName;
    for (std::sregex_iterator i = words_begin; i != words_end; ++i) {
        origImgName = i->str();
    }

    // Save the rectified image
    cv::imwrite(logFolder + "/" + serial + "Rect/" + origImgName, imgRect);
}

void computeStatistics(std::vector<float> vec, float &median, float &mean, float &std) {
    // Sort the vector and compute the median
    std::sort(vec.begin(), vec.end());
    median = vec[vec.size()/2];

    // Compute the mean
    mean = (std::accumulate(vec.begin(), vec.end(), 0.0) / vec.size());

    // Compute the standard deviation
    double accum = 0.0;
    std::for_each (std::begin(vec), std::end(vec), [&](const double d) {
        accum += (d - mean) * (d - mean);
    });
    std = sqrt(accum / (vec.size()-1));
}