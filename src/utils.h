#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <vector>
#include <string>
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;

namespace utils {

    /** Return the global filepaths of the images having the given extension
     * @param path          Source path
     * @param extension     Image extension
     * @param sort          Sort the filepaths alphabetically
    */ 
    std::vector<std::string> getImgPaths(const std::string path, const std::string extension, const bool sort=true) {
        std::vector<std::string> imgPaths;
        
        // Get path of files matching the given extension
        for (const auto & entry : fs::directory_iterator(path)){
            std::string filetype = entry.path().extension().u8string();
            if (filetype.size() > 0 && filetype.substr(1) == extension)                 // i.e. ".png" ---->  "png"
            imgPaths.emplace_back(entry.path().u8string());
        }

        // Sort them alphabetically
        if (sort)
            std::sort(imgPaths.begin(), imgPaths.end());
        
        return imgPaths;
    }

     /** Check that all the images in the path have the expected resolution
     * 
     * @param imgPaths      Paths to the images
     * @param imgRes        Detected resolution of the images (output)
    */ 
    bool checkImgsResolution(const std::vector<std::string> &imgPaths, cv::Size &imgRes){
        const cv::Size expectedRes = cv::imread(imgPaths[0], cv::IMREAD_COLOR).size();
        
        for (uint i = 0; i < imgPaths.size(); i++){
            if (cv::imread(imgPaths[i], cv::IMREAD_COLOR).size() != expectedRes)
                return false;
        }

        imgRes = expectedRes;
        return true;
    }

    /** Return all the images with the given extension
     * @param imgGray           Source gray image
     * @param boardWidth        Number of corner intersection of the chess row
     * @param boardHeight       Number of corner intersection of the chess column
     * @param chessCorners      Output vector of corners 2D coordinates 
    */ 
    bool findChessCorners(cv::Mat &imgGray, const int &boardWidth, const int &boardHeight, std::vector<cv::Point2f> &chessCorners){
        bool found = cv::findChessboardCorners(imgGray, cv::Size(boardWidth, boardHeight), chessCorners);                               
            
        // If all corners were found, refine corner positions
        if (found){
            cv::cornerSubPix(imgGray, chessCorners, cv::Size(5,5), cv::Size(-1,-1),                                                     
                            cv::TermCriteria(cv::TermCriteria::Type::EPS | 
                            cv::TermCriteria::Type::MAX_ITER, 30, 0.001));
        }

        return found;
    } 

    /** Save image with chessboard corners
     * @param filepath      File output path
     * @param img           Input image
     * @param boardSize     Width and height of the chessboard
     * @param chessCorners  Chess corners
     */
    void saveChessCornersAsImg(const std::string& filepath, cv::Mat& img, const cv::Size &boardSize, std::vector<cv::Point2f> &chessCorners)
    {   
        // Create new image to avoid editing img
        cv::Mat chessImg = img.clone();

        cv::drawChessboardCorners(chessImg, boardSize, chessCorners, true);
        cv::imwrite(filepath, chessImg);
    }

    /** Load chessboard data (width, height, cell size) from left and right calibration files. Check their consistency.
     * Return false if consistencies are found
     * 
     * @param fsL           Opened cv::Filestorage on the left calibration file
     * @param fsR           Opened cv::Filestorage on the right calibration file
     * @param boardWidth    Out chessboard width value 
     * @param boardHeight   Out chessboard height value 
     * @param cellSize      Out chessboard cell size
     * 
    */
    bool loadAndCheckChessboardData(const cv::FileStorage &fsL, const cv::FileStorage &fsR, int &boardWidth, int &boardHeight, float &cellSize){
        int boardWidthL, boardWidthR, boardHeightL, boardHeightR;
        float cellSizeL, cellSizeR;

        fsL["Board_width"] >> boardWidthL;
        fsL["Board_weight"] >> boardHeightL;
        fsL["Cell_size"] >> cellSizeL;
        fsR["Board_width"] >> boardWidthR;
        fsR["Board_weight"] >> boardHeightR;
        fsR["Cell_size"] >> cellSizeR;

        if ((boardWidthL == boardWidthR) && (boardHeightL == boardHeightR) && (cellSizeL == cellSizeR)){
            boardWidth = boardWidthL;
            boardHeight = boardHeightL;
            cellSize = cellSizeL;
            return true;
        } else {
            return false;
        }
    }
    
} // namespace util