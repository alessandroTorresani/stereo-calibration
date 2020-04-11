#include <vector>
#include <string>
#include <experimental/filesystem>
#include <opencv2/core/core.hpp>
namespace fs = std::experimental::filesystem;

namespace utils{
    /** Return all the images with the given extension
     * @param path          Source path
     * @param extension     Image extension
     * @param out           Output vector of image filepaths
    */ 
    void getImgPaths(const std::string path, const std::string extension, std::vector<std::string>& out){
        for (const auto & entry : fs::directory_iterator(path)){
            std::string filetype = entry.path().extension().u8string();
            if (filetype.size() > 0 && filetype.substr(1) == extension)                 // i.e. ".png" ---->  "png"
            out.emplace_back(entry.path().u8string());
        }
    }

     /** Check that all the images in the path have the expected resolution
     * 
     * @param imgPaths      Paths to the images
     * @param expectedRes   Expected resolution
    */ 
    bool checkImgsResolution(const std::vector<std::string> &imgPaths, const cv::Size &expectedRes){
        for (uint i = 0; i < imgPaths.size(); i++){
            if (cv::imread(imgPaths[i], cv::IMREAD_COLOR).size() != expectedRes)
                return false;
        }
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
                            cv::TermCriteria::Type::MAX_ITER, 500, 0.1));
        }
        return found;
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
}