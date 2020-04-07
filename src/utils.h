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

    /** Check that all the images of the calibration dataset have the same resolution
     * 
     * @param imgPaths  Paths to the images
     * @param res       Out resolution of the dataset, if tests succedes
    */ 
    bool checkImgsResolution(const std::vector<std::string> &imgPaths, cv::Size &res){
        cv::Size refRes = cv::imread(imgPaths[0], cv::IMREAD_COLOR).size();                 // Compare img resolution of the first image with all the other ones
        for (uint i = 1; i < imgPaths.size(); i++){
            if (cv::imread(imgPaths[i], cv::IMREAD_COLOR).size() != refRes)
                return false;
        }
        res = refRes;
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

        fsL["BoardWidth"] >> boardWidthL;
        fsL["BoardHeight"] >> boardHeightL;
        fsL["CellSize"] >> cellSizeL;
        fsR["BoardWidth"] >> boardWidthR;
        fsR["BoardHeight"] >> boardHeightR;
        fsR["CellSize"] >> cellSizeR;

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