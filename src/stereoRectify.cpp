#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <iostream>

int main(int argc, char** argv){
    if (argc < 7){
        std::cerr << "Usage ./stereoRectify calibL calibR calibStereo imgWidth imgHeight outFilename" << std::endl;
        exit(1);
    }
    cv::Vec3d T;
    cv::Mat K1,K2,D1,D2,R;
    const int imgWidth = std::stoi(argv[4]);
    const int imgHeight = std::stoi(argv[5]);
    const std::string outFilename = argv[6];
    cv::FileStorage fsL(argv[1], cv::FileStorage::READ);
    cv::FileStorage fsR(argv[2], cv::FileStorage::READ);
    cv::FileStorage fsRT(argv[3], cv::FileStorage::READ);

    // Read calibration of the single cameras
    fsL["K"] >> K1;
    fsL["D"] >> D1;
    fsR["K"] >> K2;
    fsR["D"] >> D2;

    // Read R and T between the left and right cameras
    fsRT["R"] >> R;
    fsRT["T"] >> T;

    // Compute P1, P2, R1, R2 and Q with stereoRectification
    cv::Mat R1, R2, P1, P2, Q;
    cv::stereoRectify(K1, D1, K2, D2, cv::Size(imgWidth, imgHeight), R, T, R1, R2, P1, P2, Q);
    std::cout << "Stereo rectification computed\n";

    // Save P1, P2, R1, R2 and Q
    cv::FileStorage fsO(outFilename, cv::FileStorage::WRITE);
    fsO << "R1" << R1;
    fsO << "R2" << R2;
    fsO << "P1" << P1;
    fsO << "P2" << P2;
    fsO << "Q" << Q;
    std::cout << "\tResults written to " << outFilename << "\n";

    return 0;
}