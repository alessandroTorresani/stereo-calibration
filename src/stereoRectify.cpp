#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <iostream>

int main(int argc, char** argv){
    if (argc < 4){
        std::cerr << "Usage ./stereoRectify calibL calibR calibStereo" << std::endl;
        exit(1);
    }
    cv::Vec3d T;
    cv::Mat KL,KR,DL,DR,R;
    cv::FileStorage fsL(argv[1], cv::FileStorage::READ);
    cv::FileStorage fsR(argv[2], cv::FileStorage::READ);
    cv::FileStorage fsRT(argv[3], cv::FileStorage::READ);

    cv::Size imgRes;
    fsRT["Img_res"] >> imgRes;
    std::cout << imgRes << std::endl;
 
    // Read calibration of the single cameras
    fsL["K"] >> KL;
    fsL["D"] >> DL;
    fsR["K"] >> KR;
    fsR["D"] >> DR;

    // Read R and T between the left and right cameras
    fsRT["R"] >> R;
    fsRT["T"] >> T;

    // Compute P1, P2, R1, R2 and Q with stereoRectification
    cv::Mat RL, RR, PL, PR, Q;
    cv::stereoRectify(KL, DL, KR, DR, imgRes, R, T, RL, RR, PL, PR, Q);
    std::cout << "Stereo rectification computed\n";

    // Write results
    std::string serialL, serialR;
    fsRT["Serial_left"] >> serialL;
    fsRT["Serial_right"] >> serialR;
    const std::string fsOutName = "cam_rectify_" + serialL + "_to_" + serialR + ".yml";
    //--
    cv::FileStorage fsOut(fsOutName, cv::FileStorage::WRITE);
    fsOut << "R1" << RL;
    fsOut << "R2" << RR;
    fsOut << "P1" << PL;
    fsOut << "P2" << PR;
    fsOut << "Q" << Q;
    std::cout << "\tResults written to " << fsOutName << "\n";

    return 0;
}