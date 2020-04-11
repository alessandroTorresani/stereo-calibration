#include <iostream>
#include <sstream>
#include <fstream>
#include <opencv2/core/core.hpp>

int main(int argc, char** argv){
    if (argc < 4){
        std::cerr << "Usage: ./configOpenvslam calibL calibR calibRectify\n";
        exit(1);
    }
    cv::Mat KL, DL, KR, DR, PL, PR, RL, RR;                     // PR not needed,as PL = PR
    cv::FileStorage fsL(argv[1], cv::FileStorage::READ);
    cv::FileStorage fsR(argv[2], cv::FileStorage::READ);
    cv::FileStorage fsRect(argv[3], cv::FileStorage::READ);
    
    // Quantities to be written
    const std::string camName = "\"GetCameras stereo\"";
    const std::string camSetup = "\"stereo\"";
    const std::string camModel = "\"perspective\"";
    //--
    float fx, fy, cx, cy;
    const float k1 = 0, k2 = 0, p1 = 0, p2 = 0, k3 = 0;
    //--
    const float fps = 15.0;
    int cols, rows;
    double focalXBaseline;
    const std::string colorOrder = "\"RGB\"";
    //--
    std::string stereoRectParams;
    //--
    const int orbMaxKp = 1000;
    const float orbScaleF = 1.2f;
    const int orbLevels = 8;
    const int initFastTH = 20;
    const int minFastTH = 7;
    //--
    const int initMinTriangKp = 100;
    //-- 
    // Skip pangolin parameters
    //--
    const int gcExposure = 10000;
    const int gcGain = 12;
    const std::string gcDecime = "true";
    std::string serialL, serialR;


    // Load data from the config files
    fsL["K"] >> KL;
    fsL["D"] >> DL;
    fsRect["R1"] >> RL;
    fsRect["P1"] >> PL;
    fsR["K"] >> KR;
    fsR["D"] >> DR;
    fsRect["R2"] >> RR;
    fsRect["P2"] >> PR;
    //--
    fx = PL.at<double>(0,0);
    fy = PL.at<double>(1,1);
    cx = PL.at<double>(0,2);
    cy = PL.at<double>(1,2);
    //--
    cv::Size imgResL, imgResR;
    fsL["Img_res"] >> imgResL;
    fsR["Img_res"] >> imgResR;
    if (imgResL != imgResR){
        std::cerr << "Left and right calibration has different value of ImgRes. Check your data\n";
        exit(1);
    }
    cols = imgResL.width;
    rows = imgResL.height;
    //--
    focalXBaseline = -PR.at<double>(0,3);
    //--
    fsL["Serial"] >> serialL; 
    fsR["Serial"] >> serialR;


    // Write the .yml file --> Cannot use Filestorage because it does not support attributes names containing "."
    std::ofstream outF;
    std::stringstream ss;
    //--
    ss.precision(10);
    const std::string fileName = "openvslam_stereo_" + serialL + "_to_" + serialR + ".yml";
    //--
    outF.open(fileName);
    ss << "Camera.name: " << camName << "\n";
    ss << "Camera.setup: " << camSetup << "\n";
    ss << "Camera.model: " << camModel << "\n\n";
    //--
    ss << "Camera.fx: " << fx << "\n";
    ss << "Camera.fy: " << fy << "\n";
    ss << "Camera.cx: " << cx << "\n";
    ss << "Camera.cy: " << cy << "\n\n";
    //--
    ss << "Camera.k1: " << k1 << "\n";
    ss << "Camera.k2: " << k2 << "\n";
    ss << "Camera.p1: " << p1 << "\n";
    ss << "Camera.p2: " << p2 << "\n";
    ss << "Camera.k3: " << k3 << "\n\n";
    //--
    ss << "Camera.fps: " << fps << "\n";
    ss << "Camera.cols: " << cols << "\n";
    ss << "Camera.rows: " << rows << "\n";
    ss << "Camera.focal_x_baseline: " << focalXBaseline << "\n";
    ss << "Camera.color_order: " << colorOrder << "\n\n";
    //--
    stereoRectParams = "StereoRectifier.K_left: [" + std::to_string(KL.at<double>(0,0)) + ", " + std::to_string(KL.at<double>(0,1)) + ", " + std::to_string(KL.at<double>(0,2)) + 
        ", " + std::to_string(KL.at<double>(1, 0)) + ", " + std::to_string(KL.at<double>(1,1)) + ", " + std::to_string(KL.at<double>(1,2)) + 
        ", " + std::to_string(KL.at<double>(2, 0)) + ", " + std::to_string(KL.at<double>(2,1)) + ", " + std::to_string(KL.at<double>(2,2)) +"]\n" + 
        "StereoRectifier.D_left: [" + std::to_string(DL.at<double>(0,0)) + ", " + std::to_string(DL.at<double>(0,1)) + ", " + std::to_string(DL.at<double>(0,2)) + 
        ", " + std::to_string(DL.at<double>(0, 3)) + ", " + std::to_string(DL.at<double>(0,4)) +"]\n" +
        "StereoRectifier.R_left: [" + std::to_string(RL.at<double>(0,0)) + ", " + std::to_string(RL.at<double>(0,1)) + ", " + std::to_string(RL.at<double>(0,2)) + 
        ", " + std::to_string(RL.at<double>(1, 0)) + ", " + std::to_string(RL.at<double>(1,1)) + ", " + std::to_string(RL.at<double>(1, 2)) + 
        ", " + std::to_string(RL.at<double>(2, 0)) + ", " + std::to_string(RL.at<double>(2,1)) + ", " + std::to_string(RL.at<double>(2, 2)) + "]\n" + 
        "StereoRectifier.K_right: [" + std::to_string(KR.at<double>(0,0)) + ", " + std::to_string(KR.at<double>(0,1)) + ", " + std::to_string(KR.at<double>(0,2)) + 
        ", " + std::to_string(KR.at<double>(1, 0)) + ", " + std::to_string(KR.at<double>(1,1)) + ", " + std::to_string(KR.at<double>(1,2)) + 
        ", " + std::to_string(KR.at<double>(2, 0)) + ", " + std::to_string(KR.at<double>(2,1)) + ", " + std::to_string(KR.at<double>(2,2)) +"]\n" + 
        "StereoRectifier.D_right: [" + std::to_string(DR.at<double>(0,0)) + ", " + std::to_string(DR.at<double>(0,1)) + ", " + std::to_string(DR.at<double>(0,2)) + 
        ", " + std::to_string(DR.at<double>(0, 3)) + ", " + std::to_string(DR.at<double>(0,4)) +"]\n" +
        "StereoRectifier.R_right: [" + std::to_string(RR.at<double>(0,0)) + ", " + std::to_string(RR.at<double>(0,1)) + ", " + std::to_string(RR.at<double>(0,2)) + 
        ", " + std::to_string(RR.at<double>(1, 0)) + ", " + std::to_string(RR.at<double>(1,1)) + ", " + std::to_string(RR.at<double>(1, 2)) + 
        ", " + std::to_string(RR.at<double>(2, 0)) + ", " + std::to_string(RR.at<double>(2,1)) + ", " + std::to_string(RR.at<double>(2, 2)) + "]\n\n";
    ss << stereoRectParams;
    //--
    ss.precision(2);
    ss << "Feature.max_num_keypoints: " << orbMaxKp << "\n";
    ss << "Feature.scale_factor: " << orbScaleF << "\n";
    ss << "Feature.num_levels: " << orbLevels << "\n";
    ss << "Feature.ini_fast_threshold: " << initFastTH << "\n";
    ss << "Feature.min_fast_threshold: " << minFastTH << "\n\n";
    //--
    ss << "Initializer.num_min_triangulated_pts: " << initMinTriangKp << "\n\n";
    //--
    ss << "Gc.exposure: " << gcExposure << "\n";
    ss << "Gc.gain: " << gcGain << "\n";
    ss << "Gc.decime: " << gcDecime << "\n";
    ss << "Gc.serial.left: " << "\"" << serialL << "\"\n";
    ss << "Gc.serial.right: " << "\"" << serialR << "\"\n";
    //--
    outF << ss.rdbuf();
    std::cout << "\n\tConfiguration file of openvslam \"" << fileName << "\" written\n";
    
    return 0;   

}
