#include <iostream>
#include <sstream>
#include <fstream>
#include <opencv2/core/core.hpp>

int main(int argc, char** argv){
    if (argc < 2){
        std::cerr << "Usage: ./configOpenvslam calib\n";
        exit(1);
    }
    cv::Mat K, D;
    cv::FileStorage fs(argv[1], cv::FileStorage::READ);
    
    // Quantities to be written
    const std::string camName = "\"GetCameras mono\"";
    const std::string camSetup = "\"monocular\"";
    const std::string camModel = "\"perspective\"";
    //--
    float fx, fy, cx, cy, k1, k2, p1, p2, k3;
    //--
    const float fps = 15.0;
    int cols, rows;
    const std::string colorOrder = "\"RGB\"";
    //--
    const int orbMaxKp = 1000;
    const float orbScaleF = 1.2f;
    const int orbLevels = 8;
    const int initFastTH = 20;
    const int minFastTH = 7;
    //--
    const int gcExposure = 10000;
    const int gcGain = 12;
    const std::string gcDecime = "true";
    std::string serial;


    // Load data from the config files
    fs["K"] >> K;
    fs["D"] >> D;
    //--
    fx = K.at<double>(0,0);
    fy = K.at<double>(1,1);
    cx = K.at<double>(0,2);
    cy = K.at<double>(1,2);
    k1 = D.at<double>(0,0);
    k2 = D.at<double>(0,1);
    p1 = D.at<double>(0,2);
    p2 = D.at<double>(0,3);
    k3 = D.at<double>(0,4);
    //--
    cv::Size imgRes;
    fs["Img_res"] >> imgRes;
    cols = imgRes.width;
    rows = imgRes.height;
    //--
    fs["Serial"] >> serial; 


    // Write the .yml file --> Cannot use Filestorage because it does not support attributes names containing "."
    std::ofstream outF;
    std::stringstream ss;
    //--
    ss.precision(10);
    const std::string fileName = "openvslam_mono_" + serial + ".yml";
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
    ss << "Camera.color_order: " << colorOrder << "\n\n";
    //--
    ss.precision(2);
    ss << "Feature.max_num_keypoints: " << orbMaxKp << "\n";
    ss << "Feature.scale_factor: " << orbScaleF << "\n";
    ss << "Feature.num_levels: " << orbLevels << "\n";
    ss << "Feature.ini_fast_threshold: " << initFastTH << "\n";
    ss << "Feature.min_fast_threshold: " << minFastTH << "\n\n";
    //--
    ss << "Gc.exposure: " << gcExposure << "\n";
    ss << "Gc.gain: " << gcGain << "\n";
    ss << "Gc.decime: " << gcDecime << "\n";
    ss << "Gc.serial: " << "\"" << serial << "\"\n";
    //--
    outF << ss.rdbuf();
    std::cout << "\n\tConfiguration file of openvslam \"" << fileName << "\" written\n";
    
    return 0;   

}
