#include "popt_pp.h"
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <iostream>
#include <experimental/filesystem>

using namespace std;
using namespace cv;
namespace fs = std::experimental::filesystem;

vector< vector< Point3f > > object_points;
vector< vector< Point2f > > imagePoints1, imagePoints2;
vector< Point2f > corners1, corners2;
vector< vector< Point2f > > left_img_points, right_img_points;
Mat img1, img2, gray1, gray2;

void getImgFilepaths(string _path, string _filetype, vector<string>& _out){
  for (const auto & entry : fs::directory_iterator(_path)){
    string filetype = entry.path().extension().u8string();
    if (filetype.size() > 0 && filetype.substr(1) == _filetype)                 // filetype.substr(1): .png to png
      _out.emplace_back(entry.path().u8string());
  }
}

void load_image_points(int board_width, int board_height, float square_size, vector<string>& imgFilepathsLeft, vector<string>& imgFilepathsRight) {

  Size board_size = Size(board_width, board_height);
  int board_n = board_width * board_height;

  if (imgFilepathsLeft.size() != imgFilepathsRight.size()){
    std::cerr << "The number of left and right images must be the same" << std::endl;
    exit(1);
  }

  for (int i = 0; i < imgFilepathsLeft.size(); i++) {
    img1 = imread(imgFilepathsLeft[i], IMREAD_COLOR);
    img2 = imread(imgFilepathsRight[i], IMREAD_COLOR);
    cvtColor(img1, gray1, COLOR_BGR2GRAY);
    cvtColor(img2, gray2, COLOR_BGR2GRAY);

    bool found1 = false, found2 = false;

    found1 = cv::findChessboardCorners(img1, board_size, corners1,
    CALIB_CB_ADAPTIVE_THRESH  | CALIB_CB_FILTER_QUADS);
    found2 = cv::findChessboardCorners(img2, board_size, corners2,
    CALIB_CB_ADAPTIVE_THRESH  | CALIB_CB_FILTER_QUADS);

    if(!found1 || !found2){
      cout << "Chessboard find error!" << endl;
      cout << "leftImg: " << imgFilepathsLeft[i] << " and rightImg: " << imgFilepathsRight[i] <<endl;
      continue;
    } 

    if (found1)
    {
      cv::cornerSubPix(gray1, corners1, cv::Size(5, 5), cv::Size(-1, -1),
      cv::TermCriteria(TermCriteria::Type::EPS | TermCriteria::Type::MAX_ITER, 30, 0.1));
      cv::drawChessboardCorners(gray1, board_size, corners1, found1);
    }
    if (found2)
    {
      cv::cornerSubPix(gray2, corners2, cv::Size(5, 5), cv::Size(-1, -1),
      cv::TermCriteria(TermCriteria::Type::EPS | TermCriteria::Type::MAX_ITER, 30, 0.1));
      cv::drawChessboardCorners(gray2, board_size, corners2, found2);
    }

    vector< Point3f > obj;
    for (int i = 0; i < board_height; i++)
      for (int j = 0; j < board_width; j++)
        obj.push_back(Point3f((float)j * square_size, (float)i * square_size, 0));

    if (found1 && found2) {
      cout << i << ". Found corners!" << endl;
      imagePoints1.push_back(corners1);
      imagePoints2.push_back(corners2);
      object_points.push_back(obj);
    }
  }
  for (int i = 0; i < imagePoints1.size(); i++) {
    vector< Point2f > v1, v2;
    for (int j = 0; j < imagePoints1[i].size(); j++) {
      v1.push_back(Point2f((double)imagePoints1[i][j].x, (double)imagePoints1[i][j].y));
      v2.push_back(Point2f((double)imagePoints2[i][j].x, (double)imagePoints2[i][j].y));
    }
    left_img_points.push_back(v1);
    right_img_points.push_back(v2);
  }
}

int main(int argc, char const *argv[])
{
  char* leftcalib_file;
  char* rightcalib_file;
  char* leftimg_dir;
  char* rightimg_dir;
  char* out_file;
  char* extension;

  static struct poptOption options[] = {
    { "leftcalib_file",'u',POPT_ARG_STRING,&leftcalib_file,0,"Left camera calibration","STR" },
    { "rightcalib_file",'v',POPT_ARG_STRING,&rightcalib_file,0,"Right camera calibration","STR" },
    { "leftimg_dir",'L',POPT_ARG_STRING,&leftimg_dir,0,"Directory containing left images","STR" },
    { "rightimg_dir",'R',POPT_ARG_STRING,&rightimg_dir,0,"Directory containing right images","STR" },
    { "extension",'e',POPT_ARG_STRING,&extension,0,"Image extension","STR" },
    { "out_file",'o',POPT_ARG_STRING,&out_file,0,"Output calibration filename (YML)","STR" },
    POPT_AUTOHELP
    { NULL, 0, 0, NULL, 0, NULL, NULL }
  };

  POpt popt(NULL, argc, argv, options, 0);
  int c;
  while((c = popt.getNextOpt()) >= 0) {}

  FileStorage fsl(leftcalib_file, FileStorage::READ);
  FileStorage fsr(rightcalib_file, FileStorage::READ);

  // Load the image filepaths
  vector<string> imgFilepathsLeft, imgFilepathsRight;
  getImgFilepaths(leftimg_dir, extension, imgFilepathsLeft);
  getImgFilepaths(rightimg_dir, extension, imgFilepathsRight);

  load_image_points(fsl["board_width"], fsl["board_height"], fsl["square_size"], imgFilepathsLeft, imgFilepathsRight);

  printf("Starting Calibration\n");
  Mat K1, K2, R, F, E;
  Vec3d T;
  Mat D1, D2;
  fsl["K"] >> K1;
  fsr["K"] >> K2;
  fsl["D"] >> D1;
  fsr["D"] >> D2;
  int flag = 0;
  flag |= CALIB_FIX_INTRINSIC;
  
  cout << "Read intrinsics" << endl;
  
  stereoCalibrate(object_points, left_img_points, right_img_points, K1, D1, K2, D2, img1.size(), R, T, E, F);

  cv::FileStorage fs1(out_file, cv::FileStorage::WRITE);
  fs1 << "K1" << K1;
  fs1 << "K2" << K2;
  fs1 << "D1" << D1;
  fs1 << "D2" << D2;
  fs1 << "R" << R;
  fs1 << "T" << T;
  fs1 << "E" << E;
  fs1 << "F" << F;
  
  printf("Done Calibration\n");

  printf("Starting Rectification\n");

  cv::Mat R1, R2, P1, P2, Q;
  stereoRectify(K1, D1, K2, D2, img1.size(), R, T, R1, R2, P1, P2, Q);

  fs1 << "R1" << R1;
  fs1 << "R2" << R2;
  fs1 << "P1" << P1;
  fs1 << "P2" << P2;
  fs1 << "Q" << Q;

  printf("Done Rectification\n");

  return 0;
}
