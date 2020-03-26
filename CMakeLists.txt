cmake_minimum_required(VERSION 3.0)
project(CALIB_CAM)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(OpenCV REQUIRED)
include_directories($(OpenCV_INCLUDE_DIRS))

add_executable(calibrate calib_intrinsic.cpp popt_pp.h)
target_link_libraries(calibrate ${OpenCV_LIBS} "-lpopt" stdc++fs)

add_executable(read read_images.cpp)
target_link_libraries(read ${OpenCV_LIBS} "-lpopt" stdc++fs)

add_executable(calibrate_stereo calib_stereo.cpp)
target_link_libraries(calibrate_stereo ${OpenCV_LIBS} "-lpopt" stdc++fs)

add_executable(undistort_rectify undistort_rectify.cpp)
target_link_libraries(undistort_rectify ${OpenCV_LIBS} "-lpopt" stdc++fs)