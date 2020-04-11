cmake_minimum_required(VERSION 3.0)
project(STEREO_CALIBRATION)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

# Find dependencies
find_package(OpenCV REQUIRED)
include_directories($(OpenCV_INCLUDE_DIRS))

option(BUILD_EXPORT "Build export" OFF)

add_subdirectory(src)