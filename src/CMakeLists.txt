add_executable(singleCamCalib singleCamCalib.cpp utils.h)
add_executable(stereoCamCalib stereoCamCalib.cpp utils.h)
add_executable(stereoRectify stereoRectify.cpp utils.h)
add_executable(undistortRectify undistortRectify.cpp utils.h)
list(APPEND EXECUTABLES singleCamCalib stereoCamCalib stereoRectify undistortRectify)

foreach(EXECUTABLE IN LISTS EXECUTABLES)
    target_link_libraries(${EXECUTABLE} ${OpenCV_LIBS} stdc++fs)
endforeach()

