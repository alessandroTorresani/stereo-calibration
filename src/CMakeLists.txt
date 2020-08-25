add_executable(singleCamCalib singleCamCalib.cpp utils.h)
add_executable(stereoCamCalib stereoCamCalib.cpp utils.h)
add_executable(stereoRectify stereoRectify.cpp utils.h)
add_executable(checkRectification evaluation/checkRectification.cpp utils.h)
list(APPEND EXECUTABLES singleCamCalib stereoCamCalib stereoRectify checkRectification)

if(BUILD_EXPORT)
    add_executable(exportOpenvslamMono export/openvslamMono.cpp)
    add_executable(exportOpenvslamStereo export/openvslamStereo.cpp)
    list(APPEND EXECUTABLES exportOpenvslamMono exportOpenvslamStereo)
endif()

foreach(EXECUTABLE IN LISTS EXECUTABLES)
    target_include_directories(${EXECUTABLE} PRIVATE ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(${EXECUTABLE} ${OpenCV_LIBS} stdc++fs)
endforeach()

