cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread")

project(camera_network)

find_package(OpenCV 4.1 REQUIRED)
find_package(PCL 1.2 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})

link_directories(${OpenCV_LIBRARY_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})
add_definitions(${OpenCV_DEFINITIONS})

list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

add_executable (camera_calibration src/rgbdCameraNetwork.cpp src/threeDViewer.cpp src/calibrationHandler.cpp src/sphereTracking.cpp)
target_link_libraries (camera_calibration ${OpenCV_LIBRARIES})
target_link_libraries (camera_calibration ${PCL_LIBRARIES})
