cmake_minimum_required(VERSION 3.10)
project(Rasterizer)

set(OpenCV_DIR ./3rdParty/opencv)
find_package(OpenCV REQUIRED)

find_package(Eigen3 REQUIRED)

set(CMAKE_CXX_STANDARD 17)

include_directories(/usr/local/include)
include_directories(3rdParty)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})

add_subdirectory(Assignment3)

