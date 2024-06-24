cmake_minimum_required(VERSION 3.10)
project(Rasterizer)

set(OpenCV_DIR ./3rdParty/opencv)
find_package(OpenCV REQUIRED)

add_subdirectory(./3rdParty/eigen3)

set(CMAKE_CXX_STANDARD 17)

include_directories(/usr/local/include)
include_directories(3rdParty)
include_directories(${OpenCV_INCLUDE_DIRS})

add_subdirectory(Assignment1)

