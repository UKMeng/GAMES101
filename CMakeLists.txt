cmake_minimum_required(VERSION 3.10)
project(GAMES101)

set(OpenCV_DIR ./3rdParty/opencv)
find_package(OpenCV REQUIRED)

find_package(Eigen3 REQUIRED)

set(CMAKE_CXX_STANDARD 17)

include_directories(/usr/local/include)
include_directories(3rdParty)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})

add_subdirectory(Assignment7)

## Assignment 8

if(WIN32)
    if(MSVC)
        set(MSVC_CXX_FLAGS "-std=gnu++11")

        if(BUILD_DEBUG)
            set(CMAKE_BUILD_TYPE Debug)
        else(BUILD_DEBUG)
        endif(BUILD_DEBUG)

        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${MSVC_CXX_FLAGS}")

    endif(MSVC)

    if(MINGW)

        set(MSVC_CXX_FLAGS "-std=gnu++11")

        if(BUILD_DEBUG)
            set(CMAKE_BUILD_TYPE Debug)
        else(BUILD_DEBUG)
        endif(BUILD_DEBUG)

        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${MSVC_CXX_FLAGS}")

    endif(MINGW)

endif(WIN32)

find_package(OpenGL REQUIRED)
find_package(Threads REQUIRED)

set(FREETYPE_LIBRARY ${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/freetype/bin/freetype.lib)
set(FREETYPE_INCLUDE_DIRS ./3rdParty/freetype/include)
find_package(Freetype REQUIRED)
include_directories(${FREETYPE_INCLUDE_DIRS})

add_subdirectory(./3rdParty/CGL)
include_directories(./3rdParty/CGL/include)

#option(BUILD_LIBCGL "Build with libCGL" ON)
#if(BUILD_LIBCGL)
#    add_subdirectory(./3rdParty/CGL)
#    include_directories(./3rdParty/CGL/include)
#else(BUILD_LIBCGL)
#    find_package(CGL REQUIRED)
#    find_package(GLEW REQUIRED)
#    find_package(GLFW REQUIRED)
#endif(BUILD_LIBCGL)

add_subdirectory(Assignment8)
set(CMAKE_INSTALL_PREFIX "${RopeSim_SOURCE_DIR}/")