cmake_minimum_required(VERSION 2.8)

project(msckf_vio_mynt)

option(BUILD_TEST "build unit test" OFF)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "-O3 -g")

find_package(OpenCV REQUIRED COMPONENTS core highgui imgproc) # features2d calib3d video
if(OpenCV_FOUND)
    include_directories(${OpenCV_INCLUDE_DIRS})
    link_libraries(${OpenCV_LIBS})
endif()

find_package(Pangolin REQUIRED)
if(Pangolin_FOUND)
    include_directories(${Pangolin_INCLUDE_DIRS})
    link_libraries(${Pangolin_LIBRARIES})
endif()

add_subdirectory(msckf_core)
include_directories(msckf_core/include msckf_core/mynt_basic_algthms/include)

include_directories(include)

add_executable(run_euroc apps/run_euroc.cpp)
target_link_libraries(run_euroc pthread msckf_core)

add_executable(run_euroc_nothread apps/run_euroc_nothread.cpp)
target_link_libraries(run_euroc_nothread pthread msckf_core)
