cmake_minimum_required(VERSION 2.8)

project(msckf_vio_mynt)

option(BUILD_TEST "build unit test" OFF)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "-O3 -g")

find_package(OpenCV REQUIRED COMPONENTS core highgui) # imgproc features2d calib3d video
if(OpenCV_FOUND)
    include_directories(${OpenCV_INCLUDE_DIRS})
endif()

find_package(Pangolin REQUIRED)
if(Pangolin_FOUND)
    include_directories(${Pangolin_INCLUDE_DIRS})
endif()

add_subdirectory(msckf_core)

include_directories(./ include msckf_core/include msckf_core/mynt_basic_algthms/include)

add_executable(run_euroc_muitl_threads apps/run_euroc_muitl_threads.cpp)
target_link_libraries(run_euroc_muitl_threads pthread msckf_core ${OpenCV_LIBS} ${Pangolin_LIBRARIES})

add_executable(run_euroc_single_thread apps/run_euroc_single_thread.cpp glwindow/glwindow_x11.cpp glwindow/scenewindow.cpp)
target_link_libraries(run_euroc_single_thread pthread msckf_core ${OpenCV_LIBS} ${Pangolin_LIBRARIES})
