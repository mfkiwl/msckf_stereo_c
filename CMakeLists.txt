cmake_minimum_required(VERSION 2.8)

project(msckf_vio_mynt)

set(CMAKE_CXX_STANDARD 11)

include_directories(include)

include_directories("/usr/include/eigen3")

find_package(OpenCV)
if(OpenCV_FOUND)
    include_directories(${OpenCV_INCLUDE_DIRS})
    link_libraries(${OpenCV_LIBS})
endif()

add_executable(run_euroc src/apps/run_euroc.cpp)
target_link_libraries(run_euroc ${OpenCV_LIBS} -lpthread)


