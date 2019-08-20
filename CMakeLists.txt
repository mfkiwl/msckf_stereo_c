cmake_minimum_required(VERSION 2.8)

project(msckf_vio_mynt)

set(CMAKE_CXX_STANDARD 11)

include_directories(include)

include_directories("/usr/include/eigen3")

find_package(OpenCV REQUIRED)
if(OpenCV_FOUND)
    include_directories(${OpenCV_INCLUDE_DIRS})
    link_libraries(${OpenCV_LIBS})
endif()

find_package(yaml-cpp REQUIRED)
include_directories(${YAML_CPP_INCLUDE_DIR})
link_libraries(${YAML_CPP_LIBRARIES})

add_executable(run_euroc src/apps/run_euroc.cpp)
target_link_libraries(run_euroc ${OpenCV_LIBS} -lpthread)

################ Unit Test ################
enable_testing()
find_package(GTest)
if(GTest_FOUND)
    include_directories(${GTEST_INCLUDE_DIRS})
    add_subdirectory(test)
    add_test(NAME ./test/run_unit_test COMMAND ./test/run_unit_test)
endif()


