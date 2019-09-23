cmake_minimum_required(VERSION 2.8)

project(msckf_vio_mynt)

option(BUILD_TEST "build unit test" OFF)

set(N_CORES 3 CACHE STRING "Using N number of cores for parallel build")

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "-O3 -g")

set(CMAKE_MODULE_PATH APPEND "${CMAKE_SOURCE_DIR}/cmake_modules")

find_package(SuiteSparse REQUIRED)
if(SuiteSparse_FOUND)
    include_directories(${SUITESPARSE_INCLUDE_DIRS})
endif()

find_package(Cholmod)
if(CHOLMOD_FOUND)
    add_definitions("-DEIGEN_CHOLMOD_SUPPORT")
    include_directories(${CHOLMOD_INCLUDES})
    set(CHOLMOD_ALL_LIBS  ${CHOLMOD_LIBRARIES} ${EIGEN_BLAS_LIBRARIES} ${EIGEN_LAPACK_LIBRARIES})
    link_libraries(spqr ${LAPACK_LIBRARIES} ${CHOLMOD_ALL_LIBS})
endif()

find_package(Eigen3 REQUIRED)
if(Eigen3_FOUND)
    include_directories(${EIGEN3_INCLUDE_DIR})
endif()

find_package(OpenCV REQUIRED COMPONENTS core highgui imgproc features2d calib3d video)
if(OpenCV_FOUND)
    include_directories(${OpenCV_INCLUDE_DIRS})
    link_libraries(${OpenCV_LIBS})
endif()

find_package(yaml-cpp REQUIRED)
if(yaml-cpp_FOUND)
    include_directories(${YAML_CPP_INCLUDE_DIR})
    link_libraries(${YAML_CPP_LIBRARIES})
    message("   yaml-cpp Version: " ${yaml-cpp_VERSION})
endif()

find_package(Pangolin REQUIRED)
if(Pangolin_FOUND)
    include_directories(${Pangolin_INCLUDE_DIRS})
    link_libraries(${Pangolin_LIBRARIES})
endif()

# mynt_basic_algthms
#include(ExternalProject)
#ExternalProject_Add(mynt_basic_algthms_external
#        GIT_REPOSITORY http://gitlab.mynt.com/mynt-common-algorithms/mynt_basic_algthms.git
#        GIT_TAG master
#        UPDATE_COMMAND ""
#        INSTALL_DIR ${CMAKE_BINARY_DIR}
#        CMAKE_ARGS
#            -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
#            -DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
#            -DCMAKE_CXX_FLAGS=${CMAKE_CXX_FLAGS}
#        PREFIX ${CMAKE_CURRENT_BINARY_DIR}/mynt_basic_algthms
#        BUILD_IN_SOURCE 0
#        BUILD_COMMAND make -j${N_CORES}
#        INSTALL_COMMAND make install
#)
#ExternalProject_Get_Property(mynt_basic_algthms_external INSTALL_DIR)
#include_directories(${INSTALL_DIR}/include)
#link_directories(${INSTALL_DIR}/lib)
#add_library(mynt_maths SHARED IMPORTED)
#set(MYNT_BASIC_ALGTHMS_LIBRARIES mynt_maths)
#add_dependencies(${MYNT_BASIC_ALGTHMS_LIBRARIES} mynt_basic_algthms_external)

add_subdirectory(mynt_basic_algthms)
include_directories(mynt_basic_algthms/include)
link_libraries(mynt_maths)

include_directories(include)

set(SRC_LIST src/image_processor.cpp src/msckf_vio.cpp src/system.cpp)
set(LIB_LIST pthread)

add_executable(run_euroc src/apps/run_euroc.cpp ${SRC_LIST})
target_link_libraries(run_euroc ${LIB_LIST})

add_executable(run_euroc_nothread src/apps/run_euroc_nothread.cpp ${SRC_LIST})
target_link_libraries(run_euroc_nothread ${LIB_LIST})

# unit test
if(BUILD_TEST)
    enable_testing()
    find_package(GTest)
    if(GTest_FOUND)
        include_directories(${GTEST_INCLUDE_DIRS})
        add_subdirectory(test)
    endif()
endif()


