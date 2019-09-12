cmake_minimum_required(VERSION 2.8)

project(msckf_vio_mynt)

option(UnitTest "unit test" FALSE)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "-O3 -g")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake_modules")

find_package(Cholmod)
if(CHOLMOD_FOUND)
    add_definitions("-DEIGEN_CHOLMOD_SUPPORT")
    include_directories(${CHOLMOD_INCLUDES})
    set(CHOLMOD_ALL_LIBS  ${CHOLMOD_LIBRARIES} ${EIGEN_BLAS_LIBRARIES} ${EIGEN_LAPACK_LIBRARIES})
    link_libraries(spqr ${LAPACK_LIBRARIES} ${CHOLMOD_ALL_LIBS})
endif()

find_package(Boost REQUIRED COMPONENTS system)
if(Boost_FOUND)
    include_directories(${Boost_INCLUDE_DIR})
    link_directories(${Boost_LIBRARY_DIR})
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
endif()

find_package(Pangolin REQUIRED)
if(Pangolin_FOUND)
    include_directories(${Pangolin_INCLUDE_DIRS})
    link_libraries(${Pangolin_LIBRARIES})
endif()

add_subdirectory(math_utils)

include_directories(include math_utils/include /usr/include/suitesparse /usr/include/eigen3)

set(SRC_LIST src/image_processor.cpp src/msckf_vio.cpp src/system.cpp)
set(LIB_LIST math_utils pthread)

add_executable(run_euroc src/apps/run_euroc.cpp ${SRC_LIST})
target_link_libraries(run_euroc ${LIB_LIST})

add_executable(run_euroc_nothread src/apps/run_euroc_nothread.cpp ${SRC_LIST})
target_link_libraries(run_euroc_nothread ${LIB_LIST})

# Unit Test
if(UnitTest)
    enable_testing()
    find_package(GTest)
    if(GTest_FOUND)
        include_directories(${GTEST_INCLUDE_DIRS})
        add_subdirectory(test)
        add_test(NAME ./test/run_unit_test COMMAND ./test/run_unit_test)
    endif()
endif()


