cmake_minimum_required(VERSION 2.8)

project(msckf_vio_mynt)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "-O3 -g")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake_modules")

find_package(Cholmod)
if(CHOLMOD_FOUND)
    add_definitions("-DEIGEN_CHOLMOD_SUPPORT")
    include_directories(${CHOLMOD_INCLUDES})
    set(CHOLMOD_ALL_LIBS  ${CHOLMOD_LIBRARIES} ${EIGEN_BLAS_LIBRARIES} ${EIGEN_LAPACK_LIBRARIES})
endif()

#find_package(SPQR)
#if(SPQR_FOUND AND CHOLMOD_FOUND AND (EIGEN_Fortran_COMPILER_WORKS OR LAPACK_FOUND) )
#    add_definitions("-DEIGEN_SPQR_SUPPORT")
#    include_directories(${SPQR_INCLUDES})
#    set(SPARSE_LIBS ${SPARSE_LIBS} ${SPQR_LIBRARIES} ${LAPACK_LIBRARIES} ${CHOLMOD_ALL_LIBS})
#    message("===INCLUDE: " ${SPQR_INCLUDES})
#else()
#    message("=== SPQR NOT FOUND")
#endif()
include_directories(/usr/include/suitesparse)

include_directories(/usr/include/eigen3)

include_directories(include)

find_package(Boost REQUIRED COMPONENTS system)
include_directories(${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIR})

find_package(OpenCV REQUIRED)
if(OpenCV_FOUND)
    include_directories(${OpenCV_INCLUDE_DIRS})
    link_libraries(${OpenCV_LIBS})
endif()

find_package(yaml-cpp REQUIRED)
include_directories(${YAML_CPP_INCLUDE_DIR})
link_libraries(${YAML_CPP_LIBRARIES})

find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})
link_libraries(${Pangolin_LIBRARIES})

set(SRC_LIST src/random_numbers.cpp src/image_processor.cpp src/msckf_vio.cpp src/system.cpp)
set(LIB_LIST ${OpenCV_LIBS} ${Boost_LIBRARIES} -lpthread ${LAPACK_LIBRARIES} ${CHOLMOD_ALL_LIBS} -lspqr)

add_executable(run_euroc src/apps/run_euroc.cpp ${SRC_LIST})
target_link_libraries(run_euroc ${LIB_LIST})

add_executable(run_euroc_nothread src/apps/run_euroc_nothread.cpp ${SRC_LIST})
target_link_libraries(run_euroc_nothread ${LIB_LIST})

# Unit Test
enable_testing()
find_package(GTest)
if(GTest_FOUND)
    include_directories(${GTEST_INCLUDE_DIRS})
    add_subdirectory(test)
    add_test(NAME ./test/run_unit_test COMMAND ./test/run_unit_test)
endif()


