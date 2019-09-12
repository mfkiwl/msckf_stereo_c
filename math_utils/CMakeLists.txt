cmake_minimum_required(VERSION 3.1)
project(math_utils)

set(CMAKE_CXX_STANDARD 11)

find_package(Boost REQUIRED COMPONENTS system)
if(Boost_FOUND)
    include_directories(${Boost_INCLUDE_DIR})
    link_directories(${Boost_LIBRARY_DIR})
    link_libraries(${Boost_LIBRARIES})
endif()

include_directories(include)

add_library(math_utils SHARED src/random_numbers.cpp)

# Unit Test
if(UnitTest)
    enable_testing()
    find_package(GTest)
    if(GTest_FOUND)
        include_directories(${GTEST_INCLUDE_DIRS})
        add_subdirectory(test)
        add_test(NAME ./test/math_unit_test COMMAND ./test/math_unit_test)
    endif()
endif()