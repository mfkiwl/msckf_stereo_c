include_directories(${PROJECT_SOURCE_DIR}/include)

include_directories(/usr/include/eigen3)

add_executable(math_unit_test
        gtest_main.cpp
        test_math.cpp)

target_link_libraries(math_unit_test gtest pthread math_utils)