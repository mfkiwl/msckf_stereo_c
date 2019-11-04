include_directories(${PROJECT_SOURCE_DIR}/include)

add_executable(run_unit_test
        gtest_main.cpp
        test_yaml.cpp)

target_link_libraries(run_unit_test gtest pthread)