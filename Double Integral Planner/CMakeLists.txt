cmake_minimum_required(VERSION 3.12)
project(double_integral)

set(CMAKE_CXX_STANDARD 11)

find_package(Python3 3.8 EXACT COMPONENTS Interpreter Development)

include_directories(
    INCLUDE_DIRS include
)

add_executable(DIP_test src/main.cpp include/double_integral_planner.cpp)
target_include_directories(DIP_test PRIVATE ${Python3_INCLUDE_DIRS})
target_link_libraries(DIP_test ${Python3_LIBRARIES})