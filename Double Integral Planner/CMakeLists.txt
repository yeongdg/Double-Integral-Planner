cmake_minimum_required(VERSION 3.10.0)
project(double_integral)

set(CMAKE_CXX_STANDARD 11)

find_package(Python3 COMPONENTS Interpreter Development)

add_executable(DI_test src/main.cpp include/doubleIntegral.cpp)
target_include_directories(DI_test PRIVATE ${Python3_INCLUDE_DIRS})
target_link_libraries(DI_test ${Python3_LIBRARIES})