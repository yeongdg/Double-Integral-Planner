cmake_minimum_required(VERSION 3.0.2)
project(yaml_read_practice)

find_package(yaml-cpp REQUIRED)

set(CMAKE_CXX_STANDARD 11)

find_package(Python3 3.8 EXACT COMPONENTS Interpreter Development)


include_directories(
    INCLUDE_DIRS include
    ${YAML_CPP_INCLUDE_DIRS}
)


add_executable(yaml_read_node include/yaml_read.cpp src/yaml_read_node.cpp)
target_link_libraries(yaml_read_node ${YAML_CPP_LIBRARIES})

add_executable(single_dip_node include/double_integral_planner.cpp src/single_dip_node.cpp)
target_include_directories(single_dip_node PRIVATE ${Python3_INCLUDE_DIRS})
target_link_libraries(single_dip_node ${Python3_LIBRARIES})


add_executable(multi_dip_node   include/traj_gen.cpp
                                include/double_integral_planner.cpp 
                                include/yaml_read.cpp
                                src/multi_dip_node.cpp)
                              
target_include_directories(multi_dip_node PRIVATE ${Python3_INCLUDE_DIRS})

target_link_libraries(multi_dip_node ${Python3_LIBRARIES} 
                                    ${YAML_CPP_LIBRARIES})


add_executable(wheel_speed_node include/wheel_speed_mode.cpp
                                include/traj_gen.cpp
                                include/double_integral_planner.cpp 
                                include/yaml_read.cpp
                                src/wheel_speed_node.cpp)
                                  
    target_include_directories(wheel_speed_node PRIVATE ${Python3_INCLUDE_DIRS})
    
    target_link_libraries(wheel_speed_node  ${Python3_LIBRARIES} 
                                            ${YAML_CPP_LIBRARIES})
