cmake_minimum_required(VERSION 3.0.2)
project(ros_cga_robotics)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
)

catkin_package(
    CATKIN_DEPENDS
        roscpp
)

find_package(cbot REQUIRED)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

add_executable(cbot_fk src/fk/main.cpp)
target_link_libraries(cbot_fk
    ${catkin_LIBRARIES}
    cbot
)
