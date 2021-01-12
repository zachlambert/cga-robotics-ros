cmake_minimum_required(VERSION 3.0.2)
project(cga_robotics_ros)
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
    include
)

add_executable(delta_fk src/delta/fk.cpp)
target_link_libraries(delta_fk
    ${catkin_LIBRARIES}
    cbot
)

add_executable(delta_ik src/delta/ik.cpp)
target_link_libraries(delta_ik
    ${catkin_LIBRARIES}
    cbot
)
