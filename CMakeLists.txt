cmake_minimum_required(VERSION 3.0.2)
project(cga_robotics_ros)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
)

find_package(cbot REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
    CATKIN_DEPENDS
        roscpp
    DEPENDS
        EIGEN3
)

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

add_executable(delta_fk_vel src/delta/fk_vel.cpp)
target_link_libraries(delta_fk_vel
    ${catkin_LIBRARIES}
    cbot
)

add_executable(twist_marker src/twist_marker.cpp)
target_link_libraries(twist_marker
    ${catkin_LIBRARIES}
)
target_include_directories(twist_marker
    PRIVATE ${EIGEN3_INCLUDE_DIRS}
)
