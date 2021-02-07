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

function(add_robot_node robot node)
    add_executable(${robot}_${node}
        src/${robot}/${node}.cpp
    )
    target_link_libraries(${robot}_${node} ${catkin_LIBRARIES} cbot)
endfunction()

add_robot_node(delta fk)
add_robot_node(delta ik)
add_robot_node(delta fk_vel)
add_robot_node(delta ik_vel)

add_executable(joint_velocity_integrator src/joint_velocity_integrator.cpp)
target_link_libraries(joint_velocity_integrator
    ${catkin_LIBRARIES}
)

add_executable(twist_marker src/twist_marker.cpp)
target_link_libraries(twist_marker
    ${catkin_LIBRARIES}
)
target_include_directories(twist_marker
    PRIVATE ${EIGEN3_INCLUDE_DIRS}
)
