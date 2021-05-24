cmake_minimum_required(VERSION 3.0.2)
project(cga_robotics_ros)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    sensor_msgs
    actionlib
    actionlib_msgs
    message_generation
)

find_package(cbot REQUIRED)
find_package(Eigen3 REQUIRED)

add_action_files(
    DIRECTORY action
    FILES
        Trajectory.action
)
generate_messages(DEPENDENCIES
    std_msgs geometry_msgs actionlib_msgs
)

catkin_package(
    CATKIN_DEPENDS
        roscpp
        std_msgs
        geometry_msgs
        sensor_msgs
    DEPENDS
        EIGEN3
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    include
)

# DELTA

add_executable(delta_controller
    src/delta/controller.cpp
    src/joint_group_publisher.cpp
    src/nodes/controller.cpp
)
target_link_libraries(delta_controller ${catkin_LIBRARIES} cbot)
add_dependencies(delta_controller ${cga_robotics_ros_EXPORTED_TARGETS})

add_executable(delta_state_publisher
    src/delta/state_publisher.cpp
    src/nodes/state_publisher.cpp
)
target_link_libraries(delta_state_publisher ${catkin_LIBRARIES} cbot)

# SERIAL

add_executable(serial_controller
    src/serial/controller.cpp
    src/joint_group_publisher.cpp
    src/nodes/controller.cpp
)
target_link_libraries(serial_controller ${catkin_LIBRARIES} cbot)
add_dependencies(serial_controller ${cga_robotics_ros_EXPORTED_TARGETS})

add_executable(serial_state_publisher
    src/serial/state_publisher.cpp
    src/nodes/state_publisher.cpp
)
target_link_libraries(serial_state_publisher ${catkin_LIBRARIES} cbot)

# GENERAL

add_executable(twist_marker src/twist_marker.cpp)
target_link_libraries(twist_marker
    ${catkin_LIBRARIES}
)
target_include_directories(twist_marker
    PRIVATE ${EIGEN3_INCLUDE_DIRS}
)

add_executable(commander
    src/commander.cpp
    src/joystick_listener.cpp
)
target_link_libraries(commander
    ${catkin_LIBRARIES}
)
