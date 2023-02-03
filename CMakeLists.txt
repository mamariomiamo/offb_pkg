cmake_minimum_required(VERSION 3.0.2)
project(px4_offb)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  roscpp
  std_msgs
  tf2
  tf2_ros
  quadrotor_msgs
  eigen_conversions
  traj_utils
)

add_subdirectory(offboard_controller_lib)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES px4_offb
  CATKIN_DEPENDS geometry_msgs mavros_msgs roscpp std_msgs tf2 tf2_ros quadrotor_msgs traj_utils
#  DEPENDS system_lib
  DEPENDS Eigen3
)

include_directories(
 include
 include/px4_offb
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node 
  src/main.cpp 
  src/offb_node.cpp 
  src/px4_tf2.cpp)

add_executable(downwash
  src/downwash.cpp 
)

add_executable(px4_tf2_standalone
  src/main_tf2.cpp
  src/px4_tf2.cpp)

add_executable(traj_gen
  src/traj_gen.cpp)

## Specify libraries to link a library or executable target against
 target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
   offboard_controller_lib
 )

 target_link_libraries(downwash
 ${catkin_LIBRARIES}
)

target_link_libraries(px4_tf2_standalone
${catkin_LIBRARIES}
)

target_link_libraries(traj_gen
${catkin_LIBRARIES}
)
