cmake_minimum_required(VERSION 3.0.2)
project(open_simulator)

add_compile_options(-std=c++14 -Wall)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  tf2
  tf2_ros
  geometry
  game_planner
)

find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GTest REQUIRED)

catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS
        roscpp
        rospy
        std_msgs
        geometry_msgs
        tf2
        tf2_ros
        geometry
        game_planner
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        ${GTEST_INCLUDE_DIRS}
)

add_library(open_simulator_lib src/utils/ros_utils.cpp)
add_dependencies(open_simulator_lib ${catkin_EXPORTED_TARGETS})
target_link_libraries(open_simulator_lib ${catkin_LIBRARIES})

add_executable(compare_node src/compare_game_planner_node.cpp src/core/compare_game_planner_core.cpp)
add_dependencies(compare_node${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(compare_node ${catkin_LIBRARIES} open_simulator_lib)

add_executable(game_planner_node src/game_planner_node.cpp src/core/game_planner_core.cpp)
add_dependencies(game_planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(game_planner_node ${catkin_LIBRARIES} open_simulator_lib)

#######################################
############## INSTALL ################
#######################################

install(
        TARGETS
        compare_node
        game_planner_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
        DIRECTORY
        launch
        rviz
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)




