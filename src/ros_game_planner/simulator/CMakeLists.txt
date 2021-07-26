cmake_minimum_required(VERSION 3.0.2)
project(simulator)

add_compile_options(-std=c++14 -Wall)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  tf2
  tf2_ros
  geometry
)

find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GTest REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)
link_libraries(${JSONCPP_LIBRARIES})

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
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${GTEST_INCLUDE_DIRS}
  ${JSONCPP_INCLUDE_DIRS}
)

add_executable(simulator src/lane/lane.cpp)
target_link_libraries(simulator ${catkin_LIBRARIES} ${JSONCPP_LIBRARIES})
add_dependencies(simulator ${catkin_EXPORTED_TARGETS})

install(
        TARGETS
        simulator
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)