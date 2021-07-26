cmake_minimum_required(VERSION 3.0.2)
project(geometry)

add_compile_options(-std=c++14 -Wall)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

find_package(Eigen3 REQUIRED)
find_package(GTest REQUIRED)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES geometry
        CATKIN_DEPENDS
        roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GTEST_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(geometry src/geometry/line_segment2.cpp
                     src/geometry/polyline2.cpp)
target_link_libraries(geometry ${catkin_LIBRARIES})

install(
        TARGETS
        geometry
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

