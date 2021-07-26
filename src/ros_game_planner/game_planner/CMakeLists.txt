cmake_minimum_required(VERSION 3.0.2)
project(game_planner)

add_compile_options(-std=c++14 -Wall)

add_definitions(-DILQGAMES_LOG_DIR="${CMAKE_SOURCE_DIR}/game_planner/logs")

find_package(catkin REQUIRED COMPONENTS
             roscpp
             rospy
             std_msgs
             geometry_msgs
             tf2
             tf2_ros
             geometry
)

find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GTest REQUIRED)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES game_planner
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
)

add_library(game_planner src/dynamics/single_player_unicycle_4d.cpp
                         src/dynamics/single_player_car_6d.cpp
                         src/dynamics/multi_player_dynamical_system.cpp
                         src/cost/deterministic/quadratic_cost.cpp
                         src/cost/deterministic/quadratic_polyline2_cost.cpp
                         src/cost/deterministic/proximity_cost.cpp
                         src/cost/deterministic/semiquadratic_polyline2_cost.cpp
                         src/cost/deterministic/obstacle_cost.cpp
                         src/cost/deterministic/obstacle_linear_cost.cpp
                         src/cost/stochastic/chance_obstacle_linear_cost.cpp
                         src/cost/player_cost.cpp
                         src/constraint/base_al_constraint.cpp
                         src/constraint/deterministic/single_dimension_constraint.cpp
                         src/constraint/deterministic/proximity_constraint.cpp
                         src/constraint/deterministic/obstacle_constraint.cpp
                         src/constraint/deterministic/obstacle_linear_constraint.cpp
                         src/constraint/stochastic/single_dimension_chance_constraint.cpp
                         src/constraint/stochastic/obstacle_linear_chance_constraint.cpp
                         src/utils/operating_point.cpp
                         src/utils/loop_timer.cpp
                         src/utils/math_utils.cpp
                         src/solver/lq_solver/lq_feedback_nash_solver.cpp
                         src/solver/lq_solver/lq_feedback_stackelberg_solver.cpp
                         src/solver/iterative_solver/base_iterative_solver.cpp
                         src/solver/iterative_solver/ilq_solver.cpp
                         src/solver/iterative_solver/ilqg_solver.cpp
                         src/solver/problem.cpp
                         src/solver/solver_log.cpp
                         src/solver/augmented_lagrangian_solver.cpp
                         )
target_link_libraries(game_planner ${catkin_LIBRARIES})
add_dependencies(game_planner ${catkin_EXPORTED_TARGETS})

#####################################
############## TEST #################
#####################################

catkin_add_gtest(single_player_unicycle4d_test test/dynamics/single_player_unicycle4d_test.cpp)
target_link_libraries(single_player_unicycle4d_test ${catkin_LIBRARIES} game_planner)

catkin_add_gtest(multi_player_dynamical_system_test test/dynamics/multi_player_dynamical_system_test.cpp)
target_link_libraries(multi_player_dynamical_system_test ${catkin_LIBRARIES} game_planner)

catkin_add_gtest(quadratic_cost_test test/cost/deterministic/quadratic_cost_test.cpp)
target_link_libraries(quadratic_cost_test ${catkin_LIBRARIES} game_planner)

catkin_add_gtest(obstacle_cost_test test/cost/deterministic/obstacle_cost_test.cpp)
target_link_libraries(obstacle_cost_test ${catkin_LIBRARIES} game_planner)

catkin_add_gtest(obstacle_linear_cost_test test/cost/deterministic/obstacle_linear_cost_test.cpp)
target_link_libraries(obstacle_linear_cost_test ${catkin_LIBRARIES} game_planner)

catkin_add_gtest(chance_obstacle_linear_cost_test test/cost/stochastic/chance_obstacle_linear_cost_test.cpp)
target_link_libraries(chance_obstacle_linear_cost_test ${catkin_LIBRARIES} game_planner)

catkin_add_gtest(player_cost_test test/cost/deterministic/player_cost_test.cpp)
target_link_libraries(player_cost_test ${catkin_LIBRARIES} game_planner)

catkin_add_gtest(single_dimension_constraint_test test/constraint/deterministic/single_dimension_constraint_test.cpp)
target_link_libraries(single_dimension_constraint_test ${catkin_LIBRARIES} game_planner)

catkin_add_gtest(obstacle_constraint_test test/constraint/deterministic/obstacle_constraint_test.cpp)
target_link_libraries(obstacle_constraint_test ${catkin_LIBRARIES} game_planner)

catkin_add_gtest(obstacle_linear_constraint_test test/constraint/deterministic/obstacle_linear_constraint_test.cpp)
target_link_libraries(obstacle_linear_constraint_test ${catkin_LIBRARIES} game_planner)

catkin_add_gtest(single_dimension_chance_constraint_test test/constraint/stochastic/single_dimension_chance_constraint_test.cpp)
target_link_libraries(single_dimension_chance_constraint_test ${catkin_LIBRARIES} game_planner)

catkin_add_gtest(chance_player_cost_test test/cost/stochastic/chance_player_cost_test.cpp)
target_link_libraries(chance_player_cost_test ${catkin_LIBRARIES} game_planner)

catkin_add_gtest(lq_feedback_nash_solver_test test/solver/lq_solver/lq_feedback_nash_solver_test.cpp)
target_link_libraries(lq_feedback_nash_solver_test ${catkin_LIBRARIES} game_planner)

catkin_add_gtest(ilq_solver_test test/solver/ilq_solver_test.cpp)
target_link_libraries(ilq_solver_test ${catkin_LIBRARIES} game_planner)

catkin_add_gtest(ilqg_solver_test test/solver/ilqg_solver_test.cpp)
target_link_libraries(ilqg_solver_test ${catkin_LIBRARIES} game_planner)

#######################################
############## INSTALL ################
#######################################

install(
        TARGETS
        game_planner
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        )

install(
        DIRECTORY
        launch
        rviz
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)






