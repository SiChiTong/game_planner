# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "roscpp;rospy;std_msgs;geometry_msgs;tf2;tf2_ros;geometry".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lgame_planner".split(';') if "-lgame_planner" != "" else []
PROJECT_NAME = "game_planner"
PROJECT_SPACE_DIR = "/home/hai/game_planner_ws/install"
PROJECT_VERSION = "0.1.0"