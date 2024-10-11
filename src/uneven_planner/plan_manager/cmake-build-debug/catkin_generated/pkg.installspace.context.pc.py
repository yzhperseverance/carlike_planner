# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "roscpp;std_msgs;uneven_map;visualization_msgs;front_end;back_end;mpc_controller".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lplan_manager".split(';') if "-lplan_manager" != "" else []
PROJECT_NAME = "plan_manager"
PROJECT_SPACE_DIR = "/usr/local"
PROJECT_VERSION = "0.0.1"
