# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "roscpp;std_msgs;uneven_map;visualization_msgs;front_end".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lback_end".split(';') if "-lback_end" != "" else []
PROJECT_NAME = "back_end"
PROJECT_SPACE_DIR = "/usr/local"
PROJECT_VERSION = "0.0.1"
