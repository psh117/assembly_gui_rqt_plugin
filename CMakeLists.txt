cmake_minimum_required(VERSION 3.0.2)
project(assembly_gui_rqt_plugin)

find_package(catkin REQUIRED
  rospy
  rqt_gui_py
)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES aas
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)
include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  scripts/assembly_gui_rqt_plugin
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
  resource
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES
  plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)