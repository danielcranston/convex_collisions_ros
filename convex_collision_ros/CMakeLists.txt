cmake_minimum_required(VERSION 2.8)
project(convex_collision_ros)

find_package(
  catkin REQUIRED
  COMPONENTS
    geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
)

# INSTALL####
catkin_install_python(
  PROGRAMS scripts/collision_manager.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)

# if(CATKIN_ENABLE_TESTING)
#   find_package(rostest REQUIRED)
#   add_rostest(test/test_sorting_execution_node.test)
# endif()