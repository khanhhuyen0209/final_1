cmake_minimum_required(VERSION 2.8.3)
project(six)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  move_base_msgs
  actionlib
  geometry_msgs
  nav_msgs
  tf
)

catkin_package(
  CATKIN_DEPENDS
  rospy
  std_msgs
  move_base_msgs
  actionlib
  geometry_msgs
  nav_msgs
  tf
)

catkin_install_python(PROGRAMS
  scripts/frontier_explorer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(FILES
  worlds/testmap.world
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/worlds
)

foreach(dir config launch meshes urdf)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

