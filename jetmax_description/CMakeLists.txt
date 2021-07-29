cmake_minimum_required(VERSION 2.8.3)

project(jetmax_description)

catkin_package()

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rviz
  roscpp
  rospy
  std_msgs
  tf
  urdf
  xacro
)

foreach(dir config launch meshes urdf)
install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir config launch meshes urdf)
