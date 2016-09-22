cmake_minimum_required(VERSION 2.8.3)
project(sarafun_optoforce_calibration)

find_package(catkin REQUIRED COMPONENTS
  ar_track_alvar
  geometry_msgs
  rospy
  std_srvs
  tf
  tf_conversions
  usb_cam
  visualization_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sarafun_optoforce_calibration
#  CATKIN_DEPENDS PyKDL ar_track_alvar geometry_msgs rospy std_srvs tf tf_conversions usb_cam visualization_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

