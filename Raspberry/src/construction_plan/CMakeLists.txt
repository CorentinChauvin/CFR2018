cmake_minimum_required(VERSION 2.8.3)
project(construction_plan)

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++11)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
#catkin_python_setup()

catkin_package(
	CATKIN_DEPENDS
		cv_bridge
		roscpp
		rospy
		sensor_msgs
)

# FIND PACKAGES
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  rospy
  sensor_msgs
  roslib
  
)
find_package(PythonLibs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PythonLibs REQUIRED)

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${PYTHON_INCLUDE_DIRS})

# BUILD
add_executable(construction_plan_node src/construction_plan_node.cpp)

target_link_libraries(construction_plan_node
	${catkin_LIBRARIES}
	${OpenCV_LIBS}
	${PYTHON_LIBRARIES}
)
