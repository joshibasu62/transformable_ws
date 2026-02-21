cmake_minimum_required(VERSION 2.8.3)

project(tryd)

find_package(catkin REQUIRED COMPONENTS
controller_manager
genmsg
joint_state_controller
robot_state_publisher
roscpp
gazebo_ros
std_msgs
tf
geometry_msgs
unitree_legged_msgs
)

find_package(gazebo REQUIRED)

catkin_package(
CATKIN_DEPENDS 
unitree_legged_msgs 
)

foreach(dir config launch meshes urdf xacro)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

include_directories(
# include
${Boost_INCLUDE_DIR}
${catkin_INCLUDE_DIRS}
${GAZEBO_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")


