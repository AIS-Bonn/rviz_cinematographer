cmake_minimum_required(VERSION 2.8)
project(rviz_cinematographer_msgs)

set(CMAKE_BUILD_TYPE RelWithDebInfo)

set(MSG_DEPS
   std_msgs
   geometry_msgs
)

find_package(catkin REQUIRED COMPONENTS
   ${MSG_DEPS}
	message_generation
	genmsg
)

add_message_files(
   DIRECTORY msg 
	 FILES
   CameraMovement.msg
   CameraTrajectory.msg
   Record.msg
   Finished.msg
   Wait.msg
)

generate_messages(
	DEPENDENCIES
   ${MSG_DEPS}
)

catkin_package(
	CATKIN_DEPENDS message_runtime ${MSG_DEPS}
)

include_directories(
	${catkin_INCLUDE_DIRS}
)
