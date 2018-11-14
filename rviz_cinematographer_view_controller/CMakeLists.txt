cmake_minimum_required(VERSION 2.8.3)
project(rviz_cinematographer_view_controller)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)
find_package(Boost REQUIRED system filesystem)

# Ogre
pkg_check_modules(OGRE OGRE)

add_definitions(-DQT_NO_KEYWORDS)

find_package(OpenGL REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  rviz
  pluginlib
  nav_msgs
  rviz_cinematographer_msgs
   cv_bridge
   image_geometry
   image_transport
)

# Qt Stuff
find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)

set(QT_INCLUDE_DIRS 
  ${Qt5Core_INCLUDE_DIRS} 
  ${Qt5Widgets_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS rviz_cinematographer_msgs)

qt5_wrap_cpp(MOC_FILES
  include/${PROJECT_NAME}/rviz_cinematographer_view_controller.h
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${QT_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}
  ${OPENGL_INCLUDE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR})

add_library(${PROJECT_NAME}
        src/rviz_cinematographer_view_controller.cpp
  ${MOC_FILES}
)

target_link_libraries(
  ${PROJECT_NAME} 
  ${OGRE_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_dependencies(
  ${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION} PATTERN ".svn" EXCLUDE)

install(FILES plugin_description.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Dummy target for IDE's
FILE(GLOB_RECURSE all_headers_for_ides
        ${CMAKE_SOURCE_DIR}/**/*.hpp
        ${CMAKE_SOURCE_DIR}/**/*.h)
ADD_CUSTOM_TARGET(ide_includes_${PROJECT_NAME} SOURCES ${all_headers_for_ides})
