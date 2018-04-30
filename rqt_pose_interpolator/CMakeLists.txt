cmake_minimum_required(VERSION 2.8.3)
project(rqt_pose_interpolator)

set(CMAKE_BUILD_TYPE RelWithDebInfo)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  interactive_markers
  nav_msgs
  roscpp
  rqt_gui
  rqt_gui_cpp
  tf
  view_controller_msgs
  rviz_animated_view_controller
)

# Decide on a Qt version
if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
    set(USE_QT5 true)
else()
    set(USE_QT5 false)
endif()

# Find Qt
if(USE_QT5)
    find_package(Qt5Widgets REQUIRED)
    set(QT_LIBRARIES ${Qt5Widgets_LIBRARIES})
else()
    find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
    include(${QT_USE_FILE})
endif()

include_directories(${CMAKE_CURRENT_BINARY_DIR})

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS interactive_markers nav_msgs roscpp rqt_gui rqt_gui_cpp tf view_controller_msgs rviz_animated_view_controller
)

###########
## Build ##
###########

include_directories(
    include
    ${CATKIN_DEVEL_PREFIX}/../build/${PROJECT_NAME} # workaround for ui_pose_interpolator.h file
    ${catkin_INCLUDE_DIRS}
)

set(UI_FILES
    src/pose_interpolator.ui
    src/trajectory_editor.ui
)

set(H_FILES
    include/rqt_pose_interpolator/pose_interpolator.h
    include/rqt_pose_interpolator/trajectory_editor.h
)

if(USE_QT5)
    qt5_wrap_ui(UIC_FILES ${UI_FILES})
    qt5_wrap_cpp(MOC_FILES ${H_FILES})
else()
    qt4_wrap_ui(UIC_FILES ${UI_FILES})
    qt4_wrap_cpp(MOC_FILES ${H_FILES})
endif()

add_library(pose_interpolator_plugins
    ${UIC_FILES}
    ${MOC_FILES}
    src/plugins.cpp
    src/pose_interpolator.cpp
    src/trajectory_editor.cpp
)

target_link_libraries(pose_interpolator_plugins
    ${catkin_LIBRARIES}
    ${QT_LIBRARIES}
    yaml-cpp
)

#############
## Install ##
#############

install(TARGETS pose_interpolator_plugins
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)




# Dummy target for IDE's
FILE(GLOB_RECURSE all_headers_for_ides
        ${CMAKE_SOURCE_DIR}/**/*.hpp
        ${CMAKE_SOURCE_DIR}/**/*.h)
ADD_CUSTOM_TARGET(ide_includes_${PROJECT_NAME} SOURCES ${all_headers_for_ides})
