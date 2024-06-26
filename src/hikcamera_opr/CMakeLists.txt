cmake_minimum_required(VERSION 3.0.2)
project(hikcamera_opr)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  rospy
  sensor_msgs
  std_msgs
  image_transport
)

find_package(OpenCV 4 REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES msg_hikcamera
  CATKIN_DEPENDS cv_bridge roscpp rospy sensor_msgs std_msgs
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  include /opt/MVS/include/
  include 3rdParty/
)

##For aarch64 system, please change the path to /opt/MVS/lib/aarch64/
LINK_DIRECTORIES(
  /opt/MVS/lib/64/
  3rdParty/
)

## Declare a C++ library
add_library(hikcamera_lib
  include/hikcamera.h
  include/hikcameraDataType.h
  lib/hikcamera.cpp
)

## 3rdParty
file(GLOB_RECURSE COMM_SRC_FILES 3rdParty/comm/*.cpp)
file(GLOB_RECURSE COMM_HEAD_FILES 3rdParty/comm/*.h)
add_library(comm_lib
  ${COMM_SRC_FILES}
  ${COMM_HEAD_FILES}
)
file(GLOB_RECURSE TIMESYNC_SRC_FILES 3rdParty/timesync/*.cpp)
file(GLOB_RECURSE TIMESYNC_HEAD_FILES 3rdParty/timesync/*.h)
add_library(timesync_lib
  ${TIMESYNC_SRC_FILES}
  ${TIMESYNC_HEAD_FILES}
)
file(GLOB_RECURSE FASTCRC_SRC_FILES 3rdParty/FastCRC/*.cpp)
file(GLOB_RECURSE FASTCRC_HEAD_FILES 3rdParty/FastCRC/*.h)
add_library(FastCRC_lib
  ${FASTCRC_SRC_FILES}
  ${FASTCRC_HEAD_FILES}
)


## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(hikcamera_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_dependencies(comm_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(timesync_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(FastCRC_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


target_link_libraries(timesync_lib
  ${catkin_LIBRARIES}
  comm_lib
  FastCRC_lib
  )


target_link_libraries(hikcamera_lib
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  #  GCBase_gcc421_v3_0
  MvCameraControl
  #  GenApi_gcc421_v3_0
  MVGigEVisionSDK
  #  log4cpp_gcc421_v3_0
  MVRender
  #  Log_gcc421_v3_0
  MvUsb3vTL
  #  MathParser_gcc421_v3_0
  #  NodeMapData_gcc421_v3_0
  MediaProcess
  #  XmlParser_gcc421_v3_0
  X11

  timesync_lib
  )


## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(msg_hikcamera_grab src/msg_hikcamera_grab.cpp)
add_executable(msg_hikcamera_info src/msg_hikcamera_info.cpp)
add_executable(img_hikcamera_grab_save src/img_hikcamera_grab_save.cpp)
add_executable(key_input src/key_input.cpp)
add_executable(img_show src/img_show.cpp)

add_executable(GPRMC_sync_grab_trigger src/GPRMC_sync_grab_trigger.cpp)


## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(msg_hikcamera_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against

target_link_libraries(msg_hikcamera_grab
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  hikcamera_lib
)

target_link_libraries(msg_hikcamera_info
  ${catkin_LIBRARIES}
)

target_link_libraries(img_hikcamera_grab_save
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  hikcamera_lib
)

target_link_libraries(key_input
  ${catkin_LIBRARIES}
)

target_link_libraries(img_show
  ${catkin_LIBRARIES}
)

target_link_libraries(GPRMC_sync_grab_trigger
  ${catkin_LIBRARIES}
  hikcamera_lib
)



#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_msg_hikcamera.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
