cmake_minimum_required(VERSION 3.0)
project(ladybug_camera)

set(CMAKE_CXX_STANDARD 17)

set(build_depends
  camera_info_manager
  cv_bridge
  diagnostic_updater
  dynamic_reconfigure
  image_exposure_msgs
  image_transport
  message_generation
  nodelet
  nodelet_topic_tools
  roscpp
  rosbag
  sensor_msgs
)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR})
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules")

find_package(catkin REQUIRED COMPONENTS
  ${build_depends}
)
find_package(OpenCV 3 REQUIRED)
find_package(Ladybug REQUIRED)
find_package(DC1394 REQUIRED)

## Generate dynamic Reconfig messages
generate_dynamic_reconfigure_options(
  cfg/Ladybug.cfg
)

add_message_files(
  DIRECTORY
    msg
  FILES
    LadybugTiles.msg
    LadybugTilesCompressed.msg
)
generate_messages(
  DEPENDENCIES
    std_msgs
    sensor_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
    ${Ladybug_INCLUDE_DIRS}
  LIBRARIES
    ${PROJECT_NAME}
    ${Ladybug_LIBRARIES}
  CATKIN_DEPENDS
    ${build_depends}
  DEPENDS
    ${Ladybug_LIBRARIES}
)

add_subdirectory(lib/fastjpeg)
add_subdirectory(src/libs/decompression)
add_subdirectory(src/libs/driver)
add_subdirectory(src/nodes)
add_subdirectory(src/nodelets)

add_executable(reorder_bag
  src/reorder_bag.cpp
)
target_include_directories(reorder_bag
  PUBLIC
    ${catkin_INCLUDE_DIRS}
  )
target_link_libraries(reorder_bag
  ${catkin_LIBRARIES}
)

#############
## INSTALL ##
#############
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)