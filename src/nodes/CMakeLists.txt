cmake_minimum_required(VERSION 3.0)
project(ladybug_nodes)

#########
# ladybug_camera_node
add_executable(ladybug_driver_node
  ladybug_driver_node.cpp
)
add_dependencies(ladybug_driver_node
  ${catkin_EXPORTED_TARGETS}
)
target_include_directories(ladybug_driver_node
  PUBLIC
    include
    ${catkin_INCLUDE_DIRS}
  )
target_link_libraries(ladybug_driver_node
  ladybug_driver
  ${catkin_LIBRARIES}
)

#########
# ladybug_separator_node
add_executable(ladybug_separator_node
  ladybug_separator_node.cpp
)
target_include_directories(ladybug_separator_node
  PUBLIC
    include
    ${catkin_INCLUDE_DIRS}
)
target_link_libraries(ladybug_separator_node
  ladybug_decompression
  ${catkin_LIBRARIES}
)

#########
# ladybug_recombine_node
add_executable(ladybug_combiner_node
  ladybug_combiner_node.cpp
)
target_include_directories(ladybug_combiner_node
  PUBLIC
    include
    ${catkin_INCLUDE_DIRS}
)
target_link_libraries(ladybug_combiner_node
  ladybug_decompression
  ${catkin_LIBRARIES}
)

#########
# separated frames publisher
add_executable(ladybug_frame_publisher_node
  ladybug_frame_publisher_node.cpp
)
add_dependencies(ladybug_frame_publisher_node
  ${catkin_EXPORTED_TARGETS}
)
target_include_directories(ladybug_frame_publisher_node
  PUBLIC
    include
    ${catkin_INCLUDE_DIRS}
)
target_link_libraries(ladybug_frame_publisher_node
  ladybug_decompression
  ${catkin_LIBRARIES}
)

#########
# rectify node
add_executable(ladybug_rectifier_node
  ladybug_rectifier_node.cpp
  )
add_dependencies(ladybug_rectifier_node
  ${catkin_EXPORTED_TARGETS}
  )
target_include_directories(ladybug_rectifier_node
  PUBLIC
  include
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(ladybug_rectifier_node
  ladybug_decompression
  ${catkin_LIBRARIES}
  ${OpenCV_Libraries}
)

## Install
install(
  TARGETS
    ladybug_driver_node
    ladybug_separator_node
    ladybug_combiner_node
    ladybug_frame_publisher_node
    ladybug_rectifier_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)