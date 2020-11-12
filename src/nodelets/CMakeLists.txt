cmake_minimum_required(VERSION 3.0)
project(ladybug_nodelets)

#########
# Build driver nodelet
add_library(LadybugDriverNodelet
  LadybugDriverNodelet.cpp
)
add_dependencies(LadybugDriverNodelet
  ladybug_driver
  ${catkin_EXPORTED_TARGETS}
)
target_include_directories(LadybugDriverNodelet
  PUBLIC
    ${CMAKE_SOURCE_DIR}/include
    ${catkin_INCLUDE_DIRS}
)
target_link_libraries(LadybugDriverNodelet
  ladybug_driver
  ${catkin_LIBRARIES}
)

#########
# Build recombiner nodelet
add_library(LadybugCombinerNodelet
  LadybugCombinerNodelet.cpp
)
add_dependencies(LadybugCombinerNodelet
  ladybug_decompression
  ${catkin_EXPORTED_TARGETS}
)
target_include_directories(LadybugCombinerNodelet
  PUBLIC
  ${CMAKE_SOURCE_DIR}/include
    ${catkin_INCLUDE_DIRS}
)
target_link_libraries(LadybugCombinerNodelet
  ladybug_decompression
  ${catkin_LIBRARIES}
)

#########
# Build separator nodelet
add_library(LadybugSeparatorNodelet
  LadybugSeparatorNodelet.cpp
)
add_dependencies(LadybugSeparatorNodelet
  ladybug_decompression
  ${catkin_EXPORTED_TARGETS}
)
target_include_directories(LadybugSeparatorNodelet
  PUBLIC
    ${CMAKE_SOURCE_DIR}/include
    ${catkin_INCLUDE_DIRS}
)
target_link_libraries(LadybugSeparatorNodelet
  ladybug_decompression
  ${catkin_LIBRARIES}
)

#########
# Build separator nodelet
add_library(LadybugRectifierNodelet
  LadybugRectifierNodelet.cpp
)
add_dependencies(LadybugRectifierNodelet
  ladybug_decompression
  ${catkin_EXPORTED_TARGETS}
)
target_include_directories(LadybugRectifierNodelet
  PUBLIC
  ${CMAKE_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(LadybugRectifierNodelet
  ladybug_decompression
  ${catkin_LIBRARIES}
)

#########
# Install nodelets
install(
  TARGETS
    LadybugDriverNodelet
    LadybugSeparatorNodelet
    LadybugCombinerNodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
