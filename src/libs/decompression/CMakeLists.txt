cmake_minimum_required(VERSION 3.0)
project(ladybug_decompression)

find_package(OpenMP REQUIRED)

# ladybug_camera library
add_library(${PROJECT_NAME}
  LadybugSeparatorNode.cpp
  LadybugCombiner.cpp
  LadybugCombinerNode.cpp
  LadybugRectifierNode.cpp
  pgr_compressor_header_info.cpp
)
add_dependencies(${PROJECT_NAME}
  fastjpeg
  ${ladybug_camera_EXPORTED_TARGETS}
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    ${CMAKE_SOURCE_DIR}/include
    ${catkin_INCLUDE_DIRS}
    ${Ladybug_INCLUDE_DIRS}
    ${DC1394_INCLUDE_DIR}
)
target_link_libraries(${PROJECT_NAME}
  fastjpeg
  OpenMP::OpenMP_CXX
  ${catkin_LIBRARIES}
  ${Ladybug_LIBRARIES}
  ${DC1394_LIBRARY}
  )
