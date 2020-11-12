cmake_minimum_required(VERSION 3.0)
project(ladybug_driver)

find_package(OpenMP REQUIRED)

# ladybug_camera library
add_library(${PROJECT_NAME}
  LadybugInterface.cpp
  LadybugCompressed.cpp
  LadybugRaw.cpp
  LadybugRectified.cpp
)
add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    ${CMAKE_SOURCE_DIR}/include
    ${catkin_INCLUDE_DIRS}
    ${Ladybug_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Ladybug_LIBRARIES}
  ${OpenCV_LIBS}
  OpenMP::OpenMP_CXX
)
