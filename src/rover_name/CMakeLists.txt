cmake_minimum_required(VERSION 2.8.3)
project(rover_name)

SET(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  )

  catkin_package(
    CATKIN_DEPENDS roscpp std_msgs
  )

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  ${PROJECT_NAME}_node
  src/rover_name.cpp
)

add_dependencies(
  ${PROJECT_NAME}_node
  ${catkin_EXPORTED_TARGETS})

target_link_libraries(
  ${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
