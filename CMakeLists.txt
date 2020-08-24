cmake_minimum_required(VERSION 3.0.2)
project(mono_vision_ros)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
)


find_package( OpenCV 4.4 REQUIRED )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mono_vision_ros
#  CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(mono_vision src/main.cpp)
target_link_libraries(mono_vision ${catkin_LIBRARIES} ${OpenCV_LIBS})


