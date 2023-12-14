cmake_minimum_required(VERSION 3.0.2)
project(camera)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  cv_bridge
  image_transport
  message_generation
)
set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_service_files(
  FILES
  CalibrateColor.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs cv_bridge image_transport
  DEPENDS OpenCV
  CATKIN_DEPENDS message_runtime
)


add_executable(camera_node src/camera.cpp)
add_executable(color_detection_node src/color_detection.cpp)
add_executable(aruco_detection_node src/aruco_detec.cpp)


target_link_libraries(camera_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(color_detection_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(aruco_detection_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


