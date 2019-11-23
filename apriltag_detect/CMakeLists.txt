cmake_minimum_required(VERSION 2.8.3)
project(apriltag_detect)

set (CMAKE_CXX_STANDARD 14)


find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  rospy
  sensor_msgs
  std_msgs
  geometry_msgs
  cv_bridge
  image_geometry
  image_transport
  message_generation
  tf
)

find_package(OpenCV REQUIRED)
find_package(PkgConfig)
pkg_search_module(apriltag REQUIRED apriltag)
set(apriltag_INCLUDE_DIRS "${apriltag_INCLUDE_DIRS}/apriltag")
link_directories(${apriltag_LIBDIR})

add_message_files(
  FILES
  graphing.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS
   cv_bridge
   geometry_msgs
   image_transport
   roscpp
   rospy
   sensor_msgs
   std_msgs
   tf
   message_runtime
 DEPENDS
   OpenCV
   apriltag
 LIBRARIES
  ${PROJECT_NAME}_tag_detector
)



include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${apriltag_INCLUDE_DIRS}
)


add_library(${PROJECT_NAME}_tag_detector src/TagDetector.cpp)
target_link_libraries(${PROJECT_NAME}_tag_detector ${catkin_LIBRARIES})

add_executable(detector src/tag_detection_node.cpp)
add_dependencies(detector ${PROJECT_NAME}_tag_detector)
target_link_libraries(detector ${PROJECT_NAME}_tag_detector ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${apriltag_LIBRARIES})
