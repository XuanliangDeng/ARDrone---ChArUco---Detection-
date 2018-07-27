cmake_minimum_required(VERSION 2.8.3)
project(ardrone_test)


add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  camera_info_manager
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
)

find_package(OpenCV)

catkin_package()

include_directories(

  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ./include

)

add_executable(main src/ChArUco.cpp src/main.cpp)
target_link_libraries(main ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})












