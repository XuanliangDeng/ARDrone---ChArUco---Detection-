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


#add_executable(ardrone src/ardrone.cpp)
#target_link_libraries(ardrone ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#add_executable(ardrone_detect src/ardrone_detect.cpp)
#target_link_libraries(ardrone_detect ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


#add_executable(Detection src/Detection.cpp)
#target_link_libraries(Detection ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#add_executable(ChArUco_Detection src/ChArUco_Detection.cpp)
#target_link_libraries(ChArUco_Detection ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#add_executable(charuco_SLAM src/charuco_SLAM.cpp)
#target_link_libraries(charuco_SLAM ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


add_executable(main src/ChArUco.cpp src/main.cpp)
target_link_libraries(main ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})












