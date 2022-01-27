cmake_minimum_required(VERSION 2.8.3)
project(nao_control_tutorial_2)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  rospy
  message_generation
  sensor_msgs
  cv_bridge
  image_transport
  aruco_ros
  geometry_msgs
)

find_package(OpenCV REQUIRED)

add_service_files(
    DIRECTORY srv
    FILES
    MoveJoints.srv
    coordinate.srv
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs) 
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  
)
catkin_package(CATKIN_DEPENDS message_runtime)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS ./script/move_service.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(naoControl2 src/client.cpp)
target_link_libraries(naoControl2 ${catkin_LIBRARIES})
target_link_libraries(naoControl2
${OpenCV_LIBRARIES}
)

add_dependencies(naoControl2 ${naoControl2_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
