cmake_minimum_required(VERSION 2.8.3)
project(nao_control_tutorial_3)


find_package(catkin REQUIRED cmake_modules COMPONENTS
  roscpp
  std_msgs
  naoqi_bridge_msgs
  std_srvs
  tf
  actionlib_msgs
  geometry_msgs
)

find_package(Boost REQUIRED COMPONENTS system thread)



add_action_files(
    FILES
    Blink.action
    )

generate_messages(
    DEPENDENCIES
    actionlib_msgs
    std_msgs
    )

catkin_package(CATKIN_DEPENDS actionlib_msgs)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(nao_3 src/main.cpp)
include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(nao_3 ${catkin_LIBRARIES} ${Boost_LIBRARIES})


