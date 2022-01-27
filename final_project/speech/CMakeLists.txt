cmake_minimum_required(VERSION 2.8.3)
project(speech)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

find_package(catkin REQUIRED cmake_modules COMPONENTS
  roscpp
  std_msgs
  naoqi_bridge_msgs
  std_srvs
  tf
  actionlib_msgs
  geometry_msgs
  aruco
  aruco_msgs
  sensor_msgs
  image_transport
  cv_bridge
)

find_package(Boost REQUIRED COMPONENTS system thread)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(qibuild)


add_message_files(
    FILES
    face_area.msg
    arucomsgs.msg
    localizationmsgs.msg
)



add_service_files(
    DIRECTORY srv
    FILES
    walk_polar.srv
    prespeech.srv
    facerecoged.srv
    localization.srv
)

add_action_files(
    FILES
    Blink.action
    )

generate_messages(
    DEPENDENCIES
    actionlib_msgs
    std_msgs
    geometry_msgs
    )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES aruco_ros_utils 
  CATKIN_DEPENDS actionlib_msgs aruco sensor_msgs tf)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
install ( PROGRAMS
scripts/preSpeechServer.py
DESTINATION $${CATKIN_PACKAGE_BIN_DESTINATION}
)

install ( PROGRAMS
scripts/poseinit.py
DESTINATION $${CATKIN_PACKAGE_BIN_DESTINATION}
)

install ( PROGRAMS
scripts/face_recog.py
DESTINATION $${CATKIN_PACKAGE_BIN_DESTINATION}
)

install ( PROGRAMS
scripts/hand_operation.py
DESTINATION $${CATKIN_PACKAGE_BIN_DESTINATION}
)

install ( PROGRAMS
scripts/collisionprotection.py
DESTINATION $${CATKIN_PACKAGE_BIN_DESTINATION}
)

install ( PROGRAMS
scripts/navtest.py
DESTINATION $${CATKIN_PACKAGE_BIN_DESTINATION}
)

install ( PROGRAMS
scripts/pre_grasp.py
DESTINATION $${CATKIN_PACKAGE_BIN_DESTINATION}
)

install ( PROGRAMS
scripts/downhead.py
DESTINATION $${CATKIN_PACKAGE_BIN_DESTINATION}
)

install ( PROGRAMS
scripts/risehead.py
DESTINATION $${CATKIN_PACKAGE_BIN_DESTINATION}
)

install ( PROGRAMS
scripts/congwalk.py
DESTINATION $${CATKIN_PACKAGE_BIN_DESTINATION}
)

install ( PROGRAMS
scripts/walk_general.py
DESTINATION $${CATKIN_PACKAGE_BIN_DESTINATION}
)

install ( PROGRAMS
scripts/mapping.py
DESTINATION $${CATKIN_PACKAGE_BIN_DESTINATION}
)

install ( PROGRAMS
scripts/shakehead.py
DESTINATION $${CATKIN_PACKAGE_BIN_DESTINATION}
)

install ( PROGRAMS
scripts/localization.py
DESTINATION $${CATKIN_PACKAGE_BIN_DESTINATION}
)

install ( PROGRAMS
scripts/naoposition.py
DESTINATION $${CATKIN_PACKAGE_BIN_DESTINATION}
)

# add_executable(speech src/speech.cpp)
# include_directories(${Boost_INCLUDE_DIRS})
# target_link_libraries(speech ${catkin_LIBRARIES} ${Boost_LIBRARIES})
# add_dependencies(speech ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(test_node src/main.cpp)
include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(test_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(test_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(marker_dect src/marker_dect.cpp)
include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(marker_dect ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(marker_dect
${OpenCV_LIBRARIES}
)
add_dependencies(marker_dect speech_generate_messages ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# add_executable(testpub src/pub.cpp)
# add_dependencies(testpub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(testpub ${catkin_LIBRARIES})



#qi_create_bin(speech src/main.cpp)
#qi_use_lib(sayhelloworld ALCOMMON)

