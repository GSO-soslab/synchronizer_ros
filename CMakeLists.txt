cmake_minimum_required(VERSION 2.8.3)

project(synchronizer_ros)

# Set compile flags (-Ofast actually makes a big difference over -O3 here (maybe 20% faster)
set(CMAKE_CXX_FLAGS "-std=c++11 -Ofast")

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  sensor_msgs
  message_generation
  roscpp
  rospy
  nodelet

  image_transport
  rosserial_arduino
  rosserial_client
  rosserial_python

  dynamic_reconfigure
)

add_message_files(
  FILES
  ImageNumbered.msg
  ImuMicro.msg
  TimeNumbered.msg
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
)

generate_dynamic_reconfigure_options(
  cfg/Science.cfg
  cfg/Led.cfg
  cfg/Servo.cfg
  cfg/Clock.cfg
  
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_nodelet
  CATKIN_DEPENDS message_runtime
)

include_directories(
	include
    ${catkin_INCLUDE_DIRS}
)

#############
# LIBRARIES #
#############

# link_directories(
# 	../firmware/libraries/ADIS16445/
# )

add_library(${PROJECT_NAME}_nodelet
  src/synchronizer_nodelet.cpp
  src/synchronizer.cpp
)

target_link_libraries(${PROJECT_NAME}_nodelet
  ${catkin_LIBRARIES}
)
add_dependencies(${PROJECT_NAME}_nodelet ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME}_node src/synchronizer_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})

add_executable(synchronizer_manager_node src/synchronizer_manager.cpp)
target_link_libraries(synchronizer_manager_node ${catkin_LIBRARIES})
add_dependencies(synchronizer_manager_node ${PROJECT_NAME}_gencfg)

# add_executable(versavis_imu_receiver
#    src/versavis_imu_receiver.cpp
# )
# add_dependencies(versavis_imu_receiver ${${PROJECT_NAME}_EXPORTED_TARGETS})
# target_link_libraries(versavis_imu_receiver ${catkin_LIBRARIES})

############
## EXPORT ##
############

# # Install nodelet library
# install(TARGETS ${PROJECT_NAME}_nodelet
#   DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# )

# # Install header files
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# )

# # Install launch files
# install(DIRECTORY launch/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
# )

# # Install xml files
# install(FILES nodelet_plugin.xml
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
