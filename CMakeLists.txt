cmake_minimum_required(VERSION 2.8.3)
project(omnicopter_sim)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  sensor_msgs
  message_generation
  std_msgs
  tf
)

add_message_files(
	FILES
    RCInput.msg
    MotorCommand.msg
    AttSp.msg
    PosSp.msg
    Arm.msg
    State.msg
)

#Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
	LIBRARIES rtimulib_ros
	CATKIN_DEPENDS message_runtime std_msgs
)

include_directories("/usr/local/include/Eigen")

include_directories(include ${catkin_INCLUDE_DIRS})
#Add all executables
add_executable(omnicopter src/omnicopter.cpp)
add_executable(control_allocation src/control_allocation.cpp)
add_executable(attitude_controller src/attitude_controller.cpp)
add_executable(position_controller src/position_controller.cpp)
add_executable(rc_mapping src/rc_mapping.cpp)
add_executable(visualization src/visualization.cpp)

add_dependencies(omnicopter ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(control_allocation ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(rc_mapping ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(visualization ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(attitude_controller ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(position_controller ${${PROJECT_NAME}_EXPORTED_TARGETS})

target_link_libraries(omnicopter ${catkin_LIBRARIES})
target_link_libraries(control_allocation ${catkin_LIBRARIES})
target_link_libraries(attitude_controller ${catkin_LIBRARIES})
target_link_libraries(position_controller ${catkin_LIBRARIES})
target_link_libraries(rc_mapping ${catkin_LIBRARIES})
target_link_libraries(visualization ${catkin_LIBRARIES})



