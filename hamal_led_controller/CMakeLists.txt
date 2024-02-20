cmake_minimum_required(VERSION 3.0.2)
project(hamal_led_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  geometry_msgs
  actionlib_msgs
  message_generation
  hamal_custom_interfaces
)

add_message_files(
  FILES
  Led.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs hamal_custom_interfaces
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/led_controller.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
