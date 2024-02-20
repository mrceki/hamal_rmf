cmake_minimum_required(VERSION 3.5.0)
project(hamal_ff_client LANGUAGES C CXX)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # we dont use add_compile_options with pedantic in message packages
  # because the Python C extensions dont comply with it
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

include(GNUInstallDirs)

#===============================================================================

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_srvs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  actionlib
  move_base_msgs
  message_generation
  std_msgs
  bt_amr
  hamal_custom_interfaces
)

add_service_files(
  DIRECTORY srv
  FILES DockingBt.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

option(BUILD_DOCKING_SERVICE "Build hamal_docking_service and link with bt_amr library" ON)

if (catkin_FOUND) 

  find_package(free_fleet REQUIRED)
  if(BUILD_DOCKING_SERVICE)
    find_package(bt_amr REQUIRED)
  endif()

  catkin_package(
    CATKIN_DEPENDS roscpp std_msgs 
    LIBRARIES bt_amr
  )

  #=============================================================================

  add_executable(hamal_ff_client
    src/main.cpp
    src/utilities.cpp
    src/ClientNode.cpp
    src/ClientNodeConfig.cpp
  )

  if(BUILD_DOCKING_SERVICE)
    add_executable(hamal_docking_service
      src/docking_bt_service.cpp
    )
    target_link_libraries(hamal_docking_service
      ${catkin_LIBRARIES}
      ${bt_amr_LIBRARIES}
    )
    target_include_directories(hamal_docking_service
      PRIVATE
        ${free_fleet_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
        ${bt_amr_INCLUDE_DIRS}
    )
    add_dependencies(hamal_docking_service
    ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  endif()

  target_link_libraries(hamal_ff_client
    ${free_fleet_LIBRARIES}
    ${catkin_LIBRARIES}
    ${bt_amr_LIBRARIES}  
  )
  target_include_directories(hamal_ff_client
    PRIVATE
      ${free_fleet_INCLUDE_DIRS}
      ${catkin_INCLUDE_DIRS}
      ${bt_amr_INCLUDE_DIRS}  
  )

  #=============================================================================

  set(testing_targets
    fake_action_server
    fake_docking_server
    fake_lifter_server
    fake_lifter_action_server
  )
  
  foreach(target ${testing_targets})
    add_executable(${target}
      src/tests/${target}.cpp
    )
    target_link_libraries(${target}
      ${catkin_LIBRARIES}
    )
    target_include_directories(${target}
      PRIVATE
        ${catkin_INCLUDE_DIRS}
    )
  endforeach()
  
  #=============================================================================

  install(
    TARGETS
    hamal_ff_client
      ${testing_targets}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

else()
  message("catkin not found so skipping this ROS1 package")
endif()
