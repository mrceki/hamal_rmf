cmake_minimum_required(VERSION 3.5)
project(rmf_hamal)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmf_task_msgs REQUIRED)
find_package(rmf_dispenser_msgs REQUIRED)
find_package(rmf_ingestor_msgs REQUIRED)
find_package(rmf_fleet_msgs REQUIRED)
# find_package(gazebo_ros REQUIRED)
# find_package(rmf_building_map_tools REQUIRED)

# Include directories
include_directories(include)

# Add executable
add_executable(hamal_task_dispatcher src/hamal_task_dispatcher.cpp)

# Link the executable with the necessary libraries
ament_target_dependencies(hamal_task_dispatcher
  rclcpp
  rmf_task_msgs
  rmf_dispenser_msgs
  rmf_ingestor_msgs
  rmf_fleet_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION share/${PROJECT_NAME}/
)
# Install executable
install(TARGETS
  hamal_task_dispatcher
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  rmf_config
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter, which checks for copyrights
  # uncomment the line when copyright and license are not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a Git repo)
  # uncomment the line when this package is not in a Git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(ament_cmake rclcpp rmf_task_msgs rmf_dispenser_msgs rmf_ingestor_msgs)

ament_package()

message(NOTICE "Starting Compilation...") 

file(GLOB_RECURSE traffic_editor_paths "rmf_config/*.building.yaml")

foreach(path ${traffic_editor_paths})
  message(NOTICE "path: ${path}")

  # Get the output world name
  string(REGEX REPLACE "\\.[^.]*\.[^.]*$" "" no_extension_path ${path})
  string(REGEX MATCH "[^\/]+$" world_name  ${no_extension_path})

  set(map_path ${path})
  set(output_world_name ${world_name})
  set(output_dir ${CMAKE_CURRENT_BINARY_DIR}/maps/${output_world_name})
  set(output_world_path ${output_dir}/${output_world_name}.world)
  set(output_model_dir ${output_dir}/models)

  message(NOTICE "map_path: ${map_path}")
  message(NOTICE "output_world_name: ${output_world_name}")
  message(NOTICE "output_dir: ${output_dir}")

  message(NOTICE "output_world_path: ${output_world_path}")
  message(NOTICE "output_model_dir: ${output_model_dir}")

  ##############################################################################
  # Generate Gz world and download Models
  ##############################################################################

  message(NOTICE "BUILDING WORLD FILE WITH COMMAND: ros2 run rmf_building_map_tools building_map_generator gazebo ${map_path} ${output_world_path} ${output_model_dir}")
  message(NOTICE "....Downloading Models and Building to ${output_model_dir}")
  add_custom_command(
    OUTPUT ${output_world_path}
    COMMAND ros2 run rmf_building_map_tools building_map_generator gazebo ${map_path} ${output_world_path} ${output_model_dir}
    COMMAND ros2 run rmf_building_map_tools building_map_model_downloader ${map_path} -f -e ${output_model_dir}
    DEPENDS ${map_path}
  )

  add_custom_target(generate_${output_world_name}_world ALL
    DEPENDS ${output_world_path}
  )

  message(NOTICE "....Downloading and Building Models to ${output_model_dir} >>>>> DONE")


  message(NOTICE "....BUILDING WORLD FILE...DONE")

  ##############################################################################
  # Generate the nav graphs
  ##############################################################################

  set(output_nav_graphs_dir ${output_dir}/nav_graphs/)
  set(output_nav_graphs_phony ${output_nav_graphs_dir}/phony)
  message(NOTICE "....Generating Lane Graphs in  ${output_nav_graphs_dir}")
  add_custom_command(
    OUTPUT ${output_nav_graphs_phony}
    COMMAND ros2 run rmf_building_map_tools building_map_generator nav ${map_path} ${output_nav_graphs_dir}
    DEPENDS ${map_path}
  )

  add_custom_target(generate_${output_world_name}_nav_graphs ALL
    DEPENDS ${output_nav_graphs_phony}
  )

  install(
    DIRECTORY ${output_dir}
    DESTINATION share/${PROJECT_NAME}/maps
  )

  message(NOTICE "....Generating Lane Graphs in  ${output_nav_graphs_dir}...DONE")

endforeach()

