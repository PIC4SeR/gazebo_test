cmake_minimum_required(VERSION 3.8)
project(gazebo_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(hunav_sim QUIET)
find_package(std_srvs REQUIRED)

#display a warning if hunav_sim is not found

if(NOT hunav_sim_FOUND)
  message(WARNING "hunav_sim not found. this is fine for testing in real world, but not for simulation and training")
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY
  launch
  config
  maps
  goals_and_poses
  behavior_trees
  rviz
  experiments
  DESTINATION share/${PROJECT_NAME}/
)


# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python scripts
install(PROGRAMS
  scripts/experiment_manager
  scripts/full_experiment
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
