cmake_minimum_required(VERSION 3.8)
project(husky_planner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(
  include
)

set(SOURCE_FILES
  src/a_star_utils/a_star_utils.cpp
  src/environment/environment.cpp
  src/prm_utils/prm_utils.cpp
  src/sampler_structs/distribution.cpp
  src/planner.cpp
  src/sampler.cpp
)

add_executable(build_env_node src/nodes/build_env.cpp ${SOURCE_FILES})
ament_target_dependencies(build_env_node rclcpp std_msgs visualization_msgs)
#target_include_directories(build_env_node PUBLIC
#  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#  $<INSTALL_INTERFACE:include>
#)
add_executable(test_planner src/nodes/test_planner.cpp src/planner.cpp src/sampler.cpp src/sampler_structs/distribution.cpp src/prm_utils/prm_utils.cpp src/a_star_utils/a_star_utils.cpp src/environment/environment.cpp)
ament_target_dependencies(test_planner rclcpp std_msgs visualization_msgs)

add_executable(path_planner src/nodes/path_planner.cpp src/planner.cpp src/sampler.cpp src/sampler_structs/distribution.cpp src/prm_utils/prm_utils.cpp src/a_star_utils/a_star_utils.cpp src/environment/environment.cpp)
ament_target_dependencies(path_planner rclcpp std_msgs visualization_msgs)

add_executable(basic_line src/nodes/standalone_line.cpp)
ament_target_dependencies(basic_line rclcpp std_msgs visualization_msgs)

install(TARGETS
  build_env_node
  test_planner
  path_planner
  basic_line
  DESTINATION lib/${PROJECT_NAME}
)

#install(DIRECTORY include/
#  DESTINATION include/
#)

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

ament_package()
