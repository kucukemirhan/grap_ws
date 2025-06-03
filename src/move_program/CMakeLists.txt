cmake_minimum_required(VERSION 3.8)
project(move_program)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

add_executable(move_program src/move_program.cpp)
add_executable(xyz_trajectory src/xyz_trajectory.cpp)
add_executable(cartesian_path src/cartesian_path.cpp)

target_include_directories(move_program PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(move_program PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(
  move_program
  "moveit_ros_planning_interface"
  "rclcpp"
)


target_include_directories(xyz_trajectory PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(xyz_trajectory PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(
  xyz_trajectory
  "moveit_ros_planning_interface"
  "rclcpp"
)


target_include_directories(cartesian_path PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(cartesian_path PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(
  cartesian_path
  "moveit_ros_planning_interface"
  "rclcpp"
)

install(TARGETS 
  xyz_trajectory
  move_program
  cartesian_path
  DESTINATION lib/${PROJECT_NAME})
  
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
