cmake_minimum_required(VERSION 3.10)
project(waypoint_telep)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(orocos_kdl_vendor REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdf REQUIRED)

add_executable(waypoint_telep_node src/waypoint_telep_node.cpp)
target_compile_features(waypoint_telep_node PUBLIC cxx_std_17)
target_include_directories(waypoint_telep_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(waypoint_telep_node
  control_msgs
  geometry_msgs
  kdl_parser
  nav_msgs
  rclcpp
  rclcpp_action
  sensor_msgs
  std_msgs
  std_srvs
  trajectory_msgs
  tf2
  tf2_ros
  urdf
)
target_link_libraries(waypoint_telep_node orocos-kdl)

install(TARGETS waypoint_telep_node
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS scripts/waypoint_telep_ui.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME waypoint_telep_ui
)
install(PROGRAMS scripts/waypoint_stream_worker.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME waypoint_stream_worker
)
install(DIRECTORY config launch web
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)
install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(test_path_math test/test_path_math.cpp)
  target_compile_features(test_path_math PUBLIC cxx_std_17)
  target_include_directories(test_path_math PRIVATE include)
  ament_add_gtest(test_limit_ik test/test_limit_ik.cpp)
  target_compile_features(test_limit_ik PUBLIC cxx_std_17)
  target_include_directories(test_limit_ik PRIVATE include)
  target_link_libraries(test_limit_ik orocos-kdl)
  ament_add_pytest_test(test_mock_integration test/test_mock_integration.py TIMEOUT 240)
  ament_add_pytest_test(test_ui_logic test/test_ui_logic.py TIMEOUT 30)
endif()

ament_package()
