cmake_minimum_required(VERSION 3.8)
project(cclp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(raylib 3.0 REQUIRED)
ament_auto_find_build_dependencies()

add_executable(cord_correction_node src/cord_correction_node.cpp)
target_link_libraries(cord_correction_node raylib)
ament_target_dependencies(
  cord_correction_node
  "rclcpp"
  "rosidl_default_generators"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
  "geometry_msgs"
  "std_msgs"
  "sensor_msgs"
)

add_executable(correction_test src/correction_test.cpp)
target_link_libraries(correction_test raylib)
ament_target_dependencies(
  correction_test
  "rclcpp"
)

install(TARGETS
  cord_correction_node
  correction_test
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
