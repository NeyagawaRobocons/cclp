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

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Point2D.msg"
  "msg/Line.msg"
  "msg/LineArray.msg"
  DEPENDENCIES std_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
ament_export_dependencies(rosidl_default_runtime)


ament_auto_add_executable(cord_correction_node src/cord_correction_node.cpp)
target_link_libraries(cord_correction_node raylib)
target_link_libraries(cord_correction_node "${cpp_typesupport_target}")

ament_auto_add_executable(monitor src/monitor.cpp)
target_link_libraries(monitor raylib)
target_link_libraries(monitor "${cpp_typesupport_target}")

ament_auto_add_executable(correction_test src/correction_test.cpp)
target_link_libraries(correction_test raylib)
target_link_libraries(correction_test "${cpp_typesupport_target}")

install(TARGETS
  cord_correction_node
  monitor
  correction_test
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

find_package(ament_cmake_python REQUIRED)

# ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  scripts/line_map_server_node.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
