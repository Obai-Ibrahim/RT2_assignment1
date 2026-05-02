cmake_minimum_required(VERSION 3.8)
project(assignment_1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(custom_interfaces REQUIRED)
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


# build server executable
#[[
add_executable(server_node src/server.cpp)
ament_target_dependencies(server_node rclcpp rclcpp_action geometry_msgs nav_msgs rclcpp_components custom_interfaces)
install(TARGETS
  server_node
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(client_node src/client.cpp)
ament_target_dependencies(client_node rclcpp rclcpp_action rclcpp_components custom_interfaces)
install(TARGETS
  client_node
  DESTINATION lib/${PROJECT_NAME}
)
]]
add_library(nav_components_lib SHARED
  src/client.cpp  # Use your actual filename here
  src/server.cpp  # Use your actual filename here
)

ament_target_dependencies(nav_components_lib 
  rclcpp 
  rclcpp_action 
  rclcpp_components 
  geometry_msgs 
  nav_msgs 
  tf2 
  tf2_ros 
  tf2_geometry_msgs 
  custom_interfaces
)

# Register the Client
rclcpp_components_register_node(nav_components_lib
  PLUGIN "nav_system::NavClient"
  EXECUTABLE client_component_exe
)

# Register the Server (Don't forget this!)
rclcpp_components_register_node(nav_components_lib
  PLUGIN "nav_system::Server"       # Matches the class name in server.cpp
  EXECUTABLE server_component_exe      # A unique nickname for the server
)

install(TARGETS nav_components_lib
  EXPORT export_nav_components
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_nav_components)

ament_package()

