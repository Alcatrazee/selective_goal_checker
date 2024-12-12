cmake_minimum_required(VERSION 3.8)
project(selective_goal_checker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_2d_msgs REQUIRED)
find_package(nav_2d_utils REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(selective_goal_checker SHARED src/selective_goal_checker.cpp)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
# target_link_libraries(selective_goal_checker PUBLIC
#   ${geometry_msgs_TARGETS}
#   nav2_core::nav2_core
#   rclcpp::rclcpp
#   rclcpp_lifecycle::rclcpp_lifecycle
#   ${rcl_interfaces_TARGETS}
#   nav2_util::nav2_util_core
#   nav_2d_utils::conversions
#   pluginlib::pluginlib
# )

ament_target_dependencies(selective_goal_checker
  rclcpp
  pluginlib
  nav2_core
  nav2_util
  geometry_msgs
)

target_include_directories(selective_goal_checker
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

pluginlib_export_plugin_description_file(nav2_core selective_goal_checker.xml)

# 安装插件
install(TARGETS selective_goal_checker
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(FILES selective_goal_checker.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
