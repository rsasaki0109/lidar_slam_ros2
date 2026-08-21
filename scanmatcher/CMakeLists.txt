cmake_minimum_required(VERSION 3.5)
project(scanmatcher)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17 (required by small_gicp)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  #add_compile_options(-Wall -Wextra -Wpedantic)
  #add_compile_options(-Wextra -Wpedantic)
endif()

#SET(CMAKE_BUILD_TYPE "RELEASE")
SET(CMAKE_CXX_FLAGS "-O2 -g ${CMAKE_CXX_FLAGS}")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rosidl_runtime_c REQUIRED)
find_package(rosidl_runtime_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(lidarslam_msgs REQUIRED)
find_package(ndt_omp_ros2 REQUIRED)
find_package(lidarslam_plugin_interfaces REQUIRED)
find_package(lidarslam_default_plugins REQUIRED)
find_package(lidarslam_registration_loader REQUIRED)
find_package(fast_gicp QUIET)
find_package(small_gicp QUIET)
find_package(PCL REQUIRED)
find_package(OpenMP)

set(dependencies
  rclcpp
  rclcpp_components
  rcl_interfaces
  rosidl_runtime_c
  rosidl_runtime_cpp
  tf2_ros
  tf2_geometry_msgs
  tf2_sensor_msgs
  tf2_eigen
  pcl_conversions
  geometry_msgs
  sensor_msgs
  nav_msgs
  lidarslam_msgs
  ndt_omp_ros2
  lidarslam_plugin_interfaces
  lidarslam_registration_loader
)

if(fast_gicp_FOUND)
  list(APPEND dependencies fast_gicp)
  add_definitions(-DHAS_FAST_GICP)
  message(STATUS "fast_gicp found")
else()
  message(STATUS "fast_gicp not found, FAST_GICP/FAST_VGICP disabled")
endif()

if(small_gicp_FOUND)
  add_definitions(-DHAS_SMALL_GICP)
  message(STATUS "small_gicp found")
else()
  message(STATUS "small_gicp not found, SMALL_GICP/SMALL_VGICP disabled")
endif()

if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_lidar_undistortion test/test_lidar_undistortion.cpp)
  target_include_directories(test_lidar_undistortion PRIVATE include ${PCL_INCLUDE_DIRS})
  target_link_libraries(test_lidar_undistortion ${PCL_LIBRARIES})

  ament_add_gtest(test_math_utils test/test_math_utils.cpp)
  target_include_directories(test_math_utils PRIVATE include ${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
  ament_target_dependencies(test_math_utils
    tf2_geometry_msgs
    tf2_eigen
    geometry_msgs
  )
  target_link_libraries(test_math_utils ${PCL_LIBRARIES})

  ament_add_gtest(test_odom_prior_utils test/test_odom_prior_utils.cpp)
  target_include_directories(test_odom_prior_utils PRIVATE include ${EIGEN3_INCLUDE_DIR})

  ament_add_gtest(test_pose_prediction test/test_pose_prediction.cpp)
  target_include_directories(test_pose_prediction PRIVATE include ${EIGEN3_INCLUDE_DIR})
  ament_target_dependencies(test_pose_prediction tf2)

  ament_add_gtest(test_pose_acceptance test/test_pose_acceptance.cpp)
  target_include_directories(test_pose_acceptance PRIVATE include ${EIGEN3_INCLUDE_DIR})
  ament_target_dependencies(test_pose_acceptance tf2 tf2_geometry_msgs geometry_msgs)

  ament_add_gtest(test_imu_processing test/test_imu_processing.cpp)
  target_include_directories(test_imu_processing PRIVATE include ${EIGEN3_INCLUDE_DIR})
  ament_target_dependencies(test_imu_processing tf2 tf2_geometry_msgs geometry_msgs)

  ament_add_gtest(test_map_update_policy test/test_map_update_policy.cpp)
  target_include_directories(test_map_update_policy PRIVATE include ${EIGEN3_INCLUDE_DIR})

  ament_add_gtest(test_registration_config test/test_registration_config.cpp)
  target_include_directories(test_registration_config PRIVATE include)
  ament_target_dependencies(test_registration_config lidarslam_plugin_interfaces)

  ament_add_gtest(test_registration_runtime test/test_registration_runtime.cpp)
  target_include_directories(test_registration_runtime PRIVATE include)
  ament_target_dependencies(test_registration_runtime lidarslam_plugin_interfaces)

  ament_add_gtest(test_offline_map_export test/test_offline_map_export.cpp)
  target_include_directories(test_offline_map_export PRIVATE include ${PCL_INCLUDE_DIRS})
  ament_target_dependencies(test_offline_map_export
    tf2_geometry_msgs
    tf2_eigen
    lidarslam_msgs
    pcl_conversions)
  target_link_libraries(test_offline_map_export ${PCL_LIBRARIES})

  ament_add_gtest(test_point_colorizer test/test_point_colorizer.cpp)
  target_include_directories(test_point_colorizer PRIVATE include ${EIGEN3_INCLUDE_DIR})
endif()

add_library(scanmatcher_component SHARED
  src/scanmatcher_component.cpp
)

target_compile_definitions(scanmatcher_component PRIVATE "GS_SM_BUILDING_DLL")

if(small_gicp_FOUND)
  target_link_libraries(scanmatcher_component small_gicp::small_gicp)
endif()

ament_target_dependencies(scanmatcher_component ${dependencies})

if(TARGET lidarslam_default_plugins::lidarslam_default_plugins)
  # The shell includes the built-in implementation include so pclomp's
  # templates are instantiated in this legacy translation unit.  Consume the
  # package's headers and exported include context without linking its future
  # plugin DSO into the production shell.
  get_target_property(lidarslam_default_plugins_include_dirs
    lidarslam_default_plugins::lidarslam_default_plugins
    INTERFACE_INCLUDE_DIRECTORIES)
  target_include_directories(scanmatcher_component PUBLIC
    ${lidarslam_default_plugins_include_dirs})
else()
  message(FATAL_ERROR "lidarslam_default_plugins exported target is unavailable")
endif()
target_link_libraries(scanmatcher_component ${PCL_LIBRARIES})
if(OpenMP_CXX_FOUND)
  target_link_libraries(scanmatcher_component OpenMP::OpenMP_CXX)
endif()

if(BUILD_TESTING)
  ament_add_gtest(test_registration_plugin_injection test/test_registration_plugin_injection.cpp)
  target_include_directories(test_registration_plugin_injection PRIVATE include)
  ament_target_dependencies(test_registration_plugin_injection
    rclcpp lidarslam_registration_loader)
  target_link_libraries(test_registration_plugin_injection scanmatcher_component ${PCL_LIBRARIES})

  ament_add_gtest(test_fast_gicp_selector test/test_fast_gicp_selector.cpp)
  target_include_directories(test_fast_gicp_selector PRIVATE include)
  ament_target_dependencies(test_fast_gicp_selector rclcpp)
  target_link_libraries(test_fast_gicp_selector scanmatcher_component ${PCL_LIBRARIES})
endif()

add_executable(scanmatcher_node
src/scanmatcher_node.cpp
)

target_link_libraries(scanmatcher_node
scanmatcher_component ${PCL_LIBRARIES})

ament_target_dependencies(scanmatcher_node
  rclcpp)

find_package(rosbag2_cpp REQUIRED)
add_executable(scan_matcher_offline_runner
  src/scan_matcher_offline_runner.cpp
)
target_link_libraries(scan_matcher_offline_runner
  scanmatcher_component ${PCL_LIBRARIES})
ament_target_dependencies(scan_matcher_offline_runner
  rclcpp rosbag2_cpp sensor_msgs geometry_msgs lidarslam_msgs pcl_conversions
  lidarslam_registration_loader)

if(small_gicp_FOUND)
  add_executable(small_gicp_odom_node
  src/small_gicp_odom_node.cpp
  )
  ament_target_dependencies(small_gicp_odom_node
    rclcpp sensor_msgs nav_msgs tf2_ros pcl_conversions)
  target_link_libraries(small_gicp_odom_node
    ${PCL_LIBRARIES} small_gicp::small_gicp)
endif()

find_package(message_filters REQUIRED)
add_executable(pointcloud_colorization_node
  src/pointcloud_colorization_node.cpp
)
target_include_directories(pointcloud_colorization_node PRIVATE include ${EIGEN3_INCLUDE_DIR})
ament_target_dependencies(pointcloud_colorization_node
  rclcpp sensor_msgs geometry_msgs tf2_ros tf2_eigen pcl_conversions message_filters)
target_link_libraries(pointcloud_colorization_node ${PCL_LIBRARIES})

include_directories(
    include
    ${PCL_INCLUDE_DIRS}
)
link_directories(
  ${PCL_LIBRARY_DIRS}
)
add_definitions(${PCL_DEFINITIONS})

rclcpp_components_register_nodes(scanmatcher_component
  "graphslam::ScanMatcherComponent")

ament_export_libraries(scanmatcher_component)

install(
  DIRECTORY "include/"
  DESTINATION include
)

install(
  DIRECTORY launch param
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  scanmatcher_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

set(INSTALL_TARGETS scanmatcher_node scan_matcher_offline_runner
  pointcloud_colorization_node)
if(small_gicp_FOUND)
  list(APPEND INSTALL_TARGETS small_gicp_odom_node)
endif()
install(TARGETS ${INSTALL_TARGETS}
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_package()
