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
find_package(fast_gicp QUIET)
find_package(small_gicp QUIET)
find_package(PCL REQUIRED)
find_package(OpenMP)

set(dependencies
  rclcpp
  rclcpp_components
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
)

if(fast_gicp_FOUND)
  list(APPEND dependencies fast_gicp)
  add_definitions(-DHAS_FAST_GICP)
  message(STATUS "fast_gicp found")
else()
  message(STATUS "fast_gicp not found, FAST_GICP/FAST_VGICP disabled")
endif()

if(small_gicp_FOUND)
  list(APPEND dependencies small_gicp)
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
endif()

add_library(scanmatcher_component SHARED
src/scanmatcher_component.cpp
)

target_compile_definitions(scanmatcher_component PRIVATE "GS_SM_BUILDING_DLL")

ament_target_dependencies(scanmatcher_component ${dependencies})

add_executable(scanmatcher_node
src/scanmatcher_node.cpp
)

target_link_libraries(scanmatcher_node
scanmatcher_component ${PCL_LIBRARIES})

ament_target_dependencies(scanmatcher_node
  rclcpp)

if(small_gicp_FOUND)
  add_executable(small_gicp_odom_node
  src/small_gicp_odom_node.cpp
  )
  ament_target_dependencies(small_gicp_odom_node
    rclcpp sensor_msgs nav_msgs tf2_ros pcl_conversions small_gicp)
  target_link_libraries(small_gicp_odom_node ${PCL_LIBRARIES})
endif()

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

set(INSTALL_TARGETS scanmatcher_node)
if(small_gicp_FOUND)
  list(APPEND INSTALL_TARGETS small_gicp_odom_node)
endif()
install(TARGETS ${INSTALL_TARGETS}
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_package()
