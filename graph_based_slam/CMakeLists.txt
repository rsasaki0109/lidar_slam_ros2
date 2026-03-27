cmake_minimum_required(VERSION 3.5)
project(graph_based_slam)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  #add_compile_options(-Wall -Wextra -Wpedantic)
  #add_compile_options(-Wextra -Wpedantic)
endif()

set(CMAKE_CXX_FLAGS "-O2 -g ${CMAKE_CXX_FLAGS}")
add_compile_definitions(G2O_USE_VENDORED_CERES)

# Find missing symbols during link time
add_link_options(-Wl,--no-undefined)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(lidarslam_msgs REQUIRED)
find_package(ndt_omp_ros2 REQUIRED)
find_package(PCL REQUIRED)
find_package(g2o QUIET
  COMPONENTS
    g2o::core
    g2o::types_slam3d
    g2o::solver_eigen
)
if(NOT g2o_FOUND)
  # Fallback: find g2o libraries directly (Ubuntu/Debian packages without cmake config)
  find_path(G2O_INCLUDE_DIR g2o/core/sparse_optimizer.h PATHS /usr/include /usr/local/include)
  find_library(G2O_CORE_LIB g2o_core)
  find_library(G2O_TYPES_SLAM3D_LIB g2o_types_slam3d)
  find_library(G2O_SOLVER_EIGEN_LIB g2o_solver_eigen)
  find_library(G2O_STUFF_LIB g2o_stuff)
  if(G2O_INCLUDE_DIR AND G2O_CORE_LIB AND G2O_TYPES_SLAM3D_LIB AND G2O_SOLVER_EIGEN_LIB)
    message(STATUS "Found g2o via fallback: ${G2O_CORE_LIB}")
    # Create imported targets to match CMake config mode
    foreach(_tgt core types_slam3d solver_eigen stuff)
      string(TOUPPER "${_tgt}" _TGT_UPPER)
      if(NOT TARGET g2o::${_tgt})
        add_library(g2o::${_tgt} SHARED IMPORTED)
        set_target_properties(g2o::${_tgt} PROPERTIES
          IMPORTED_LOCATION "${G2O_${_TGT_UPPER}_LIB}"
          INTERFACE_INCLUDE_DIRECTORIES "${G2O_INCLUDE_DIR}"
        )
      endif()
    endforeach()
    set(g2o_FOUND TRUE)
  else()
    message(FATAL_ERROR "Could not find g2o libraries")
  endif()
endif()
find_package(OpenMP)

option(
  GRAPH_BASED_SLAM_ENABLE_3D_BBS
  "Build optional 3D-BBS loop-closure verifier support"
  ON
)
set(
  GRAPH_BASED_SLAM_3D_BBS_ROOT
  "${CMAKE_CURRENT_SOURCE_DIR}/../Thirdparty/3d_bbs"
  CACHE PATH
  "Path to the vendored 3D-BBS source tree"
)

set(GRAPH_BASED_SLAM_HAVE_3D_BBS FALSE)
if(GRAPH_BASED_SLAM_ENABLE_3D_BBS AND
  EXISTS "${GRAPH_BASED_SLAM_3D_BBS_ROOT}/bbs3d/include/cpu_bbs3d/bbs3d.hpp")
  add_library(graph_based_slam_3d_bbs_vendor STATIC
    ${GRAPH_BASED_SLAM_3D_BBS_ROOT}/bbs3d/src/cpu_bbs3d/bbs3d.cpp
    ${GRAPH_BASED_SLAM_3D_BBS_ROOT}/bbs3d/src/cpu_bbs3d/voxelmaps.cpp
    ${GRAPH_BASED_SLAM_3D_BBS_ROOT}/bbs3d/src/cpu_bbs3d/voxelmaps_io.cpp
  )
  target_include_directories(graph_based_slam_3d_bbs_vendor PUBLIC
    ${GRAPH_BASED_SLAM_3D_BBS_ROOT}/bbs3d/include
    ${EIGEN3_INCLUDE_DIR}
  )
  if(OPENMP_FOUND)
    target_link_libraries(graph_based_slam_3d_bbs_vendor PUBLIC OpenMP::OpenMP_CXX)
  endif()
  set(GRAPH_BASED_SLAM_HAVE_3D_BBS TRUE)
  message(STATUS "Building graph_based_slam with optional 3D-BBS support")
else()
  message(STATUS "Building graph_based_slam without optional 3D-BBS support")
endif()

if(OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_lint_auto_find_test_dependencies()
  ament_add_gtest(
    test_gnss_weighting
    test/test_gnss_weighting.cpp
  )
  if(TARGET test_gnss_weighting)
    target_include_directories(test_gnss_weighting PRIVATE include)
    ament_target_dependencies(test_gnss_weighting sensor_msgs)
  endif()
  ament_add_gtest(
    test_scan_context
    test/test_scan_context.cpp
  )
  if(TARGET test_scan_context)
    target_include_directories(test_scan_context PRIVATE include ${EIGEN3_INCLUDE_DIR})
  endif()
  ament_add_gtest(
    test_solid_descriptor
    test/test_solid_descriptor.cpp
  )
  if(TARGET test_solid_descriptor)
    target_include_directories(test_solid_descriptor PRIVATE include ${EIGEN3_INCLUDE_DIR})
    target_link_libraries(test_solid_descriptor ${PCL_LIBRARIES})
  endif()
  ament_add_gtest(
    test_submap_bev_descriptor
    test/test_submap_bev_descriptor.cpp
  )
  if(TARGET test_submap_bev_descriptor)
    target_include_directories(test_submap_bev_descriptor PRIVATE include ${EIGEN3_INCLUDE_DIR})
  endif()
  ament_add_gtest(
    test_dynamic_object_filter
    test/test_dynamic_object_filter.cpp
  )
  if(TARGET test_dynamic_object_filter)
    target_include_directories(
      test_dynamic_object_filter PRIVATE include ${EIGEN3_INCLUDE_DIR}
    )
  endif()
  ament_add_pytest_test(
    test_autoware_map_tools
    test/test_autoware_map_tools.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_navsatfix_covariance_inspector
    test/test_navsatfix_covariance_inspector.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_applanix_gsof50_inspector
    test/test_applanix_gsof50_inspector.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_applanix_navsatfix_bridge
    test/test_applanix_navsatfix_bridge.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_applanix_imu_bridge
    test/test_applanix_imu_bridge.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_static_transform_extractor
    test/test_static_transform_extractor.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_applanix_gsof49_reference_extractor
    test/test_applanix_gsof49_reference_extractor.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_benchmark_reporting_tools
    test/test_benchmark_reporting_tools.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_rko_lio_benchmark_tools
    test/test_rko_lio_benchmark_tools.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_docs_entrypoints
    test/test_docs_entrypoints.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_aligned_trajectory_metrics
    test/test_aligned_trajectory_metrics.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_v2_beta_readiness_report
    test/test_v2_beta_readiness_report.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_stress_validation_report
    test/test_stress_validation_report.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_place_recognition_report
    test/test_place_recognition_report.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_packet_imu_deskew_validation_report
    test/test_packet_imu_deskew_validation_report.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_dynamic_object_filter_report
    test/test_dynamic_object_filter_report.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_dynamic_object_filter_validation_report
    test/test_dynamic_object_filter_validation_report.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_submission_bundle_tools
    test/test_submission_bundle_tools.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_map_authoring_report
    test/test_map_authoring_report.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_classic_path_report
    test/test_classic_path_report.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_odom_prior_validation_report
    test/test_odom_prior_validation_report.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_exploration_closeout_report
    test/test_exploration_closeout_report.py
    TIMEOUT 120
  )
endif()

add_library(graph_based_slam_component SHARED
  src/graph_based_slam_component.cpp
  src/three_d_bbs_loop_verifier.cpp
)

target_compile_definitions(graph_based_slam_component PRIVATE "GS_GBS_BUILDING_DLL")
if(GRAPH_BASED_SLAM_HAVE_3D_BBS)
  target_compile_definitions(graph_based_slam_component PRIVATE GRAPH_BASED_SLAM_HAVE_3D_BBS)
  target_link_libraries(graph_based_slam_component graph_based_slam_3d_bbs_vendor)
endif()

ament_target_dependencies(graph_based_slam_component
  rclcpp
  rclcpp_components
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  tf2_sensor_msgs
  pcl_conversions
  geometry_msgs
  sensor_msgs
  nav_msgs
  std_srvs
  lidarslam_msgs
  ndt_omp_ros2
)

target_link_libraries(graph_based_slam_component
  g2o::core
  g2o::types_slam3d
  g2o::solver_eigen
  g2o::stuff
)

add_executable(graph_based_slam_node
  src/graph_based_slam_node.cpp
)

target_link_libraries(graph_based_slam_node
  graph_based_slam_component
)

ament_target_dependencies(graph_based_slam_node
  rclcpp)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)
link_directories(
  ${PCL_LIBRARY_DIRS}
)
add_definitions(${PCL_DEFINITIONS})

rclcpp_components_register_nodes(graph_based_slam_component
  "graphslam::GraphBasedSlamComponent")

ament_export_libraries(graph_based_slam_component)

install(
  DIRECTORY "include/"
  DESTINATION include
)

install(
  DIRECTORY launch param
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  graph_based_slam_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  graph_based_slam_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_package()
