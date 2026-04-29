cmake_minimum_required(VERSION 3.5)

if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

if(POLICY CMP0144)
  cmake_policy(SET CMP0144 NEW)
endif()

project(lidarslam)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(scanmatcher REQUIRED)
find_package(graph_based_slam REQUIRED)
find_package(lidarslam_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ndt_omp_ros2 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenMP)
find_package(g2o QUIET
  COMPONENTS
    g2o::core
    g2o::types_slam3d
    g2o::types_slam3d_addons
    g2o::solver_eigen
)
if(NOT g2o_FOUND)
  find_path(G2O_INCLUDE_DIR g2o/core/sparse_optimizer.h PATHS /usr/include /usr/local/include)
  find_library(G2O_CORE_LIB g2o_core)
  find_library(G2O_TYPES_SLAM3D_LIB g2o_types_slam3d)
  find_library(G2O_TYPES_SLAM3D_ADDONS_LIB g2o_types_slam3d_addons)
  find_library(G2O_SOLVER_EIGEN_LIB g2o_solver_eigen)
  find_library(G2O_STUFF_LIB g2o_stuff)
  if(G2O_INCLUDE_DIR AND G2O_CORE_LIB AND G2O_TYPES_SLAM3D_LIB AND G2O_SOLVER_EIGEN_LIB)
    message(STATUS "Found g2o via fallback: ${G2O_CORE_LIB}")
    foreach(_tgt core types_slam3d types_slam3d_addons solver_eigen stuff)
      string(TOUPPER "${_tgt}" _TGT_UPPER)
      if(NOT TARGET g2o::${_tgt} AND G2O_${_TGT_UPPER}_LIB)
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

set(CMAKE_CXX_FLAGS "-O2 -g ${CMAKE_CXX_FLAGS}")

if(OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)
link_directories(
  ${PCL_LIBRARY_DIRS}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_lint_auto_find_test_dependencies()
  ament_add_pytest_test(
    test_rko_lio_launch_config
    test/test_rko_lio_launch_config.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_lidarslam_launch_config
    test/test_lidarslam_launch_config.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_lo_slam_launch_config
    test/test_lo_slam_launch_config.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_small_gicp_lo_slam_launch_config
    test/test_small_gicp_lo_slam_launch_config.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_open_data_gnss_smoke_script
    test/test_open_data_gnss_smoke_script.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_open_data_applanix_velodyne_gnss_smoke_script
    test/test_open_data_applanix_velodyne_gnss_smoke_script.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_open_data_applanix_velodyne_gnss_benchmark_script
    test/test_open_data_applanix_velodyne_gnss_benchmark_script.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_open_data_packet_imu_deskew_validation_matrix_script
    test/test_open_data_packet_imu_deskew_validation_matrix_script.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_dynamic_object_filter_benchmark_script
    test/test_dynamic_object_filter_benchmark_script.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_open_data_classic_path_benchmark_suite_script
    test/test_open_data_classic_path_benchmark_suite_script.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_place_recognition_benchmark_script
    test/test_place_recognition_benchmark_script.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_rko_lio_mid360_crossval_benchmark_script
    test/test_rko_lio_mid360_crossval_benchmark_script.py
    TIMEOUT 120
  )
endif()

add_executable(lidarslam
  src/lidarslam.cpp
)

target_link_libraries(lidarslam
  ${PCL_LIBRARIES}
  g2o::core
  g2o::types_slam3d
  g2o::types_slam3d_addons
  g2o::solver_eigen
)

ament_target_dependencies(lidarslam
  rclcpp
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  nav_msgs
  pcl_conversions
  std_srvs
  scanmatcher
  graph_based_slam
  lidarslam_msgs
  ndt_omp_ros2
)

install(
  DIRECTORY launch param rviz
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  lidarslam
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
