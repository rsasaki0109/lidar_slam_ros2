# ROS-free build of the mapping core: the deterministic backend (GraphSlamApplication and the
# libraries it stands on) and the RKO-LIO frontend core, with plain CMake and no ROS 2 install.
#
#   cmake -S lidarslam_core -B build-core -DCMAKE_BUILD_TYPE=Release
#   cmake --build build-core -j && ctest --test-dir build-core
#
# The ROS 2 packages (graph_based_slam, rko_lio's ROS wrapper, ...) are unchanged and remain the
# ROS option. This directory carries COLCON_IGNORE so a colcon workspace does not pick it up.
# Target definitions mirror graph_based_slam/CMakeLists.txt; test_lidarslam_core_build.py keeps
# the two in step (same core sources, every ROS-free gtest registered here).
cmake_minimum_required(VERSION 3.22)
project(lidarslam_core LANGUAGES C CXX)

option(LIDARSLAM_CORE_BUILD_TESTS "Build the ROS-free unit tests" ON)
option(LIDARSLAM_CORE_BUILD_FRONTEND "Build the RKO-LIO frontend core" ON)
option(LIDARSLAM_CORE_ENABLE_3D_BBS "Build the optional 3D-BBS loop verifier" ON)

get_filename_component(LIDARSLAM_REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)
set(GBS_ROOT "${LIDARSLAM_REPO_ROOT}/graph_based_slam")
set(NDT_OMP_ROOT "${LIDARSLAM_REPO_ROOT}/Thirdparty/ndt_omp_ros2")
set(BBS3D_ROOT "${LIDARSLAM_REPO_ROOT}/Thirdparty/3d_bbs")
set(RKO_LIO_ROOT "${LIDARSLAM_REPO_ROOT}/Thirdparty/rko_lio")

# Same language level and flags as graph_based_slam/CMakeLists.txt.
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
set(CMAKE_CXX_FLAGS "-O2 -g ${CMAKE_CXX_FLAGS}")
add_compile_definitions(G2O_USE_VENDORED_CERES)
add_link_options(-Wl,--no-undefined)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenMP REQUIRED COMPONENTS C CXX)
find_package(zstd QUIET)

# g2o: a CMake package when one is installed, else the Ubuntu/Debian libraries (libg2o-dev).
find_package(g2o QUIET COMPONENTS g2o::core g2o::types_slam3d g2o::solver_eigen)
if(NOT g2o_FOUND)
  find_path(G2O_INCLUDE_DIR g2o/core/sparse_optimizer.h PATHS /usr/include /usr/local/include)
  find_library(G2O_CORE_LIB g2o_core)
  find_library(G2O_TYPES_SLAM3D_LIB g2o_types_slam3d)
  find_library(G2O_SOLVER_EIGEN_LIB g2o_solver_eigen)
  find_library(G2O_STUFF_LIB g2o_stuff)
  if(NOT (G2O_INCLUDE_DIR AND G2O_CORE_LIB AND G2O_TYPES_SLAM3D_LIB AND G2O_SOLVER_EIGEN_LIB
          AND G2O_STUFF_LIB))
    message(FATAL_ERROR "g2o not found (Ubuntu/Debian: apt install libg2o-dev)")
  endif()
  foreach(_tgt core types_slam3d solver_eigen stuff)
    string(TOUPPER "${_tgt}" _TGT_UPPER)
    if(NOT TARGET g2o::${_tgt})
      add_library(g2o::${_tgt} SHARED IMPORTED)
      set_target_properties(g2o::${_tgt} PROPERTIES
        IMPORTED_LOCATION "${G2O_${_TGT_UPPER}_LIB}"
        INTERFACE_INCLUDE_DIRECTORIES "${G2O_INCLUDE_DIR}")
    endif()
  endforeach()
endif()

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")

# --- Registration interfaces and NDT-OMP (the ROS packages lidarslam_plugin_interfaces and
# ndt_omp_ros2 without their ament wrappers).
add_library(lidarslam_plugin_interfaces INTERFACE)
target_include_directories(lidarslam_plugin_interfaces INTERFACE
  "${LIDARSLAM_REPO_ROOT}/lidarslam_plugin_interfaces/include" ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS})

add_library(ndt_omp STATIC
  ${NDT_OMP_ROOT}/src/pclomp/voxel_grid_covariance_omp.cpp
  ${NDT_OMP_ROOT}/src/pclomp/ndt_omp.cpp
  ${NDT_OMP_ROOT}/src/pclomp/gicp_omp.cpp)
set_target_properties(ndt_omp PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(ndt_omp PUBLIC ${NDT_OMP_ROOT}/include ${PCL_INCLUDE_DIRS})
target_link_libraries(ndt_omp PUBLIC ${PCL_LIBRARIES} OpenMP::OpenMP_CXX)

# --- Backend core (same definitions as graph_based_slam/CMakeLists.txt).
set(GBS_HAVE_3D_BBS FALSE)
if(LIDARSLAM_CORE_ENABLE_3D_BBS AND EXISTS "${BBS3D_ROOT}/bbs3d/include/cpu_bbs3d/bbs3d.hpp")
  add_library(graph_based_slam_3d_bbs_vendor STATIC
    ${BBS3D_ROOT}/bbs3d/src/cpu_bbs3d/bbs3d.cpp
    ${BBS3D_ROOT}/bbs3d/src/cpu_bbs3d/voxelmaps.cpp
    ${BBS3D_ROOT}/bbs3d/src/cpu_bbs3d/voxelmaps_io.cpp)
  target_include_directories(graph_based_slam_3d_bbs_vendor PUBLIC
    ${BBS3D_ROOT}/bbs3d/include ${EIGEN3_INCLUDE_DIR})
  target_link_libraries(graph_based_slam_3d_bbs_vendor PUBLIC OpenMP::OpenMP_CXX)
  set(GBS_HAVE_3D_BBS TRUE)
endif()

add_library(graph_based_slam_small_gicp STATIC
  ${GBS_ROOT}/third_party/small_gicp/src/small_gicp/registration/registration.cpp
  ${GBS_ROOT}/third_party/small_gicp/src/small_gicp/registration/registration_helper.cpp)
set_target_properties(graph_based_slam_small_gicp PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_features(graph_based_slam_small_gicp PUBLIC cxx_std_17)
target_include_directories(graph_based_slam_small_gicp SYSTEM PUBLIC
  ${GBS_ROOT}/third_party/small_gicp/include ${EIGEN3_INCLUDE_DIR})
target_link_libraries(graph_based_slam_small_gicp PUBLIC OpenMP::OpenMP_CXX)
add_library(small_gicp::small_gicp ALIAS graph_based_slam_small_gicp)

add_library(graph_slam_application STATIC ${GBS_ROOT}/src/graph_slam_application.cpp)
set_target_properties(graph_slam_application PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(graph_slam_application PUBLIC
  ${GBS_ROOT}/include ${EIGEN3_INCLUDE_DIR} ${PCL_INCLUDE_DIRS})
target_link_libraries(graph_slam_application
  ndt_omp lidarslam_plugin_interfaces ${PCL_LIBRARIES}
  g2o::core g2o::types_slam3d g2o::solver_eigen g2o::stuff)
target_compile_definitions(graph_slam_application PUBLIC GRAPH_BASED_SLAM_HAVE_SMALL_GICP)
target_link_libraries(graph_slam_application small_gicp::small_gicp)

add_library(graph_slam_map_point STATIC ${GBS_ROOT}/src/map_point.cpp)
set_target_properties(graph_slam_map_point PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_features(graph_slam_map_point PRIVATE cxx_std_17)
target_include_directories(graph_slam_map_point PUBLIC
  ${GBS_ROOT}/include ${EIGEN3_INCLUDE_DIR} ${PCL_INCLUDE_DIRS})
target_link_libraries(graph_slam_map_point ${PCL_LIBRARIES})

add_library(graph_slam_io_adapters STATIC ${GBS_ROOT}/src/filesystem_io_ports.cpp)
set_target_properties(graph_slam_io_adapters PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_features(graph_slam_io_adapters PRIVATE cxx_std_17)
target_include_directories(graph_slam_io_adapters PUBLIC
  ${GBS_ROOT}/include PRIVATE ${GBS_ROOT}/src ${PCL_INCLUDE_DIRS})
target_link_libraries(graph_slam_io_adapters ${PCL_LIBRARIES})

# --- RKO-LIO frontend core (Thirdparty/rko_lio with its ROS wrapper switched off).
if(LIDARSLAM_CORE_BUILD_FRONTEND)
  set(RKO_LIO_BUILD_ROS OFF CACHE BOOL "" FORCE)
  set(RKO_LIO_FETCH_CONTENT_DEPS OFF CACHE BOOL "" FORCE)
  # Eigen, TBB and robin-map come from the system (libeigen3-dev, libtbb-dev, robin-map-dev);
  # Sophus has no Ubuntu package, so it is fetched with RKO-LIO's own recipe.
  find_package(Sophus CONFIG QUIET)
  if(NOT Sophus_FOUND)
    include(FetchContent)
    set(RKO_LIO_FETCHCONTENT_COMMON_FLAGS OVERRIDE_FIND_PACKAGE SYSTEM EXCLUDE_FROM_ALL)
    include(${RKO_LIO_ROOT}/cmake/mock_find_package.cmake)
    include(${RKO_LIO_ROOT}/cmake/dependencies/sophus/sophus.cmake)
  endif()
  set(_lidarslam_core_build_testing "${BUILD_TESTING}")
  set(BUILD_TESTING OFF)
  add_subdirectory(${RKO_LIO_ROOT} rko_lio EXCLUDE_FROM_ALL)
  set(BUILD_TESTING "${_lidarslam_core_build_testing}")
  add_custom_target(lidarslam_core_frontend ALL DEPENDS rko_lio.core)
endif()

# --- Unit tests: every gtest of graph_based_slam that needs no ROS message or client library.
if(LIDARSLAM_CORE_BUILD_TESTS)
  enable_testing()
  find_package(GTest REQUIRED)
  set(LIDARSLAM_CORE_GTESTS
    test_gnss_geometry test_gnss_anchor_matching test_gnss_origin_accumulator
    test_adjacent_edge_auto_scale test_scan_context test_solid_descriptor
    test_submap_bev_descriptor test_bev_mutual_visibility test_loop_edge_robustifier
    test_loop_search_schedule test_serialized_work_drain test_background_work_drain
    test_triangle_descriptor test_triangle_descriptor_database test_submap_creation
    test_gnss_alignment test_candidate_aggregator test_loop_verifier test_map_saver
    test_voxel_grid_overflow test_map_point test_graph_slam_application test_external_io_ports
    test_pose_graph_optimization test_se3_lie test_point_cluster test_scatter_eigen_cost
    test_plane_ba test_synthetic_degeneracy_fixtures test_localizability_analysis
    test_degeneracy_aware_solve test_persistent_weak_direction test_rko_multiscan_observability
    test_selective_visual_fusion test_radar_ego_velocity test_radar_continuous_fusion
    test_intensity_profile test_odometry_covariance_localizability
    test_degeneracy_diagnostics_csv test_degeneracy_report_summary test_map_refiner
    test_hba_pyramid test_plane_feature_association test_adaptive_voxel_plane_extractor
    test_map_quality_metrics test_registration_determinism test_dynamic_object_filter
    test_planar_map_filter test_multisession_join)
  # graph_slam_application calls ThreeDBBSLoopVerifier, whose source each consumer compiles. The
  # ROS tests compile it without GRAPH_BASED_SLAM_HAVE_3D_BBS (the verifier reports itself
  # unavailable); the tests here link the same build of it.
  add_library(graph_slam_test_loop_verifier STATIC ${GBS_ROOT}/src/three_d_bbs_loop_verifier.cpp)
  target_include_directories(graph_slam_test_loop_verifier PUBLIC
    ${GBS_ROOT}/include ${EIGEN3_INCLUDE_DIR} ${PCL_INCLUDE_DIRS})
  target_link_libraries(graph_slam_test_loop_verifier ${PCL_LIBRARIES})
  foreach(_test IN LISTS LIDARSLAM_CORE_GTESTS)
    add_executable(${_test} ${GBS_ROOT}/test/${_test}.cpp)
    target_compile_features(${_test} PRIVATE cxx_std_17)
    target_include_directories(${_test} PRIVATE ${GBS_ROOT}/include ${GBS_ROOT}/src ${RKO_LIO_ROOT}
      ${EIGEN3_INCLUDE_DIR} ${PCL_INCLUDE_DIRS})
    target_link_libraries(${_test} GTest::gtest_main graph_slam_application
      graph_slam_test_loop_verifier graph_slam_io_adapters graph_slam_map_point ndt_omp
      lidarslam_plugin_interfaces
      ${PCL_LIBRARIES} g2o::core g2o::solver_eigen g2o::stuff g2o::types_slam3d)
    add_test(NAME ${_test} COMMAND ${_test})
    set_tests_properties(${_test} PROPERTIES TIMEOUT 300)
  endforeach()
  if(GBS_HAVE_3D_BBS)
    target_link_libraries(test_graph_slam_application graph_based_slam_3d_bbs_vendor)
  endif()
endif()
