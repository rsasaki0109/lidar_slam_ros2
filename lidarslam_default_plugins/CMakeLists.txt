cmake_minimum_required(VERSION 3.8)
project(lidarslam_default_plugins)

if(NOT CMAKE_CXX_STANDARD)
  # The public registration contract remains C++14 source-compatible, but
  # this pclomp implementation must use the same GNU C++17 mode as the
  # scanmatcher shell.  Eigen/PCL template code can take a different code
  # path when __STRICT_ANSI__ is set, which would break the legacy NDT
  # characterization at the adapter boundary.
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# scanmatcher is compiled as GNU C++17; keep the template instantiation mode
# identical for pclomp/Eigen floating-point behavior.
set(CMAKE_CXX_EXTENSIONS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenMP)
find_package(pluginlib REQUIRED)
find_package(lidarslam_plugin_interfaces REQUIRED)
find_package(ndt_omp_ros2 REQUIRED)
find_package(small_gicp QUIET)

add_library(${PROJECT_NAME} SHARED
  src/ndt_omp_registration.cpp
  src/gicp_omp_registration.cpp
)

if(small_gicp_FOUND)
  # Keep the optional Small implementation out of the NDT/GICP DSO.  The
  # scanmatcher component owns the same-TU host factories, while this target
  # is loaded only by an explicit external pluginlib selector.
  add_library(lidarslam_small_gicp_plugins SHARED
    src/small_gicp_registration.cpp
  )
  target_compile_features(lidarslam_small_gicp_plugins PUBLIC cxx_std_17)
  target_include_directories(lidarslam_small_gicp_plugins PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${PCL_INCLUDE_DIRS}
  )
  target_link_libraries(lidarslam_small_gicp_plugins
    ${PCL_LIBRARIES}
    small_gicp::small_gicp
  )
  ament_target_dependencies(lidarslam_small_gicp_plugins
    Eigen3
    pluginlib
    lidarslam_plugin_interfaces
  )
  if(OpenMP_CXX_FOUND)
    target_link_libraries(lidarslam_small_gicp_plugins OpenMP::OpenMP_CXX)
  endif()
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    # This is deliberately scoped to the Small external DSO.  It prevents
    # weak pclomp/small_gicp template definitions in the host component from
    # interposing on this plugin while leaving the NDT/GICP DSO unchanged.
    set_property(TARGET lidarslam_small_gicp_plugins APPEND_STRING PROPERTY
      LINK_FLAGS " -Wl,-Bsymbolic-functions")
  endif()
  message(STATUS "small_gicp found; isolated SMALL_GICP/SMALL_VGICP DSO enabled")
else()
  message(STATUS "small_gicp not found; SMALL_GICP/SMALL_VGICP adapters disabled")
endif()

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME}
  Eigen3
  pluginlib
  lidarslam_plugin_interfaces
  ndt_omp_ros2
)

if(PCL_DEFINITIONS)
  add_definitions(${PCL_DEFINITIONS})
endif()
if(OpenMP_CXX_FOUND)
  target_link_libraries(${PROJECT_NAME} OpenMP::OpenMP_CXX)
endif()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_ndt_omp_registration
    test/test_ndt_omp_registration.cpp
  )
  if(TARGET test_ndt_omp_registration)
    target_include_directories(test_ndt_omp_registration PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include
      ${PCL_INCLUDE_DIRS}
    )
    target_compile_features(test_ndt_omp_registration PRIVATE cxx_std_17)
    target_link_libraries(test_ndt_omp_registration
      ${PROJECT_NAME}
      ${PCL_LIBRARIES}
    )
    ament_target_dependencies(test_ndt_omp_registration
      Eigen3
      lidarslam_plugin_interfaces
      ndt_omp_ros2
    )
    if(OpenMP_CXX_FOUND)
      target_link_libraries(test_ndt_omp_registration OpenMP::OpenMP_CXX)
    endif()
  endif()

  ament_add_gtest(test_gicp_omp_registration
    test/test_gicp_omp_registration.cpp
  )
  if(TARGET test_gicp_omp_registration)
    target_include_directories(test_gicp_omp_registration PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include
      ${PCL_INCLUDE_DIRS}
    )
    target_compile_features(test_gicp_omp_registration PRIVATE cxx_std_17)
    target_link_libraries(test_gicp_omp_registration
      ${PROJECT_NAME}
      ${PCL_LIBRARIES}
    )
    ament_target_dependencies(test_gicp_omp_registration
      Eigen3
      lidarslam_plugin_interfaces
      ndt_omp_ros2
    )
    if(OpenMP_CXX_FOUND)
      target_link_libraries(test_gicp_omp_registration OpenMP::OpenMP_CXX)
    endif()
  endif()

  if(small_gicp_FOUND)
    ament_add_gtest(test_small_gicp_registration
      test/test_small_gicp_registration.cpp
    )
    if(TARGET test_small_gicp_registration)
      target_include_directories(test_small_gicp_registration PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${PCL_INCLUDE_DIRS}
      )
      target_compile_features(test_small_gicp_registration PRIVATE cxx_std_17)
      target_link_libraries(test_small_gicp_registration
        lidarslam_small_gicp_plugins
        ${PCL_LIBRARIES}
        small_gicp::small_gicp
      )
      ament_target_dependencies(test_small_gicp_registration
        Eigen3
        lidarslam_plugin_interfaces
      )
      if(OpenMP_CXX_FOUND)
        target_link_libraries(test_small_gicp_registration OpenMP::OpenMP_CXX)
      endif()
    endif()
  endif()
endif()

install(DIRECTORY include/ DESTINATION include)
set(plugin_install_targets ${PROJECT_NAME})
if(small_gicp_FOUND)
  list(APPEND plugin_install_targets lidarslam_small_gicp_plugins)
endif()
install(TARGETS ${plugin_install_targets}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
if(small_gicp_FOUND)
  ament_export_libraries(lidarslam_small_gicp_plugins)
endif()
ament_export_dependencies(Eigen3 PCL pluginlib lidarslam_plugin_interfaces ndt_omp_ros2)
if(small_gicp_FOUND)
  ament_export_dependencies(small_gicp)
  pluginlib_export_plugin_description_file(
    lidarslam_plugin_interfaces
    registration_plugins.xml
  )
  pluginlib_export_plugin_description_file(
    lidarslam_plugin_interfaces
    registration_plugins_small.xml
  )
else()
  pluginlib_export_plugin_description_file(
    lidarslam_plugin_interfaces
    registration_plugins.xml
  )
endif()
ament_package()
