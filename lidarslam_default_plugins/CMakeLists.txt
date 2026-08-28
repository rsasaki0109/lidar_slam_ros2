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
find_package(fast_gicp QUIET)
find_package(small_gicp QUIET)

add_library(${PROJECT_NAME} SHARED
  src/ndt_omp_registration.cpp
  src/gicp_omp_registration.cpp
)

if(fast_gicp_FOUND)
  # FAST is deliberately isolated from the pclomp/small DSOs.  The optional
  # adapter is explicit opt-in; its absence must not alter the default NDT or
  # legacy GICP path.
  add_library(lidarslam_fast_gicp_plugins SHARED
    src/fast_gicp_registration.cpp
  )
  target_compile_features(lidarslam_fast_gicp_plugins PUBLIC cxx_std_17)
  target_include_directories(lidarslam_fast_gicp_plugins PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${PCL_INCLUDE_DIRS}
  )
  target_link_libraries(lidarslam_fast_gicp_plugins ${PCL_LIBRARIES})
  ament_target_dependencies(lidarslam_fast_gicp_plugins
    Eigen3
    pluginlib
    lidarslam_plugin_interfaces
    fast_gicp
  )
  if(OpenMP_CXX_FOUND)
    target_link_libraries(lidarslam_fast_gicp_plugins OpenMP::OpenMP_CXX)
  endif()
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    # Keep fast_gicp template symbols in the optional DSO boundary.  This
    # prevents accidental interposition with the legacy pclomp/small paths.
    set_property(TARGET lidarslam_fast_gicp_plugins APPEND_STRING PROPERTY
      LINK_FLAGS " -Wl,-Bsymbolic-functions")
  endif()
  message(STATUS "fast_gicp found; isolated FAST_GICP/FAST_VGICP DSO enabled")
else()
  message(STATUS "fast_gicp not found; FAST_GICP/FAST_VGICP adapters disabled")
endif()

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

set(_registration_ndt_required_bits 159)
set(_registration_gicp_required_bits 137)
set(_registration_deterministic_bit 64)
set(_registration_ndt_schema_sha256
  "0b38bd0c73d94d9d01fb6c656aec6210f434b65f8cf88f1aea9e61e433c9ba44")
set(_registration_gicp_schema_sha256
  "6c35ace8983dcfdc86d609ed9347c4743638b3c51665bec43b9f5a59b25b6f19")
set(_registration_fast_schema_sha256
  "af44393771764fb1eff20d75743995995134bd35911b93d49839af14406b7bb5")
set(_registration_small_schema_sha256
  "c5d1bce8fbe885537121606949bc1c0e7ae5dcd5e5279ea9e28581f83ff995f5")

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
    TIMEOUT 300
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

  if(fast_gicp_FOUND)
    ament_add_gtest(test_fast_gicp_registration
      test/test_fast_gicp_registration.cpp
    )
    if(TARGET test_fast_gicp_registration)
      target_include_directories(test_fast_gicp_registration PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${PCL_INCLUDE_DIRS}
      )
      target_compile_features(test_fast_gicp_registration PRIVATE cxx_std_17)
      target_link_libraries(test_fast_gicp_registration
        lidarslam_fast_gicp_plugins
        ${PCL_LIBRARIES}
      )
      ament_target_dependencies(test_fast_gicp_registration
        Eigen3
        lidarslam_plugin_interfaces
        fast_gicp
      )
    endif()
  endif()
endif()

install(DIRECTORY include/ DESTINATION include)
set(plugin_install_targets ${PROJECT_NAME})
if(fast_gicp_FOUND)
  list(APPEND plugin_install_targets lidarslam_fast_gicp_plugins)
endif()
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
if(fast_gicp_FOUND)
  ament_export_libraries(lidarslam_fast_gicp_plugins)
endif()
if(small_gicp_FOUND)
  ament_export_libraries(lidarslam_small_gicp_plugins)
endif()
ament_export_dependencies(Eigen3 PCL pluginlib lidarslam_plugin_interfaces ndt_omp_ros2)
if(fast_gicp_FOUND)
  ament_export_dependencies(fast_gicp)
  pluginlib_export_plugin_description_file(
    lidarslam_plugin_interfaces
    registration_plugins_fast.xml
  )
endif()
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

# These run from the generated install script after each target and XML have
# been installed.  The DSO digest therefore binds the exact post-link bytes;
# a relink or toolchain change produces a different sidecar and is rejected
# before pluginlib construction.
lidarslam_install_registration_contract(${PROJECT_NAME} registration_plugins.xml
  "lidarslam_default_plugins/NdtOmp" ${_registration_ndt_required_bits}
  ${_registration_deterministic_bit} 1 1 0
  "lidarslam.registration.ndt.v1" 1 "${_registration_ndt_schema_sha256}")
lidarslam_install_registration_contract(${PROJECT_NAME} registration_plugins.xml
  "lidarslam_default_plugins/GicpOmp" ${_registration_gicp_required_bits} 0 0 2 0
  "lidarslam.registration.gicp.v1" 1 "${_registration_gicp_schema_sha256}")
if(fast_gicp_FOUND)
  lidarslam_install_registration_contract(lidarslam_fast_gicp_plugins
    registration_plugins_fast.xml "lidarslam_default_plugins/FastGicp"
    ${_registration_gicp_required_bits} 0 0 2 0
    "lidarslam.registration.fast_gicp.v1" 1 "${_registration_fast_schema_sha256}")
  lidarslam_install_registration_contract(lidarslam_fast_gicp_plugins
    registration_plugins_fast.xml "lidarslam_default_plugins/FastVGicp"
    ${_registration_gicp_required_bits} 0 0 2 0
    "lidarslam.registration.fast_gicp.v1" 1 "${_registration_fast_schema_sha256}")
endif()
if(small_gicp_FOUND)
  lidarslam_install_registration_contract(lidarslam_small_gicp_plugins
    registration_plugins_small.xml "lidarslam_default_plugins/SmallGicpPcl"
    ${_registration_gicp_required_bits} ${_registration_deterministic_bit} 0 2 0
    "lidarslam.registration.small_gicp.v1" 1 "${_registration_small_schema_sha256}")
  lidarslam_install_registration_contract(lidarslam_small_gicp_plugins
    registration_plugins_small.xml "lidarslam_default_plugins/SmallVGicpPcl"
    ${_registration_gicp_required_bits} ${_registration_deterministic_bit} 0 2 0
    "lidarslam.registration.small_gicp.v1" 1 "${_registration_small_schema_sha256}")
endif()
ament_package()
