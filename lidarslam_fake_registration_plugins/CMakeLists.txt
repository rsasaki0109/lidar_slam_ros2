cmake_minimum_required(VERSION 3.8)
project(lidarslam_fake_registration_plugins)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(lidarslam_plugin_interfaces REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/fake_registration_plugins.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_14)
ament_target_dependencies(${PROJECT_NAME}
  pluginlib
  lidarslam_plugin_interfaces
)

pluginlib_export_plugin_description_file(
  lidarslam_plugin_interfaces
  registration_plugins.xml
)
pluginlib_export_plugin_description_file(
  lidarslam_plugin_interfaces
  missing_registration_plugins.xml
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

set(_registration_fake_schema_sha256
  "4d2d0f7f87d168fa5a04b22d1f3fbfc468c3576ae1e4f529dc8b6a7fca8a7c30")
foreach(_registration_fake_class
    "lidarslam_fake_registration_plugins/Identity"
    "lidarslam_fake_registration_plugins/BadApi"
    "lidarslam_fake_registration_plugins/BadMetadata"
    "lidarslam_fake_registration_plugins/Rejecting"
    "lidarslam_fake_registration_plugins/Unlicensed"
    "lidarslam_fake_registration_plugins/ConstructorProbe")
  lidarslam_install_registration_contract(${PROJECT_NAME} registration_plugins.xml
    "${_registration_fake_class}" 129 64 1 1 0
    "lidarslam.registration.generic.v1" 1 "${_registration_fake_schema_sha256}")
endforeach()
lidarslam_install_registration_contract(${PROJECT_NAME} registration_plugins.xml
  "lidarslam_fake_registration_plugins/NoGuess" 128 64 1 1 0
  "lidarslam.registration.generic.v1" 1 "${_registration_fake_schema_sha256}")

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(pluginlib lidarslam_plugin_interfaces)
ament_export_libraries(${PROJECT_NAME})
ament_package()
