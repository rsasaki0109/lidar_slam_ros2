cmake_minimum_required(VERSION 3.8)
project(lidarslam_plugin_interfaces)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME} INTERFACE Eigen3::Eigen ${PCL_LIBRARIES})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_registration_contract test/test_registration_contract.cpp)
  target_link_libraries(test_registration_contract ${PROJECT_NAME})
endif()

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME})

# Registration plugins are expected to build outside this source tree. Export
# the contract-manifest helper from the public SDK package so a consumer only
# needs find_package(lidarslam_plugin_interfaces); no sibling checkout path is
# required.
set(_registration_contract_cmake_dir
  "${CMAKE_CURRENT_SOURCE_DIR}/../lidarslam_registration_loader/cmake")
set(_registration_contract_helper
  "${_registration_contract_cmake_dir}/registration_contract.cmake")
set(_registration_contract_generator
  "${_registration_contract_cmake_dir}/generate_registration_contract_manifest.cmake")
install(FILES "${_registration_contract_generator}"
  DESTINATION share/${PROJECT_NAME}/cmake)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 PCL)
ament_package(CONFIG_EXTRAS "${_registration_contract_helper}")
