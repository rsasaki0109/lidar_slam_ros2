cmake_minimum_required(VERSION 3.8)
project(lidarslam_registration_plugin_template)

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
  src/template_registration.cpp
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

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(lidarslam_registration_loader REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_registration_plugin_template
    test/test_template_registration.cpp
  )
  if(TARGET test_registration_plugin_template)
    target_compile_features(test_registration_plugin_template PRIVATE cxx_std_14)
    target_link_libraries(test_registration_plugin_template ${PROJECT_NAME})
    ament_target_dependencies(test_registration_plugin_template
      lidarslam_plugin_interfaces
    )
  endif()

  ament_add_gtest(test_registration_plugin_template_loader
    test/test_template_plugin_loader.cpp
  )
  if(TARGET test_registration_plugin_template_loader)
    target_compile_features(test_registration_plugin_template_loader PRIVATE cxx_std_17)
    ament_target_dependencies(test_registration_plugin_template_loader
      lidarslam_plugin_interfaces
      lidarslam_registration_loader
    )
  endif()
endif()

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(pluginlib lidarslam_plugin_interfaces)
ament_export_libraries(${PROJECT_NAME})
ament_package()
