cmake_minimum_required(VERSION 3.8)
project(lidarslam_registration_loader)

option(LIDARSLAM_BUILD_REGISTRATION_STRESS
  "Build the bounded C++14 registration session stress executable" OFF)
option(LIDARSLAM_REGISTRATION_STRESS_TSAN
  "Build the registration session stress executable with ThreadSanitizer" OFF)

if(NOT CMAKE_CXX_STANDARD)
  # Jazzy's pluginlib/ament_index implementation uses std::filesystem and
  # std::optional.  This is shell-only; the installed RegistrationPlugin
  # contract remains source-compatible with C++14.
  set(CMAKE_CXX_STANDARD 17)
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
  src/registration_plugin_loader.cpp
  src/registration_contract_manifest.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
ament_target_dependencies(${PROJECT_NAME}
  pluginlib
  lidarslam_plugin_interfaces
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_registration_plugin_loader
    test/test_registration_plugin_loader.cpp
  )
  if(TARGET test_registration_plugin_loader)
    target_compile_features(test_registration_plugin_loader PRIVATE cxx_std_17)
    target_link_libraries(test_registration_plugin_loader ${PROJECT_NAME})
    ament_target_dependencies(test_registration_plugin_loader
      lidarslam_plugin_interfaces
    )
  endif()
endif()

if(LIDARSLAM_BUILD_REGISTRATION_STRESS)
  find_package(Threads REQUIRED)
  include(CTest)
  file(SHA256 "${CMAKE_CURRENT_SOURCE_DIR}/../tools/registration_plugin_session_stress.cpp"
    LIDARSLAM_REGISTRATION_STRESS_SOURCE_SHA256)
  add_executable(registration_plugin_session_stress
    ../tools/registration_plugin_session_stress.cpp)
  # The installed public interface remains C++14.  This executable links the
  # C++17 ROS/pluginlib shell only as an implementation library.
  target_compile_features(registration_plugin_session_stress PRIVATE cxx_std_14)
  target_compile_definitions(registration_plugin_session_stress PRIVATE
    LIDARSLAM_REGISTRATION_STRESS_SOURCE_SHA256="${LIDARSLAM_REGISTRATION_STRESS_SOURCE_SHA256}")
  target_link_libraries(registration_plugin_session_stress
    ${PROJECT_NAME} Threads::Threads)
  ament_target_dependencies(registration_plugin_session_stress
    lidarslam_plugin_interfaces)
  if(LIDARSLAM_REGISTRATION_STRESS_TSAN)
    if(NOT CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
      message(FATAL_ERROR "ThreadSanitizer stress mode requires GCC or Clang")
    endif()
    # Instrument both the harness and the actual session implementation;
    # instrumenting only the consumer would miss races in the loader shell.
    target_compile_options(${PROJECT_NAME} PRIVATE
      -fsanitize=thread -fno-omit-frame-pointer)
    target_link_libraries(${PROJECT_NAME} -fsanitize=thread)
    target_compile_options(registration_plugin_session_stress PRIVATE
      -fsanitize=thread -fno-omit-frame-pointer)
    target_link_libraries(registration_plugin_session_stress -fsanitize=thread)
  endif()
  add_test(NAME registration_plugin_session_stress
    COMMAND registration_plugin_session_stress
      --receipt
      "${CMAKE_CURRENT_BINARY_DIR}/registration_plugin_session_stress.receipt.json")
  set_tests_properties(registration_plugin_session_stress PROPERTIES
    LABELS "registration-plugin;stress"
    TIMEOUT 30)
endif()

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(pluginlib lidarslam_plugin_interfaces)
ament_package()
