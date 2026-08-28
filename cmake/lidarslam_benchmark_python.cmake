# Canonical installation projection for the benchmark/evidence Python tools.
# The source implementation remains in one repository scripts/ directory.

function(lidarslam_install_benchmark_python_surface source_root project_name)
  if(NOT IS_DIRECTORY "${source_root}/scripts")
    message(FATAL_ERROR "benchmark Python source tree is missing: ${source_root}/scripts")
  endif()
  if(NOT EXISTS "${source_root}/lidarslam_benchmark_tools/__init__.py")
    message(FATAL_ERROR "benchmark Python package metadata is missing")
  endif()
  if(NOT EXISTS "${source_root}/lidarslam_benchmark_tools/gaussian_splatting/__init__.py")
    message(FATAL_ERROR "Gaussian-splatting Python package metadata is missing")
  endif()
  if(NOT EXISTS "${source_root}/lidarslam_benchmark_tools/lidarslam_tools/__init__.py")
    message(FATAL_ERROR "shared benchmark Python package metadata is missing")
  endif()
  if(NOT EXISTS "${source_root}/scripts/lidarslam_benchmark_tool")
    message(FATAL_ERROR "benchmark Python CLI wrapper is missing")
  endif()

  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  set(_python_install_dir
      "lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages")

  install(
    FILES "${source_root}/lidarslam_benchmark_tools/__init__.py"
    DESTINATION "${_python_install_dir}/lidarslam_benchmark_tools")
  install(
    FILES "${source_root}/lidarslam_benchmark_tools/gaussian_splatting/__init__.py"
    DESTINATION "${_python_install_dir}/lidarslam_benchmark_tools/gaussian_splatting")
  install(
    FILES "${source_root}/lidarslam_benchmark_tools/lidarslam_tools/__init__.py"
    DESTINATION "${_python_install_dir}/lidarslam_benchmark_tools/lidarslam_tools")
  install(
    DIRECTORY "${source_root}/scripts/"
    DESTINATION "${_python_install_dir}/lidarslam_benchmark_tools"
    FILES_MATCHING
    PATTERN "*.py"
    # This launcher is a host validation orchestrator.  It must remain
    # available from the source checkout but cannot become part of the
    # benchmark/evidence Python API surface or be imported from an install.
    PATTERN "validate_registration_plugin_jazzy.py" EXCLUDE)
  install(
    PROGRAMS "${source_root}/scripts/container_phase_evidence.sh"
    DESTINATION "${_python_install_dir}/lidarslam_benchmark_tools")
  install(
    DIRECTORY "${source_root}/tools/gaussian_splatting/"
    DESTINATION "${_python_install_dir}/lidarslam_benchmark_tools/gaussian_splatting"
    FILES_MATCHING
    PATTERN "*.py"
    PATTERN "bim_reference_scripts" EXCLUDE)
  install(
    DIRECTORY "${source_root}/scripts/lidarslam_tools/"
    DESTINATION "${_python_install_dir}/lidarslam_benchmark_tools/lidarslam_tools"
    FILES_MATCHING
    PATTERN "*.py")
  install(
    DIRECTORY "${source_root}/configs/"
    DESTINATION "share/${project_name}/configs"
    FILES_MATCHING
    PATTERN "*.yaml"
    PATTERN "*.yml"
    PATTERN "*.json")
  install(
    FILES "${source_root}/docs/benchmark-python-surface.md"
    DESTINATION "share/${project_name}/docs")
  install(
    PROGRAMS "${source_root}/scripts/lidarslam_benchmark_tool"
    DESTINATION "lib/${project_name}")
endfunction()
