# Copyright 2026 Sasaki
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


# Configure-time helper for install-time, post-link contract sidecars.

# Capture this path while this helper is being included.  install(CODE) runs
# later from CMake's install-script context, where CMAKE_CURRENT_LIST_DIR is
# the build/install script directory rather than this source directory.
set(LIDARSLAM_REGISTRATION_CONTRACT_GENERATOR
  "${CMAKE_CURRENT_LIST_DIR}/generate_registration_contract_manifest.cmake")

if(NOT DEFINED LIDARSLAM_REGISTRATION_TOOLCHAIN_TAG)
  if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    string(REGEX MATCH "^[0-9]+" _registration_gcc_major "${CMAKE_CXX_COMPILER_VERSION}")
    set(_registration_compiler_tag "gcc-${_registration_gcc_major}")
  elseif(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    string(REGEX MATCH "^[0-9]+" _registration_clang_major "${CMAKE_CXX_COMPILER_VERSION}")
    set(_registration_compiler_tag "clang-${_registration_clang_major}")
  elseif(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    set(_registration_compiler_tag "msvc-${MSVC_VERSION}")
  else()
    set(_registration_compiler_tag "unknown-compiler")
  endif()
  set(_registration_abi_probe_args)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    # GCC/Clang expose _GLIBCXX_USE_CXX11_ABI from this libstdc++ header,
    # not from their bare predefined-macro stream.
    list(APPEND _registration_abi_probe_args -include bits/c++config.h)
  endif()
  execute_process(
    COMMAND ${CMAKE_CXX_COMPILER} ${CMAKE_CXX_FLAGS}
      ${_registration_abi_probe_args} -dM -E -x c++ /dev/null
    OUTPUT_VARIABLE _registration_predefined_macros
    ERROR_QUIET)
  string(REGEX MATCH "#define[ \t]+_GLIBCXX_USE_CXX11_ABI[ \t]+([01])"
    _registration_abi_match "${_registration_predefined_macros}")
  if(_registration_abi_match)
    set(_registration_stdlib_tag "libstdcxx-cxx11-abi-${CMAKE_MATCH_1}")
  else()
    set(_registration_stdlib_tag "stdlib-abi-unknown")
  endif()
  set(LIDARSLAM_REGISTRATION_TOOLCHAIN_TAG
    "${_registration_compiler_tag};${_registration_stdlib_tag}" CACHE INTERNAL
    "Registration plugin toolchain identity")
endif()

function(lidarslam_install_registration_contract TARGET_NAME XML_NAME CLASS_ID
    REQUIRED_BITS OPTIONAL_BITS TARGET_POLICY CORRESPONDENCE_METRIC THREAD_MODEL
    CONFIG_SCHEMA_ID CONFIG_SCHEMA_VERSION CONFIG_SCHEMA_SHA256)
  if(NOT TARGET "${TARGET_NAME}")
    message(FATAL_ERROR
      "registration contract target does not exist: ${TARGET_NAME}")
  endif()
  # Do not depend on CMP0087 or a generator expression inside install(CODE):
  # those expressions are policy-scoped by the consuming project and may be
  # left literal in the generated install script.  Capture the target's
  # configured basename now, while preserving the actual post-link bytes for
  # hashing at install time.
  get_target_property(_registration_output_name "${TARGET_NAME}" OUTPUT_NAME)
  if(NOT _registration_output_name OR _registration_output_name MATCHES "-NOTFOUND$")
    set(_registration_output_name "${TARGET_NAME}")
  endif()
  get_target_property(_registration_prefix "${TARGET_NAME}" PREFIX)
  if(NOT _registration_prefix OR _registration_prefix MATCHES "-NOTFOUND$")
    set(_registration_prefix "${CMAKE_SHARED_LIBRARY_PREFIX}")
  endif()
  get_target_property(_registration_suffix "${TARGET_NAME}" SUFFIX)
  if(NOT _registration_suffix OR _registration_suffix MATCHES "-NOTFOUND$")
    set(_registration_suffix "${CMAKE_SHARED_LIBRARY_SUFFIX}")
  endif()
  if(_registration_output_name MATCHES "\\$<" OR
    _registration_prefix MATCHES "\\$<" OR _registration_suffix MATCHES "\\$<")
    message(FATAL_ERROR
      "registration contract target filename contains an unresolved generator expression: ${TARGET_NAME}")
  endif()
  set(_registration_dso_basename
    "${_registration_prefix}${_registration_output_name}${_registration_suffix}")
  # CMake treats semicolons as list separators when it parses install(CODE).
  # Emit the two toolchain components separately and reconstruct the literal
  # delimiter at install time, so the JSON receives ';' rather than '\\;'.
  list(GET LIDARSLAM_REGISTRATION_TOOLCHAIN_TAG 0 _registration_compiler_tag_install)
  list(LENGTH LIDARSLAM_REGISTRATION_TOOLCHAIN_TAG _registration_toolchain_tag_length)
  if(_registration_toolchain_tag_length GREATER 1)
    list(GET LIDARSLAM_REGISTRATION_TOOLCHAIN_TAG 1 _registration_stdlib_tag_install)
  else()
    set(_registration_stdlib_tag_install "stdlib-abi-unknown")
  endif()
  set(_registration_cancellation_model_install 0)
  if(ARGC GREATER 11)
    set(_registration_cancellation_model_install "${ARGV11}")
  endif()
  string(CONCAT _registration_code
    "set(REGISTRATION_CONTRACT_XML \"${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/${XML_NAME}\")\n"
    "set(REGISTRATION_CONTRACT_DSO \"${CMAKE_INSTALL_PREFIX}/lib/${_registration_dso_basename}\")\n"
    "set(REGISTRATION_CONTRACT_CLASS_ID \"${CLASS_ID}\")\n"
    "set(REGISTRATION_CONTRACT_TOOLCHAIN_TAG \"${_registration_compiler_tag_install}\")\n"
    # Keep the literal semicolon out of install(CODE)'s configure-time
    # argument list.  Reconstruct it inside the generated install script.
    "string(ASCII 59 REGISTRATION_CONTRACT_TOOLCHAIN_SEPARATOR)\n"
    "string(APPEND REGISTRATION_CONTRACT_TOOLCHAIN_TAG \"\${REGISTRATION_CONTRACT_TOOLCHAIN_SEPARATOR}${_registration_stdlib_tag_install}\")\n"
    "set(REGISTRATION_CONTRACT_API_MIN_MAJOR 1)\n"
    "set(REGISTRATION_CONTRACT_API_MIN_MINOR 0)\n"
    "set(REGISTRATION_CONTRACT_API_MAX_MAJOR 1)\n"
    "set(REGISTRATION_CONTRACT_API_MAX_MINOR 0)\n"
    "set(REGISTRATION_CONTRACT_REQUIRED_CAPABILITY_BITS ${REQUIRED_BITS})\n"
    "set(REGISTRATION_CONTRACT_OPTIONAL_CAPABILITY_BITS ${OPTIONAL_BITS})\n"
    "set(REGISTRATION_CONTRACT_TARGET_POLICY ${TARGET_POLICY})\n"
    "set(REGISTRATION_CONTRACT_CORRESPONDENCE_METRIC ${CORRESPONDENCE_METRIC})\n"
    "set(REGISTRATION_CONTRACT_THREAD_MODEL ${THREAD_MODEL})\n"
    "set(REGISTRATION_CONTRACT_CANCELLATION_MODEL ${_registration_cancellation_model_install})\n"
    "set(REGISTRATION_CONTRACT_CONFIG_SCHEMA_ID \"${CONFIG_SCHEMA_ID}\")\n"
    "set(REGISTRATION_CONTRACT_CONFIG_SCHEMA_VERSION ${CONFIG_SCHEMA_VERSION})\n"
    "set(REGISTRATION_CONTRACT_CONFIG_SCHEMA_SHA256 \"${CONFIG_SCHEMA_SHA256}\")\n"
    "include(\"${LIDARSLAM_REGISTRATION_CONTRACT_GENERATOR}\")\n")
  install(CODE "${_registration_code}")
endfunction()
