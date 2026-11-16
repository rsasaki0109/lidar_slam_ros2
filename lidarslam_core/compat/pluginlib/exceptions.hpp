// Copyright 2026 Sasaki
// All rights reserved.
//
// Software License Agreement (BSD 2-Clause Simplified License)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in
//    the documentation and/or other materials provided with the
//    distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// Stand-in for pluginlib's exceptions in the ROS-free build (lidarslam_core). Only the types
// the registration loader catches; see class_loader.hpp.

#ifndef LIDARSLAM_CORE_COMPAT__PLUGINLIB__EXCEPTIONS_HPP_
#define LIDARSLAM_CORE_COMPAT__PLUGINLIB__EXCEPTIONS_HPP_

#include <stdexcept>
#include <string>

namespace pluginlib
{

class PluginlibException : public std::runtime_error
{
public:
  explicit PluginlibException(const std::string & error_desc)
  : std::runtime_error(error_desc) {}
};

class LibraryLoadException : public PluginlibException
{
public:
  using PluginlibException::PluginlibException;
};

class CreateClassException : public PluginlibException
{
public:
  using PluginlibException::PluginlibException;
};

}  // namespace pluginlib

#endif  // LIDARSLAM_CORE_COMPAT__PLUGINLIB__EXCEPTIONS_HPP_
