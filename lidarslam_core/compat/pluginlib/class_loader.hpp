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

// Stand-in for pluginlib::ClassLoader in the ROS-free build (lidarslam_core).
//
// The registration loader is compiled unchanged; without ROS there is no plugin index, so the
// loader's construction always fails and the resolver serves the host built-in registrations
// only (NDT, GICP, SMALL_GICP), as the ROS 2 build does when no plugin package is installed.
// No object of this type is ever constructed.

#ifndef LIDARSLAM_CORE_COMPAT__PLUGINLIB__CLASS_LOADER_HPP_
#define LIDARSLAM_CORE_COMPAT__PLUGINLIB__CLASS_LOADER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "pluginlib/exceptions.hpp"

namespace pluginlib
{

template<typename T>
class ClassLoader
{
public:
  ClassLoader(
    const std::string & package, const std::string & base_class,
    const std::string & attrib_name = "plugin",
    const std::vector<std::string> & plugin_xml_paths = {})
  {
    (void)base_class;
    (void)attrib_name;
    (void)plugin_xml_paths;
    throw PluginlibException(
            "this build has no pluginlib (no ROS 2); registration plugins of package '" + package +
            "' cannot be loaded, only the host built-in registrations");
  }

  std::vector<std::string> getDeclaredClasses() {return {};}
  std::string getClassLibraryPath(const std::string &) {return {};}
  std::string getPluginManifestPath(const std::string &) {return {};}
  std::shared_ptr<T> createSharedInstance(const std::string & lookup_name)
  {
    throw CreateClassException("no pluginlib in this build: " + lookup_name);
  }
};

}  // namespace pluginlib

#endif  // LIDARSLAM_CORE_COMPAT__PLUGINLIB__CLASS_LOADER_HPP_
