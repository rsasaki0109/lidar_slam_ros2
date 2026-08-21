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
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef LIDARSLAM_DEFAULT_PLUGINS__SMALL_GICP_REGISTRATION_HPP_
#define LIDARSLAM_DEFAULT_PLUGINS__SMALL_GICP_REGISTRATION_HPP_

#include <memory>
#include <string>

#include <lidarslam_plugin_interfaces/registration.hpp>

namespace lidarslam_default_plugins
{

/**
 * Typed adapter for small_gicp's PCL GICP wrapper.
 *
 * The public class intentionally exposes only the C++14 registration
 * contract.  small_gicp headers and template instantiations stay in the
 * implementation include so the scanmatcher host can include that same
 * implementation from its legacy translation unit when exact equivalence is
 * required.
 */
class SmallGicpRegistration
  : public lidarslam::plugins::registration::RegistrationPlugin
{
public:
  SmallGicpRegistration();
  ~SmallGicpRegistration() override;

  SmallGicpRegistration(const SmallGicpRegistration &) = delete;
  SmallGicpRegistration & operator=(const SmallGicpRegistration &) = delete;

  lidarslam::plugins::registration::PluginMetadata metadata() const override;
  lidarslam::plugins::registration::Capabilities capabilities() const override;

  bool configure(
    const lidarslam::plugins::registration::ParameterMap & parameters,
    std::string * error) override;

  bool setInputTarget(
    const lidarslam::plugins::registration::PointCloudConstPtr & target,
    std::string * error) override;

  lidarslam::plugins::registration::AlignmentResult align(
    const lidarslam::plugins::registration::AlignmentRequest & request) override;

  void reset() noexcept override;

protected:
  explicit SmallGicpRegistration(bool voxelized);

  bool voxelized() const noexcept;

private:
  struct Impl;
  struct PerCallStateGuard;
  static void clearPerCallState(Impl * implementation) noexcept;
  std::unique_ptr<Impl> impl_;
};

/**
 * The VGICP variant has a separate pluginlib type so its class ID cannot be
 * confused with SMALL_GICP.  Both variants share the same typed adapter
 * implementation and differ only in the fixed registration type and voxel
 * resolution parameter.
 */
class SmallVgicpRegistration final : public SmallGicpRegistration
{
public:
  SmallVgicpRegistration();
  ~SmallVgicpRegistration() override;

  lidarslam::plugins::registration::PluginMetadata metadata() const override;
};

}  // namespace lidarslam_default_plugins

#endif  // LIDARSLAM_DEFAULT_PLUGINS__SMALL_GICP_REGISTRATION_HPP_
