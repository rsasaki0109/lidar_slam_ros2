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
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
// AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
// THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef LIDARSLAM_DEFAULT_PLUGINS__GICP_OMP_REGISTRATION_HPP_
#define LIDARSLAM_DEFAULT_PLUGINS__GICP_OMP_REGISTRATION_HPP_

#include <memory>
#include <string>

#include <lidarslam_plugin_interfaces/registration.hpp>

namespace lidarslam_default_plugins
{

/**
 * Typed adapter for the existing pclomp GICP construction in scanmatcher.
 *
 * The host compatibility factory constructs this class in the legacy
 * scanmatcher translation unit.  The public header deliberately keeps pclomp
 * implementation templates out of the C++14 registration interface.
 */
class GicpOmpRegistration final
  : public lidarslam::plugins::registration::RegistrationPlugin,
  public lidarslam::plugins::registration::RegistrationPluginDescriptorProvider
{
public:
  GicpOmpRegistration();
  ~GicpOmpRegistration() override;

  GicpOmpRegistration(const GicpOmpRegistration &) = delete;
  GicpOmpRegistration & operator=(const GicpOmpRegistration &) = delete;

  lidarslam::plugins::registration::PluginMetadata metadata() const override;
  lidarslam::plugins::registration::Capabilities capabilities() const override;
  lidarslam::plugins::registration::RegistrationRuntimeDescriptor
  registrationDescriptor() const override;

  bool configure(
    const lidarslam::plugins::registration::ParameterMap & parameters,
    std::string * error) override;

  bool setInputTarget(
    const lidarslam::plugins::registration::PointCloudConstPtr & target,
    std::string * error) override;

  lidarslam::plugins::registration::AlignmentResult align(
    const lidarslam::plugins::registration::AlignmentRequest & request) override;

  void reset() noexcept override;

private:
  struct Impl;
  struct PerCallStateGuard;
  static void clearPerCallState(Impl * implementation) noexcept;
  std::unique_ptr<Impl> impl_;
};

}  // namespace lidarslam_default_plugins

#endif  // LIDARSLAM_DEFAULT_PLUGINS__GICP_OMP_REGISTRATION_HPP_
