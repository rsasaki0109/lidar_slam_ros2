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

#ifndef LIDARSLAM_REGISTRATION_PLUGIN_TEMPLATE__TEMPLATE_REGISTRATION_HPP_
#define LIDARSLAM_REGISTRATION_PLUGIN_TEMPLATE__TEMPLATE_REGISTRATION_HPP_

#include <string>

#include <lidarslam_plugin_interfaces/registration.hpp>

namespace lidarslam_registration_plugin_template
{

class IdentityRegistration final
  : public lidarslam::plugins::registration::RegistrationPlugin,
    public lidarslam::plugins::registration::RegistrationPluginDescriptorProvider
{
public:
  IdentityRegistration() = default;
  ~IdentityRegistration() override = default;

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
  bool configured_{false};
  std::string mode_{"identity"};
  lidarslam::plugins::registration::PointCloud::Ptr target_;
};

}  // namespace lidarslam_registration_plugin_template

#endif  // LIDARSLAM_REGISTRATION_PLUGIN_TEMPLATE__TEMPLATE_REGISTRATION_HPP_
