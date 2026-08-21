// Copyright 2026 Sasaki
// All rights reserved.
//
// Software License Agreement (BSD 2-Clause Simplified License)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
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

#ifndef LIDARSLAM_FAKE_REGISTRATION_PLUGINS__FAKE_REGISTRATION_PLUGINS_HPP_
#define LIDARSLAM_FAKE_REGISTRATION_PLUGINS__FAKE_REGISTRATION_PLUGINS_HPP_

#include <string>

#include <lidarslam_plugin_interfaces/registration.hpp>

namespace lidarslam_fake_registration_plugins
{

class BasicRegistration
  : public lidarslam::plugins::registration::RegistrationPlugin
{
public:
  explicit BasicRegistration(std::string class_id, std::uint16_t api_major = 1);
  ~BasicRegistration() override = default;

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
  std::string class_id_;
  std::uint16_t api_major_{1};
  bool configured_{false};
  lidarslam::plugins::registration::PointCloudConstPtr target_;
};

class IdentityRegistration final : public BasicRegistration
{
public:
  IdentityRegistration();
};

class NoGuessRegistration final : public BasicRegistration
{
public:
  NoGuessRegistration();
  lidarslam::plugins::registration::Capabilities capabilities() const override;
};

class BadApiRegistration final : public BasicRegistration
{
public:
  BadApiRegistration();
};

class BadMetadataRegistration final : public BasicRegistration
{
public:
  BadMetadataRegistration();
  lidarslam::plugins::registration::PluginMetadata metadata() const override;
};

class RejectingRegistration final : public BasicRegistration
{
public:
  RejectingRegistration();
  bool configure(
    const lidarslam::plugins::registration::ParameterMap & parameters,
    std::string * error) override;
};

class UnlicensedRegistration final : public BasicRegistration
{
public:
  UnlicensedRegistration();
  lidarslam::plugins::registration::PluginMetadata metadata() const override;
};

}  // namespace lidarslam_fake_registration_plugins

#endif  // LIDARSLAM_FAKE_REGISTRATION_PLUGINS__FAKE_REGISTRATION_PLUGINS_HPP_
