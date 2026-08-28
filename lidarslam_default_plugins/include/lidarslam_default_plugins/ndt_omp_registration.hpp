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
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following disclaimer
//    in the documentation and/or other materials provided with the
//    distribution.
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

#ifndef LIDARSLAM_DEFAULT_PLUGINS__NDT_OMP_REGISTRATION_HPP_
#define LIDARSLAM_DEFAULT_PLUGINS__NDT_OMP_REGISTRATION_HPP_

#include <cstddef>
#include <memory>
#include <string>

#include <lidarslam_plugin_interfaces/registration.hpp>

namespace lidarslam_default_plugins
{

struct TargetCellCacheStats
{
  std::size_t capacity{0U};
  std::size_t size{0U};
  std::size_t hits{0U};
  std::size_t misses{0U};
  std::size_t evictions{0U};
};

/**
 * Built-in adapter for the pclomp DIRECT7 NDT used by the current scan matcher.
 *
 * The adapter deliberately has no ROS or pluginlib dependency.  A shell owns
 * configuration and lifetime, and serializes calls according to the declared
 * capabilities.  The implementation is kept behind a PIMPL so consumers only
 * need the stable registration interface, not pclomp template definitions.
 */
class NdtOmpRegistration final
  : public lidarslam::plugins::registration::RegistrationPlugin,
  public lidarslam::plugins::registration::RegistrationPluginDescriptorProvider
{
public:
  NdtOmpRegistration();
  ~NdtOmpRegistration() override;

  NdtOmpRegistration(const NdtOmpRegistration &) = delete;
  NdtOmpRegistration & operator=(const NdtOmpRegistration &) = delete;

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

  // Concrete diagnostics for the backend characterization/tests.  The
  // generic C++14 RegistrationPlugin contract is unchanged; callers that do
  // not opt into target-cell caching see zero capacity and no retained state.
  TargetCellCacheStats targetCellCacheStats() const noexcept;

private:
  struct Impl;
  struct PerCallStateGuard;
  static void clearPerCallState(Impl * implementation) noexcept;
  std::unique_ptr<Impl> impl_;
};

}  // namespace lidarslam_default_plugins

#endif  // LIDARSLAM_DEFAULT_PLUGINS__NDT_OMP_REGISTRATION_HPP_
