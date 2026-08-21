# Registration plugin authoring guide

This guide is the minimum author contract for an external scan-to-map
registration plugin. The public boundary is the installed, ROS-free
`lidarslam::plugins::registration::RegistrationPlugin` interface. The source
compatibility target is C++14; the shell-side `pluginlib` loader is C++17 and
owns discovery, startup validation, provenance, and loader lifetime.

The repository contains a buildable reference at
`examples/lidarslam_registration_plugin_template`. It returns the requested
initial guess and source cloud so the contract can be tested without making an
algorithm or accuracy claim. Copy its package structure, then replace the
identity behavior with a real registration implementation.

## 1. Package boundary

An external package should contain only the plugin implementation and its
manifest. It must not include `scanmatcher` or `graph_based_slam` internals,
read ROS parameters directly, perform pluginlib discovery, or choose a
fallback algorithm. The shell selects one explicit class ID and injects a
configured object before sensor processing.

The required class ID is a stable external namespace, for example:

```text
acme_registration_plugins/RobustNdt
```

Do not use the reserved host namespace `lidarslam_builtin/`. A class ID is
case-sensitive and must be identical in all three places:

1. the plugin XML `class name`;
2. `PluginMetadata::class_id`;
3. the host's explicit selector.

The host rejects unknown IDs, duplicate/reserved IDs, API-major mismatches,
invalid metadata, disallowed licenses, unsupported capabilities, and rejected
configuration. There is no implicit NDT/GICP or legacy fallback.

## 2. C++14 contract

Implement all six methods:

```cpp
class RobustNdt final
  : public lidarslam::plugins::registration::RegistrationPlugin
{
public:
  PluginMetadata metadata() const override;
  Capabilities capabilities() const override;
  bool configure(const ParameterMap &, std::string * error) override;
  bool setInputTarget(const PointCloudConstPtr &, std::string * error) override;
  AlignmentResult align(const AlignmentRequest &) override;
  void reset() noexcept override;
};
```

The interface header is the only public algorithm dependency. Build the plugin
library with C++14 and strict warnings:

```cmake
find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(lidarslam_plugin_interfaces REQUIRED)

add_library(${PROJECT_NAME} SHARED src/robust_ndt.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_14)
ament_target_dependencies(${PROJECT_NAME}
  pluginlib lidarslam_plugin_interfaces)
pluginlib_export_plugin_description_file(
  lidarslam_plugin_interfaces registration_plugins.xml)
```

The plugin target may link its own algorithm dependency, but it must not make
the ROS-free interface package depend on ROS or pluginlib. C++ binary ABI is
not promised across Humble/Jazzy; build and test a plugin separately for each
target distribution/toolchain.

## 3. Metadata and capabilities

Return stable metadata before and after `configure()`:

- `class_id`: exact XML selector;
- `implementation_version`: non-empty implementation version controlled by the
  plugin author;
- `api_version`: `kHostApiVersion` or a compatible older minor version;
- `license`: an SPDX-style permissive identifier accepted by the shell policy:
  `Apache-2.0`, `BSD-2-Clause`, `BSD-3-Clause`, `ISC`, `MIT`, or `Zlib`.

Advertise only capabilities that are actually implemented and tested. In
particular:

- `kInitialGuess`, priors, adaptive maximum distance, covariance, and
  `kAlignedSource` must correspond to the request/result behavior;
- select `TargetPolicy` to describe whether the plugin accepts the host's
  prepared target, requires raw target geometry, or preprocesses the target;
- select `CorrespondenceMetric` to match the valid diagnostic returned by every
  successful alignment;
- use `kDeterministic` only after repeated fixture runs prove byte-identical
  transform, fitness, diagnostics, and aligned cloud under the declared thread
  model;
- declare `kSerializedOwner` unless concurrent calls are explicitly safe.

The loader re-queries capabilities after configuration, so configuration-
dependent claims such as deterministic single-thread operation must be
resolved before the session is accepted.

## 4. Lifecycle and failure semantics

The required lifecycle is:

```text
construct -> metadata/capabilities -> configure -> setInputTarget -> align*
                                                                    |
                                                                  reset
```

`configure()` must validate the typed `ParameterMap` once at startup. Reject
unknown keys, wrong `ParameterValue` types, non-finite values, and invalid
ranges with `false` and an actionable error string. Do not partially apply a
configuration after returning `false`.

`setInputTarget()` is called only after successful configuration. Reject a null,
empty, or non-finite target; copy it or retain it only under a documented
immutable-lifetime contract. `align()` must:

- reject an unconfigured plugin or missing target with a typed failure;
- validate source geometry and return `kInvalidInput` for null, empty, or
  non-finite input;
- call `validateRequest()` and return `kUnsupportedCapability` for a request
  that exceeds the declared capabilities;
- validate an enabled finite initial guess; when
  `initial_guess_enabled == false`, ignore the matrix contents entirely;
- return `FailureCode::kNone` only with a valid result, and set convergence,
  transform, fitness, diagnostics, and optional aligned cloud consistently;
- catch implementation exceptions and convert them to `kInternalError` or an
  algorithm-specific typed failure. No exception may cross the interface.

`reset()` is `noexcept`, clears target and configuration state, and leaves the
plugin safe to destroy or configure again. After reset, the next alignment
must fail with `kNotConfigured`; it must not silently reuse the prior target.

## 5. Plugin XML, install, and provenance

The manifest path and library are part of the install-space contract:

```xml
<?xml version="1.0"?>
<library path="acme_registration_plugins">
  <class
    name="acme_registration_plugins/RobustNdt"
    type="acme_registration_plugins::RobustNdt"
    base_class_type="lidarslam::plugins::registration::RegistrationPlugin">
    <description>Acme robust NDT registration.</description>
  </class>
</library>
```

Export the XML with `pluginlib_export_plugin_description_file`, install the
shared library and public headers, and declare the same BSD/MIT/Apache-style
license in `package.xml`. Do not put negative-test classes or missing-library
fixtures in an author package. The repository's
`lidarslam_fake_registration_plugins` package is reserved for loader failure
tests.

## 6. Contract tests and clean proof

At minimum, an author package should test metadata/XML identity, typed config
acceptance/rejection, target validation, one successful alignment, invalid
input, capability mismatch, disabled-initial-guess behavior, repeated
determinism when advertised, and reset-after-align. The template provides both
a C++14 direct contract test and a C++17 shell-loader discovery/lifetime test.

Run the template proof from the repository root:

```bash
bash scripts/run_registration_plugin_template_check.sh --keep-work-dir
```

The script copies only the interface and loader into a temporary underlay,
copies the template into a separate temporary overlay, starts from
`/opt/ros/$ROS_DISTRO/setup.bash`, and never sources the repository install
space. It builds the plugin target as C++14, runs the direct contract fixture,
then loads the installed class through the shell loader. A passing receipt
contains:

```text
m1_template_proof=pass
template_package=lidarslam_registration_plugin_template
template_class=lidarslam_registration_plugin_template/Identity
plugin_cxx_standard=14
loader_test_cxx_standard=17
repository_install_sourced=false
```

Run it once on Humble and once on Jazzy. Keep the image/compiler, PCL/Eigen,
pluginlib version, resolved manifest path, and library path with the receipt.
These are compatibility and contract results, not an accuracy or SOTA claim.
