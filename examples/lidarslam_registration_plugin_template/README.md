# Registration plugin template

This package is a small, buildable authoring example for the installed
`lidarslam_plugin_interfaces::RegistrationPlugin` C++14 source contract. It is
an identity implementation for contract tests, not a SLAM algorithm or an
accuracy claim.

Read the [registration plugin authoring guide](../../docs/registration-plugin-authoring.md)
before replacing the implementation. The guide covers capability declarations,
typed configuration, failure/reset behavior, plugin XML, license/provenance,
and the clean external-consumer proof.

The example class ID is:

```text
lidarslam_registration_plugin_template/Identity
```

The library is intentionally split from the loader. The plugin target is
C++14; the optional shell-side loader contract test is C++17 because the ROS 2
pluginlib shell owns discovery and lifetime. The plugin metadata class ID must
match the XML class name exactly, and its BSD-2-Clause license is accepted by
the loader's permissive-license policy.

Do not add dataset names, ROS node state, pluginlib discovery, or silent
algorithm fallback to an implementation copied from this template.
