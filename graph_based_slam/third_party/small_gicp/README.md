# small_gicp (vendored)

A copy of the [small_gicp](https://github.com/koide3/small_gicp) library by Kenji Koide,
under the MIT license (`LICENSE` in this directory).

graph_based_slam builds it as a static library (`graph_based_slam_small_gicp`, used through the
alias `small_gicp::small_gicp`), so loop registration with `registration_method: SMALL_GICP` and
`graph_slam_multisession_join` are in every build: from source, in Docker and in the binary
release. No external small_gicp is searched for.

- Upstream commit: `fa0cfc983417c044b6036f71dfa448038bb9c820` (`v1.0.1-6-gfa0cfc9`).
- Copied: `include/small_gicp/{ann,factors,pcl,points,registration,util}` and
  `src/small_gicp/registration/{registration,registration_helper}.cpp`, with no changes.
- Left out: the benchmark and ROS conversion headers, the Python bindings, tests, examples and
  the upstream build files.
- `AMENT_IGNORE` keeps the ament linters (copyright, cpplint, cppcheck, uncrustify) off this
  code; it is checked as upstream code, not to this repository's style.

To update: copy the same paths from a newer upstream commit, record that commit here and run the
graph_based_slam tests.
