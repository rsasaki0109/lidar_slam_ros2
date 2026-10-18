#!/usr/bin/env bash
# Runs the graph_based_slam test suite of /home/user/ws/slam_dev with the default-CI exclusions
# of scripts/run_default_ci_checks.sh. Usage: test_slam_dev.sh <log> [extra ctest -R/-E args...]
set -o pipefail
log=$1; shift
set +u; source /opt/ros/jazzy/setup.bash; source /home/user/ws/slam_dev/install/setup.bash; set -u
cd /home/user/ws/slam_dev
EXCL='^(test_competitive_slam_profile|test_competitive_container_recipes|test_competitive_execution_identity_capture|test_competitive_execution_r3_source_worktree|test_competitive_holdout_authorization|test_competitive_rival_legal_provenance_capture|test_competitive_rival_source_closure|test_compose_competitive_result|test_compose_competitive_evidence_bundle|test_evaluate_competitive_sequence_gate|test_evaluate_competitive_suite_gate|test_ours_competitive_benchmark|test_ours_competitive_benchmark_cli|test_ours_competitive_benchmark_runner|test_prepare_competitive_evidence_bundle_handoff|test_publish_competitive_claim|test_run_competitive_gt_blind_benchmark|test_verify_competitive_evidence_bundle|test_glim_benchmark_runner|test_ntu_viral_acquisition_pipeline|test_fast_gicp_selector|test_ndt_omp_registration|uncrustify)$'
start=$(date +%s)
ROS_DOMAIN_ID=41 colcon test --base-paths /home/user/lidar_slam_ros2 --packages-select graph_based_slam --event-handlers console_direct- \
  --ctest-args -E "${EXCL}" "$@" > "${log}" 2>&1
echo "colcon_test_exit=$?" >> "${log}"
colcon test-result --test-result-base build/graph_based_slam >> "${log}" 2>&1
echo "test_seconds=$(( $(date +%s) - start ))" >> "${log}"
