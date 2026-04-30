# Copyright 2026 Sasaki
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Smoke tests for top-level docs entry points."""

from __future__ import annotations

from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
README_PATH = REPO_ROOT / 'README.md'
CONTRIBUTING_PATH = REPO_ROOT / 'CONTRIBUTING.md'
VERSION_PATH = REPO_ROOT / 'VERSION'
CHANGELOG_PATH = REPO_ROOT / 'CHANGELOG.md'
RELEASING_PATH = REPO_ROOT / 'RELEASING.md'
MKDOCS_CONFIG_PATH = REPO_ROOT / 'mkdocs.yml'
DOCS_INDEX_PATH = REPO_ROOT / 'docs' / 'index.md'
DOCS_ASSETS_DIR = REPO_ROOT / 'docs' / 'assets'
DOCS_EXTRA_CSS_PATH = DOCS_ASSETS_DIR / 'stylesheets' / 'extra.css'
DOCS_AUTOWARE_PROOF_SITE_IMAGE_PATH = DOCS_ASSETS_DIR / 'images' / 'autoware_map_loader_proof.png'
DOCS_DYNAMIC_FILTER_SITE_IMAGE_PATH = (
    DOCS_ASSETS_DIR / 'images' / 'dynamic_object_filter_bag6_summary.svg'
)
AUTOWARE_QUICKSTART = REPO_ROOT / 'docs' / 'autoware-quickstart.md'
AUTOWARE_MAP_AUTHORING = REPO_ROOT / 'docs' / 'autoware-map-authoring.md'
AUTOWARE_FOXGLOVE = REPO_ROOT / 'docs' / 'autoware-foxglove.md'
WORKFLOWS_DOC = REPO_ROOT / 'docs' / 'workflows.md'
BENCHMARKING_DOC = REPO_ROOT / 'docs' / 'benchmarking.md'
COMPARISON_DOC = REPO_ROOT / 'docs' / 'comparison.md'
SOCIAL_POST_DOC = REPO_ROOT / 'docs' / 'social' / 'autoware_map_authoring_post_v0.2.2.md'
ISSUE_TEMPLATE_DIR = REPO_ROOT / '.github' / 'ISSUE_TEMPLATE'
PUBLIC_AUTOWARE_ENTRYPOINT = REPO_ROOT / 'scripts' / 'run_autoware_quickstart.sh'
RELEASE_WORKFLOW = REPO_ROOT / '.github' / 'workflows' / 'release.yml'
DOCS_SITE_WORKFLOW = REPO_ROOT / '.github' / 'workflows' / 'docs-site.yml'
README_LOOP_IMAGE_PATH = REPO_ROOT / 'lidarslam' / 'images' / 'mid360_loop_closure_zoom.png'
README_AUTOWARE_PROOF_IMAGE_PATH = (
    REPO_ROOT / 'lidarslam' / 'images' / 'autoware_map_loader_proof.png'
)
README_DYNAMIC_FILTER_IMAGE_PATH = (
    REPO_ROOT / 'lidarslam' / 'images' / 'dynamic_object_filter_bag6_summary.svg'
)
SOCIAL_CARD_PATH = (
    REPO_ROOT / 'lidarslam' / 'images' / 'social_autoware_map_authoring.png'
)
SOCIAL_DEMO_VIDEO_PATH = (
    REPO_ROOT / 'lidarslam' / 'images' / 'social_autoware_map_authoring_demo.mp4'
)
BENCHMARK_SUMMARY_PATH = REPO_ROOT / 'output' / 'benchmark_summary.md'
BENCHMARK_REPORT_PATH = REPO_ROOT / 'output' / 'latest_report.html'
STRESS_REPORT_PATH = REPO_ROOT / 'output' / 'stress_validation_report_20260325.md'
V2_READINESS_PATH = REPO_ROOT / 'output' / 'v2_beta_readiness_20260324.md'


def test_docs_exist_and_are_linked_from_readme():
    """README should link to the main adoption-oriented docs."""
    readme = README_PATH.read_text(encoding='utf-8')
    version = VERSION_PATH.read_text(encoding='utf-8').strip()
    release_notes_path = REPO_ROOT / 'docs' / 'releases' / f'v{version}.md'

    assert CONTRIBUTING_PATH.is_file()
    assert VERSION_PATH.is_file()
    assert CHANGELOG_PATH.is_file()
    assert RELEASING_PATH.is_file()
    assert MKDOCS_CONFIG_PATH.is_file()
    assert DOCS_INDEX_PATH.is_file()
    assert DOCS_ASSETS_DIR.is_dir()
    assert DOCS_EXTRA_CSS_PATH.is_file()
    assert DOCS_AUTOWARE_PROOF_SITE_IMAGE_PATH.is_file()
    assert DOCS_DYNAMIC_FILTER_SITE_IMAGE_PATH.is_file()
    assert AUTOWARE_QUICKSTART.is_file()
    assert AUTOWARE_MAP_AUTHORING.is_file()
    assert AUTOWARE_FOXGLOVE.is_file()
    assert WORKFLOWS_DOC.is_file()
    assert BENCHMARKING_DOC.is_file()
    assert COMPARISON_DOC.is_file()
    assert SOCIAL_POST_DOC.is_file()
    assert DOCS_SITE_WORKFLOW.is_file()
    assert README_LOOP_IMAGE_PATH.is_file()
    assert README_AUTOWARE_PROOF_IMAGE_PATH.is_file()
    assert README_DYNAMIC_FILTER_IMAGE_PATH.is_file()
    assert SOCIAL_CARD_PATH.is_file()
    assert SOCIAL_DEMO_VIDEO_PATH.is_file()
    assert release_notes_path.is_file()
    assert '(CONTRIBUTING.md)' in readme
    assert '(CHANGELOG.md)' in readme
    assert '(RELEASING.md)' in readme
    assert '(docs/autoware-map-authoring.md)' in readme
    assert '(docs/autoware-quickstart.md)' in readme
    assert '(docs/autoware-foxglove.md)' in readme
    assert '(docs/workflows.md)' in readme
    assert '(docs/comparison.md)' in readme
    assert '(docs/benchmarking.md)' in readme
    assert 'python3 -m mkdocs serve' in readme
    assert 'run_autoware_map_beginner.sh' in readme
    assert '(lidarslam/images/autoware_map_loader_proof.png)' in readme
    assert '(lidarslam/images/dynamic_object_filter_bag6_summary.svg)' in readme
    assert 'git clone --recursive https://github.com/rsasaki0109/lidarslam_ros2.git' in readme
    assert 'rosdep install --from-paths src --ignore-src -r -y' in readme
    assert 'Required input topics for the main public path' in readme
    assert '/gnss/fix' in readme
    assert 'gnss_topic' in readme
    assert f'(docs/releases/v{version}.md)' in readme
    assert len(readme.splitlines()) <= 220


def test_docs_reference_existing_entrypoint_scripts():
    """Every documented entrypoint script should exist in the repo."""
    scripts = [
        PUBLIC_AUTOWARE_ENTRYPOINT,
        REPO_ROOT / 'scripts' / 'download_ntu_viral_tnp01.sh',
        REPO_ROOT / 'scripts' / 'run_default_ci_checks.sh',
        REPO_ROOT / 'scripts' / 'run_rko_lio_graph_autoware_dogfood.sh',
        REPO_ROOT / 'scripts' / 'run_graph_slam_pointcloud_map_in_autoware.sh',
        REPO_ROOT / 'scripts' / 'prepare_autoware_map_from_graph_slam.sh',
        REPO_ROOT / 'scripts' / 'create_map_authoring_submission_bundle.sh',
        REPO_ROOT / 'scripts' / 'run_autoware_pointcloud_map_viewer_docker.sh',
        REPO_ROOT / 'scripts' / 'prepare_foxglove_bridge_prefix.sh',
        REPO_ROOT / 'scripts' / 'run_autoware_pointcloud_map_foxglove.sh',
        REPO_ROOT / 'scripts' / 'run_graph_slam_pointcloud_map_in_autoware_foxglove.sh',
        REPO_ROOT / 'scripts' / 'run_rko_lio_graph_benchmark.sh',
        REPO_ROOT / 'scripts' / 'run_rko_lio_mid360_crossval_benchmark.sh',
        REPO_ROOT / 'scripts' / 'run_release_readiness_checks.sh',
        REPO_ROOT / 'scripts' / 'benchmark_summary.py',
        REPO_ROOT / 'scripts' / 'generate_html_report.py',
        REPO_ROOT / 'scripts' / 'generate_v2_beta_readiness_report.py',
        REPO_ROOT / 'scripts' / 'generate_map_authoring_report.py',
        REPO_ROOT / 'scripts' / 'generate_stress_validation_report.py',
        REPO_ROOT / 'scripts' / 'generate_readme_dynamic_filter_figure.py',
        REPO_ROOT / 'scripts' / 'generate_readme_autoware_proof_figure.py',
        REPO_ROOT / 'scripts' / 'generate_readme_large_loop_map_figure.py',
        REPO_ROOT / 'scripts' / 'generate_readme_loop_zoom_figure.py',
        REPO_ROOT / 'scripts' / 'generate_social_autoware_map_authoring_card.py',
        REPO_ROOT / 'scripts' / 'generate_social_autoware_demo_video.py',
        REPO_ROOT / 'scripts' / 'write_aligned_trajectory_metrics.py',
        REPO_ROOT / 'scripts' / 'generate_sample_benchmark_metrics.py',
        REPO_ROOT / 'scripts' / 'inspect_navsatfix_covariance.py',
        REPO_ROOT / 'scripts' / 'inspect_applanix_gsof50_quality.py',
        REPO_ROOT / 'scripts' / 'convert_applanix_gsof_to_navsatfix_bag.py',
        REPO_ROOT / 'scripts' / 'convert_applanix_gsof_to_imu_bag.py',
        REPO_ROOT / 'scripts' / 'extract_applanix_gsof49_reference.py',
        REPO_ROOT / 'scripts' / 'extract_static_transform_from_bag.py',
        REPO_ROOT / 'scripts' / 'prepare_velodyne_pointcloud_overlay.sh',
        REPO_ROOT / 'scripts' / 'run_open_data_gnss_smoke.sh',
        REPO_ROOT / 'scripts' / 'run_open_data_applanix_velodyne_gnss_smoke.sh',
        REPO_ROOT / 'scripts' / 'run_open_data_applanix_velodyne_gnss_benchmark.sh',
        REPO_ROOT / 'scripts' / 'run_open_data_classic_path_benchmark_suite.sh',
        REPO_ROOT / 'scripts' / 'generate_odom_prior_validation_report.py',
        REPO_ROOT / 'scripts' / 'run_open_data_packet_imu_deskew_validation_matrix.sh',
        REPO_ROOT / 'scripts' / 'run_dynamic_object_filter_benchmark.sh',
        REPO_ROOT / 'scripts' / 'generate_dynamic_object_filter_validation_report.py',
        REPO_ROOT / 'scripts' / 'generate_exploration_closeout_report.py',
        REPO_ROOT / 'scripts' / 'run_place_recognition_benchmark.sh',
        REPO_ROOT / 'scripts' / 'generate_classic_path_report.py',
        REPO_ROOT / 'scripts' / 'generate_place_recognition_report.py',
        REPO_ROOT / 'scripts' / 'generate_packet_imu_deskew_validation_report.py',
        REPO_ROOT / 'scripts' / 'generate_dynamic_object_filter_report.py',
        REPO_ROOT / 'scripts' / 'preflight_autoware_map_bag.py',
        REPO_ROOT / 'scripts' / 'run_autoware_map_beginner.sh',
        REPO_ROOT / 'scripts' / 'run_autoware_map_from_bag.py',
        REPO_ROOT / 'scripts' / 'diagnose_autoware_map_run.py',
        REPO_ROOT / 'scripts' / 'verify_autoware_map.py',
    ]
    for path in scripts:
        assert path.is_file(), path


def test_contributing_and_issue_templates_exist():
    """Community entry points should exist for benchmark and Autoware reports."""
    contributing = CONTRIBUTING_PATH.read_text(encoding='utf-8')

    assert ISSUE_TEMPLATE_DIR.is_dir()
    assert (ISSUE_TEMPLATE_DIR / 'config.yml').is_file()
    assert (ISSUE_TEMPLATE_DIR / 'benchmark-report.yml').is_file()
    assert (ISSUE_TEMPLATE_DIR / 'autoware-pointcloud-map.yml').is_file()
    assert 'Benchmark Result Submissions' in contributing
    assert 'Autoware Naming And Trademark Guidance' in contributing
    assert 'Autoware-compatible pointcloud map' in contributing
    assert 'official Autoware' in contributing
    assert 'endorsed by the Autoware Foundation' in contributing
    assert 'run_release_readiness_checks.sh' in contributing
    assert 'run_autoware_quickstart.sh' in contributing


def test_public_report_snapshots_exist():
    """Public release docs should have tracked benchmark/report snapshots."""
    assert BENCHMARK_SUMMARY_PATH.is_file()
    assert BENCHMARK_REPORT_PATH.is_file()
    assert STRESS_REPORT_PATH.is_file()
    assert V2_READINESS_PATH.is_file()


def test_release_metadata_and_core_package_versions_match():
    """Release metadata should stay aligned with core package versions."""
    version = VERSION_PATH.read_text(encoding='utf-8').strip()
    changelog = CHANGELOG_PATH.read_text(encoding='utf-8')
    releasing = RELEASING_PATH.read_text(encoding='utf-8')
    release_notes = (REPO_ROOT / 'docs' / 'releases' / f'v{version}.md').read_text(
        encoding='utf-8'
    )
    release_workflow = RELEASE_WORKFLOW.read_text(encoding='utf-8')
    docs_site_workflow = DOCS_SITE_WORKFLOW.read_text(encoding='utf-8')
    mkdocs_config = MKDOCS_CONFIG_PATH.read_text(encoding='utf-8')

    assert version == '0.2.2'
    assert version in changelog
    assert version in releasing
    assert 'v2 beta' in release_notes
    assert 'action-gh-release@v2' in release_workflow
    assert 'mkdocs.yml' in release_workflow
    assert 'docs/index.md' in release_workflow
    assert 'docs/assets/' in release_workflow
    assert 'docs/releases/' in release_workflow
    assert 'docs/autoware-map-authoring.md' in release_workflow
    assert 'docs/autoware-foxglove.md' in release_workflow
    assert 'docs/social/autoware_map_authoring_post_v0.2.2.md' in release_workflow
    assert 'docs/workflows.md' in release_workflow
    assert 'lidarslam/images/autoware_map_loader_proof.png' in release_workflow
    assert 'lidarslam/images/dynamic_object_filter_bag6_summary.svg' in release_workflow
    assert 'lidarslam/images/social_autoware_map_authoring.png' in release_workflow
    assert 'lidarslam/images/social_autoware_map_authoring_demo.mp4' in release_workflow
    assert 'actions/configure-pages@v5' in docs_site_workflow
    assert 'actions/upload-pages-artifact@v4' in docs_site_workflow
    assert 'actions/deploy-pages@v4' in docs_site_workflow
    assert 'python3 -m mkdocs build --strict' in docs_site_workflow
    assert 'README.md' in docs_site_workflow

    package_paths = [
        REPO_ROOT / 'lidarslam' / 'package.xml',
        REPO_ROOT / 'graph_based_slam' / 'package.xml',
        REPO_ROOT / 'lidarslam_msgs' / 'package.xml',
        REPO_ROOT / 'scanmatcher' / 'package.xml',
    ]
    for path in package_paths:
        package_xml = path.read_text(encoding='utf-8')
        assert f'<version>{version}</version>' in package_xml

    assert 'site_name: lidarslam_ros2 Docs' in mkdocs_config
    assert 'site_url: https://rsasaki0109.github.io/lidarslam_ros2/' in mkdocs_config
    assert 'name: material' in mkdocs_config
    assert 'assets/stylesheets/extra.css' in mkdocs_config
    assert 'Autoware-Compatible Map Authoring: autoware-map-authoring.md' in mkdocs_config
    assert 'Autoware Foxglove: autoware-foxglove.md' in mkdocs_config
    assert 'Benchmarking And Release Gate: benchmarking.md' in mkdocs_config
    assert 'v0.2.2: releases/v0.2.2.md' in mkdocs_config
    assert 'v0.2.2 Post Kit: social/autoware_map_authoring_post_v0.2.2.md' in mkdocs_config


def test_docs_cover_autoware_and_release_gate_keywords():
    """The adoption docs should mention the supported operator workflows."""
    autoware_doc = AUTOWARE_QUICKSTART.read_text(encoding='utf-8')
    autoware_map_doc = AUTOWARE_MAP_AUTHORING.read_text(encoding='utf-8')
    autoware_foxglove_doc = AUTOWARE_FOXGLOVE.read_text(encoding='utf-8')
    benchmarking_doc = BENCHMARKING_DOC.read_text(encoding='utf-8')
    comparison_doc = COMPARISON_DOC.read_text(encoding='utf-8')

    assert 'run_autoware_quickstart.sh' in autoware_doc
    assert 'preflight_autoware_map_bag.py' in autoware_doc
    assert 'run_autoware_map_beginner.sh' in autoware_doc
    assert 'run_autoware_map_from_bag.py' in autoware_doc
    assert 'diagnose_autoware_map_run.py' in autoware_doc
    assert 'Autoware-Compatible Map Authoring' in autoware_doc
    assert 'download_ntu_viral_tnp01.sh' in autoware_doc
    assert 'run_rko_lio_graph_autoware_dogfood.sh' in autoware_doc
    assert 'run_graph_slam_pointcloud_map_in_autoware.sh' in autoware_doc
    assert 'projector_type: Local' in autoware_doc
    assert 'Autoware Foxglove' in autoware_doc
    assert 'pointcloud_map/' in autoware_map_doc
    assert 'map_projector_info.yaml' in autoware_map_doc
    assert 'Beginner One-Command Path' in autoware_map_doc
    assert 'preflight_autoware_map_bag.py' in autoware_map_doc
    assert 'run_autoware_map_beginner.sh' in autoware_map_doc
    assert 'run_autoware_map_from_bag.py' in autoware_map_doc
    assert 'run_autoware_quickstart.sh' in autoware_map_doc
    assert 'verify_autoware_map.py' in autoware_map_doc
    assert 'diagnose_autoware_map_run.py' in autoware_map_doc
    assert 'foxglove_bridge' in autoware_foxglove_doc
    assert 'prepare_foxglove_bridge_prefix.sh' in autoware_foxglove_doc
    assert 'run_autoware_pointcloud_map_foxglove.sh' in autoware_foxglove_doc
    assert 'run_graph_slam_pointcloud_map_in_autoware_foxglove.sh' in autoware_foxglove_doc

    workflows_doc = WORKFLOWS_DOC.read_text(encoding='utf-8')
    assert 'Required Input Topics' in workflows_doc
    assert 'sensor_msgs/msg/PointCloud2' in workflows_doc
    assert 'sensor_msgs/msg/Imu' in workflows_doc
    assert 'lidarslam_msgs/msg/MapArray' in workflows_doc
    assert 'wheel odometry / vehicle speed topic fusion' in workflows_doc
    assert 'gnss_topic' in workflows_doc
    assert 'gnss_use_covariance_weighting' in workflows_doc
    assert 'gnss_header_stamp_max_skew_sec' in workflows_doc
    assert 'RTK-like' in workflows_doc
    assert 'inspect_navsatfix_covariance.py' in workflows_doc
    assert 'inspect_applanix_gsof50_quality.py' in workflows_doc
    assert 'convert_applanix_gsof_to_navsatfix_bag.py' in workflows_doc
    assert 'convert_applanix_gsof_to_imu_bag.py' in workflows_doc
    assert 'extract_applanix_gsof49_reference.py' in workflows_doc
    assert 'extract_static_transform_from_bag.py' in workflows_doc
    assert 'prepare_velodyne_pointcloud_overlay.sh' in workflows_doc
    assert 'run_open_data_gnss_smoke.sh' in workflows_doc
    assert 'run_open_data_applanix_velodyne_gnss_smoke.sh' in workflows_doc
    assert 'run_open_data_applanix_velodyne_gnss_benchmark.sh' in workflows_doc
    assert 'run_open_data_classic_path_benchmark_suite.sh' in workflows_doc
    assert 'run_open_data_packet_imu_deskew_validation_matrix.sh' in workflows_doc
    assert 'run_dynamic_object_filter_benchmark.sh' in workflows_doc
    assert 'velodyne_msgs/msg/VelodyneScan' in workflows_doc

    assert 'download_ntu_viral_tnp01.sh' in benchmarking_doc
    assert 'run_rko_lio_graph_benchmark.sh' in benchmarking_doc
    assert 'run_rko_lio_mid360_crossval_benchmark.sh' in benchmarking_doc
    assert 'run_open_data_applanix_velodyne_gnss_benchmark.sh' in benchmarking_doc
    assert 'run_open_data_classic_path_benchmark_suite.sh' in benchmarking_doc
    assert 'run_open_data_packet_imu_deskew_validation_matrix.sh' in benchmarking_doc
    assert 'run_dynamic_object_filter_benchmark.sh' in benchmarking_doc
    assert 'generate_exploration_closeout_report.py' in benchmarking_doc
    assert 'all-sensors-bag6' in benchmarking_doc
    assert 'classic_path_report.md' in benchmarking_doc
    assert 'exploration_closeout_report_20260327.md' in benchmarking_doc
    assert 'generate_classic_path_report.py' in benchmarking_doc
    assert 'run_place_recognition_benchmark.sh' in benchmarking_doc
    assert 'generate_place_recognition_report.py' in benchmarking_doc
    assert 'generate_packet_imu_deskew_validation_report.py' in benchmarking_doc
    assert 'generate_dynamic_object_filter_report.py' in benchmarking_doc
    assert 'run_release_readiness_checks.sh' in benchmarking_doc
    assert 'docs/comparison.md' in benchmarking_doc
    assert 'generate_v2_beta_readiness_report.py' in benchmarking_doc
    assert 'generate_map_authoring_report.py' in benchmarking_doc
    assert 'create_map_authoring_submission_bundle.sh' in benchmarking_doc
    assert 'map_qa_summary.md' in benchmarking_doc
    assert 'generate_stress_validation_report.py' in benchmarking_doc
    assert 'write_aligned_trajectory_metrics.py' in benchmarking_doc
    assert '--write-svg' in benchmarking_doc
    assert '--profile failing' in benchmarking_doc
    assert 'Capability Comparison' in comparison_doc
    assert 'Current Default Position' in comparison_doc
