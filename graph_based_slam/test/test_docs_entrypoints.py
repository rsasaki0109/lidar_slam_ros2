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

import json
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
README_PATH = REPO_ROOT / 'README.md'
CONTRIBUTING_PATH = REPO_ROOT / 'CONTRIBUTING.md'
VERSION_PATH = REPO_ROOT / 'VERSION'
CHANGELOG_PATH = REPO_ROOT / 'CHANGELOG.md'
RELEASING_PATH = REPO_ROOT / 'RELEASING.md'
SECURITY_PATH = REPO_ROOT / 'SECURITY.md'
SUPPORT_PATH = REPO_ROOT / 'SUPPORT.md'
CODE_OF_CONDUCT_PATH = REPO_ROOT / 'CODE_OF_CONDUCT.md'
GOVERNANCE_PATH = REPO_ROOT / 'GOVERNANCE.md'
CITATION_PATH = REPO_ROOT / 'CITATION.cff'
MKDOCS_CONFIG_PATH = REPO_ROOT / 'mkdocs.yml'
GITIGNORE_PATH = REPO_ROOT / '.gitignore'
DOCS_INDEX_PATH = REPO_ROOT / 'docs' / 'index.md'
GETTING_STARTED = REPO_ROOT / 'docs' / 'getting-started.md'
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
PRODUCT_CONTRACT_DOC = REPO_ROOT / 'docs' / 'product-contract.md'
GOLDEN_PATH_CLI_DOC = REPO_ROOT / 'docs' / 'golden-path-cli.md'
CLI_COMPATIBILITY_DOC = REPO_ROOT / 'docs' / 'cli-compatibility.md'
CLI_V1_CONTRACT = REPO_ROOT / 'docs' / 'contracts' / 'cli-v1.json'
DISTRIBUTION_DOC = REPO_ROOT / 'docs' / 'distribution.md'
ROSDISTRO_RELEASE_DOC = REPO_ROOT / 'docs' / 'rosdistro-release.md'
OPERATIONAL_RELIABILITY_DOC = REPO_ROOT / 'docs' / 'operational-reliability.md'
BOUNDED_FILESYSTEM_SCHEMA = (
    REPO_ROOT
    / 'docs'
    / 'schemas'
    / 'bounded-filesystem-exhaustion-v1.schema.json'
)
BOUNDED_FILESYSTEM_WORKFLOW = (
    REPO_ROOT
    / '.github'
    / 'workflows'
    / 'bounded-filesystem-exhaustion.yml'
)
SOAK_EVIDENCE_DOC = (
    REPO_ROOT / 'docs' / 'evidence' / 'real-data-soak-2026-07-28.md'
)
SOAK_EVIDENCE_JSON = (
    REPO_ROOT / 'docs' / 'evidence' / 'real-data-soak-2026-07-28.json'
)
DOCKER_FIRST_MAP_EVIDENCE_DOC = (
    REPO_ROOT / 'docs' / 'evidence' / 'docker-first-map-2026-07-28.md'
)
EXTERNAL_FIRST_MAP_DOC = (
    REPO_ROOT / 'docs' / 'external-first-map-validation.md'
)
EXTERNAL_FIRST_MAP_LEDGER = (
    REPO_ROOT
    / 'docs'
    / 'evidence'
    / 'external-first-map-validations.json'
)
EXTERNAL_FIRST_MAP_SCHEMA = (
    REPO_ROOT
    / 'docs'
    / 'schemas'
    / 'external-first-map-validations-v1.schema.json'
)
EXTERNAL_FIRST_MAP_SCRIPT = (
    REPO_ROOT / 'scripts' / 'check_external_first_map_readiness.py'
)
CLI_V1_INSTALL_EVIDENCE_DOC = (
    REPO_ROOT / 'docs' / 'evidence' / 'cli-v1-install-2026-07-28.md'
)
TIMESTAMP_ORDER_EVIDENCE_DOC = (
    REPO_ROOT
    / 'docs'
    / 'evidence'
    / 'timestamp-order-preflight-2026-07-29.md'
)
REAL_DATA_E2E_DOC = REPO_ROOT / 'docs' / 'real-data-e2e.md'
PREFLIGHT_SCHEMA = REPO_ROOT / 'docs' / 'schemas' / 'preflight-v3.schema.json'
DIAGNOSIS_SCHEMA = REPO_ROOT / 'docs' / 'schemas' / 'diagnosis-v1.schema.json'
RUN_MANIFEST_SCHEMA = REPO_ROOT / 'docs' / 'schemas' / 'run-manifest-v1.schema.json'
RUN_MANIFEST_V2_SCHEMA = (
    REPO_ROOT / 'docs' / 'schemas' / 'run-manifest-v2.schema.json'
)
RELEASE_IMAGE_SCHEMA = (
    REPO_ROOT / 'docs' / 'schemas' / 'release-image-v1.schema.json'
)
ROLLBACK_PLAN_SCHEMA = (
    REPO_ROOT / 'docs' / 'schemas' / 'rollback-plan-v1.schema.json'
)
RELEASE_BUNDLE_MANIFEST_SCHEMA = (
    REPO_ROOT
    / 'docs'
    / 'schemas'
    / 'release-bundle-manifest-v1.schema.json'
)
RELEASE_PROMOTION_SCHEMA = (
    REPO_ROOT / 'docs' / 'schemas' / 'release-promotion-v1.schema.json'
)
V09_ROADMAP_DOC = REPO_ROOT / 'docs' / 'roadmap' / 'v0.9.md'
SOCIAL_POST_DOC = REPO_ROOT / 'docs' / 'social' / 'autoware_map_authoring_post_v0.2.2.md'
ISSUE_TEMPLATE_DIR = REPO_ROOT / '.github' / 'ISSUE_TEMPLATE'
PUBLIC_AUTOWARE_ENTRYPOINT = REPO_ROOT / 'scripts' / 'run_autoware_quickstart.sh'
RELEASE_WORKFLOW = REPO_ROOT / '.github' / 'workflows' / 'release.yml'
RELEASE_BUNDLE_SCRIPT = REPO_ROOT / 'scripts' / 'build_release_bundle.py'
RELEASE_PROMOTION_SCRIPT = REPO_ROOT / 'scripts' / 'promote_release_images.py'
DOCKER_WORKFLOW = REPO_ROOT / '.github' / 'workflows' / 'docker.yml'
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


def test_docs_exist_and_are_linked_from_readme():
    """README should link to the main adoption-oriented docs."""
    readme = README_PATH.read_text(encoding='utf-8')
    version = VERSION_PATH.read_text(encoding='utf-8').strip()
    release_notes_path = REPO_ROOT / 'docs' / 'releases' / f'v{version}.md'

    assert CONTRIBUTING_PATH.is_file()
    assert VERSION_PATH.is_file()
    assert CHANGELOG_PATH.is_file()
    assert RELEASING_PATH.is_file()
    assert SECURITY_PATH.is_file()
    assert SUPPORT_PATH.is_file()
    assert CODE_OF_CONDUCT_PATH.is_file()
    assert GOVERNANCE_PATH.is_file()
    assert CITATION_PATH.is_file()
    assert MKDOCS_CONFIG_PATH.is_file()
    assert DOCS_INDEX_PATH.is_file()
    assert GETTING_STARTED.is_file()
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
    assert PRODUCT_CONTRACT_DOC.is_file()
    assert GOLDEN_PATH_CLI_DOC.is_file()
    assert CLI_COMPATIBILITY_DOC.is_file()
    assert CLI_V1_CONTRACT.is_file()
    assert DISTRIBUTION_DOC.is_file()
    assert ROSDISTRO_RELEASE_DOC.is_file()
    assert OPERATIONAL_RELIABILITY_DOC.is_file()
    assert BOUNDED_FILESYSTEM_SCHEMA.is_file()
    assert BOUNDED_FILESYSTEM_WORKFLOW.is_file()
    assert SOAK_EVIDENCE_DOC.is_file()
    assert SOAK_EVIDENCE_JSON.is_file()
    assert EXTERNAL_FIRST_MAP_DOC.is_file()
    assert EXTERNAL_FIRST_MAP_LEDGER.is_file()
    assert EXTERNAL_FIRST_MAP_SCHEMA.is_file()
    assert EXTERNAL_FIRST_MAP_SCRIPT.is_file()
    assert CLI_V1_INSTALL_EVIDENCE_DOC.is_file()
    assert REAL_DATA_E2E_DOC.is_file()
    assert PREFLIGHT_SCHEMA.is_file()
    assert DIAGNOSIS_SCHEMA.is_file()
    assert RUN_MANIFEST_SCHEMA.is_file()
    assert RUN_MANIFEST_V2_SCHEMA.is_file()
    assert RELEASE_IMAGE_SCHEMA.is_file()
    assert ROLLBACK_PLAN_SCHEMA.is_file()
    assert RELEASE_BUNDLE_MANIFEST_SCHEMA.is_file()
    assert RELEASE_PROMOTION_SCHEMA.is_file()
    assert RELEASE_BUNDLE_SCRIPT.is_file()
    assert RELEASE_PROMOTION_SCRIPT.is_file()
    assert V09_ROADMAP_DOC.is_file()
    assert SOCIAL_POST_DOC.is_file()
    assert DOCKER_WORKFLOW.is_file()
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
    assert '(SECURITY.md)' in readme
    assert '(SUPPORT.md)' in readme
    assert '(GOVERNANCE.md)' in readme
    assert '(docs/product-contract.md)' in readme
    assert '(docs/external-first-map-validation.md)' in readme
    assert '(docs/distribution.md)' in readme
    assert '(docs/roadmap/v0.9.md)' in readme
    assert '(docs/getting-started.md)' in readme
    assert '(docs/autoware-map-authoring.md)' in readme
    assert '(docs/autoware-quickstart.md)' in readme
    assert '(docs/autoware-foxglove.md)' in readme
    assert '(docs/workflows.md)' in readme
    assert '(docs/comparison.md)' in readme
    assert '(docs/benchmarking.md)' in readme
    assert 'python3 -m mkdocs serve' in readme
    assert 'lidarslam-map run' in readme
    assert '(lidarslam/images/autoware_map_loader_proof.png)' in readme
    assert 'git clone --recursive https://github.com/rsasaki0109/lidar_slam_ros2.git' in readme
    assert 'rosdep install --from-paths src --ignore-src -r -y' in readme
    # The required-topics table and the dynamic-object-filter figure moved to
    # docs/workflows.md so the README stays narrow; keep the assets on disk
    # (asserted above) and verify the README still routes readers to those docs.
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
        REPO_ROOT / 'scripts' / 'run_radarless_tunnel_ab.sh',
        REPO_ROOT / 'scripts' / 'run_rko_lio_mid360_crossval_benchmark.sh',
        REPO_ROOT / 'scripts' / 'export_mid360_robot_3d_map_preview.py',
        REPO_ROOT / 'scripts' / 'analyze_mid360_robot_public_loop_cloud.py',
        REPO_ROOT / 'scripts' / 'plan_mid360_robot_public_loop_segment_reset.py',
        REPO_ROOT / 'scripts' / 'analyze_mid360_robot_public_segment_map_cloud_alignment.py',
        REPO_ROOT / 'scripts' / 'run_mid360_robot_public_completion_gate.py',
        REPO_ROOT / 'scripts' / 'run_mid360_robot_public_continuous_relocalization_gate.py',
        REPO_ROOT / 'scripts' / 'merge_mid360_robot_public_split_bags.py',
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
    """Community entry points should cover support and structured reports."""
    contributing = CONTRIBUTING_PATH.read_text(encoding='utf-8')

    assert ISSUE_TEMPLATE_DIR.is_dir()
    assert (ISSUE_TEMPLATE_DIR / 'config.yml').is_file()
    assert (ISSUE_TEMPLATE_DIR / 'benchmark-report.yml').is_file()
    assert (ISSUE_TEMPLATE_DIR / 'autoware-pointcloud-map.yml').is_file()
    assert (ISSUE_TEMPLATE_DIR / 'bug-report.yml').is_file()
    assert (ISSUE_TEMPLATE_DIR / 'feature-request.yml').is_file()
    assert (ISSUE_TEMPLATE_DIR / 'sensor-support.yml').is_file()
    assert (ISSUE_TEMPLATE_DIR / 'first-map-validation.yml').is_file()
    assert 'Benchmark Result Submissions' in contributing
    assert 'Autoware Naming And Trademark Guidance' in contributing
    assert 'Autoware-compatible pointcloud map' in contributing
    assert 'official Autoware' in contributing
    assert 'endorsed by the Autoware Foundation' in contributing
    assert 'run_release_readiness_checks.sh' in contributing
    assert 'run_autoware_quickstart.sh' in contributing
    assert '(CODE_OF_CONDUCT.md)' in contributing
    assert '(GOVERNANCE.md)' in contributing
    assert '(SUPPORT.md)' in contributing
    assert '(SECURITY.md)' in contributing
    assert '(docs/product-contract.md)' in contributing

    issue_config = (ISSUE_TEMPLATE_DIR / 'config.yml').read_text(encoding='utf-8')
    assert 'Product Contract And Supported Scope' in issue_config
    assert 'Usage Support' in issue_config
    assert 'Report A Security Vulnerability' in issue_config

    bug_form = (ISSUE_TEMPLATE_DIR / 'bug-report.yml').read_text(
        encoding='utf-8'
    )
    assert 'lidarslam-map doctor <bag> --json' in bug_form
    assert '--preflight-only' not in bug_form

    first_map_form = (
        ISSUE_TEMPLATE_DIR / 'first-map-validation.yml'
    ).read_text(encoding='utf-8')
    for field_id in (
        'independence',
        'documentation_path',
        'release_ref',
        'environment',
        'command',
        'result',
        'verification',
        'findings',
        'privacy',
    ):
        assert f'id: {field_id}' in first_map_form
    assert 'Do not upload map geometry.' in first_map_form


def test_product_contract_has_bounded_official_surface():
    """The beginner product surface should stay explicit and bounded."""
    contract = PRODUCT_CONTRACT_DOC.read_text(encoding='utf-8')
    golden_path = GOLDEN_PATH_CLI_DOC.read_text(encoding='utf-8')
    roadmap = V09_ROADMAP_DOC.read_text(encoding='utf-8')

    assert '## Official entrypoints' in contract
    assert contract.count('| Try the fixed public demo') == 1
    assert contract.count('| Map your own compatible rosbag2') == 1
    assert contract.count('| Reproduce the fixed source-workspace quickstart') == 1
    assert 'lidarslam-map run <rosbag2_dir> --output-dir <dir>' in contract
    assert 'ros2 run lidarslam lidarslam-cli' in contract
    assert 'download_ntu_viral_tnp01.sh && bash scripts/run_autoware_quickstart.sh' in contract
    assert 'Other scripts and ROS' in contract
    assert '`run_manifest.json`' in contract
    assert '`<output>.partial`' in golden_path
    assert 'preflight-v3.schema.json' in golden_path
    assert 'preflight-v2.schema.json' in golden_path
    assert 'preflight-v1.schema.json' in golden_path
    assert 'diagnosis-v1.schema.json' in golden_path
    assert 'run-manifest-v1.schema.json' in golden_path
    assert 'run-manifest-v2.schema.json' in golden_path
    assert '--resume' in golden_path
    assert 'lidarslam-map run --help-all' in golden_path
    compatibility = CLI_COMPATIBILITY_DOC.read_text(encoding='utf-8')
    assert 'Normal help is the operator view' in compatibility
    assert 'view --help-all' in compatibility
    cli_contract = json.loads(CLI_V1_CONTRACT.read_text(encoding='utf-8'))
    assert set(cli_contract['help_modes']) == {'normal', 'all'}
    assert cli_contract['help_modes']['normal']['excludes'] == {
        'stability': ['deprecated'],
        'tiers': ['viewer-runtime'],
    }
    assert 'Resume never starts the SLAM workflow again.' in golden_path
    assert 'existing outputs are never overwritten' in (
        REPO_ROOT / 'scripts' / 'run_autoware_map_from_bag.py'
    ).read_text(encoding='utf-8')
    assert '## Non-goals' in contract
    assert 'No more than three beginner-facing entrypoints' in roadmap
    assert 'Phase 1 — Golden-path UX' in roadmap


def test_generated_output_artifacts_are_local_only():
    """Generated benchmark/report artifacts should stay out of git."""
    gitignore = GITIGNORE_PATH.read_text(encoding='utf-8')
    benchmarking_doc = BENCHMARKING_DOC.read_text(encoding='utf-8')

    assert 'output/' in gitignore
    assert 'benchmark_summary.md' in benchmarking_doc
    assert 'latest_report.html' in benchmarking_doc


def test_release_metadata_and_core_package_versions_match():
    """Release metadata should stay aligned with core package versions."""
    version = VERSION_PATH.read_text(encoding='utf-8').strip()
    changelog = CHANGELOG_PATH.read_text(encoding='utf-8')
    releasing = RELEASING_PATH.read_text(encoding='utf-8')
    release_notes = (REPO_ROOT / 'docs' / 'releases' / f'v{version}.md').read_text(
        encoding='utf-8'
    )
    release_workflow = RELEASE_WORKFLOW.read_text(encoding='utf-8')
    release_bundle_script = RELEASE_BUNDLE_SCRIPT.read_text(encoding='utf-8')
    release_promotion_script = RELEASE_PROMOTION_SCRIPT.read_text(
        encoding='utf-8'
    )
    docs_site_workflow = DOCS_SITE_WORKFLOW.read_text(encoding='utf-8')
    mkdocs_config = MKDOCS_CONFIG_PATH.read_text(encoding='utf-8')

    assert version == '0.7.0'
    assert version in changelog
    assert 'VERSION="$(tr -d \'\\n\' < VERSION)"' in releasing
    assert 'git tag "v${VERSION}"' in releasing
    assert 'Autoware-compatible' in release_notes
    assert 'action-gh-release@v2' in release_workflow
    assert 'docker/setup-buildx-action@v4' in release_workflow
    assert 'docker/login-action@v4' in release_workflow
    assert 'docker/metadata-action@v6' in release_workflow
    assert 'docker/build-push-action@v7' in release_workflow
    assert 'actions/attest@v4' in release_workflow
    assert 'actions/download-artifact@v7' in release_workflow
    assert 'release-image-${{ matrix.ros_distro }}.json' in release_workflow
    assert 'push-by-digest=true' in release_workflow
    assert 'scripts/create_release_image_record.py' in release_workflow
    assert 'scripts/promote_release_images.py' in release_workflow
    assert 'release-promotion.json' in release_workflow
    assert 'refusing to move' in release_promotion_script
    assert 'moving_tag_mutated' in release_promotion_script
    assert 'v<VERSION>-<distro>' in (
        DISTRIBUTION_DOC.read_text(encoding='utf-8')
    )
    assert 'scripts/build_release_bundle.py' in release_workflow
    for bundled_path in (
        'mkdocs.yml',
        'docs/index.md',
        'docs/assets',
        'docs/releases/',
        'docs/autoware-map-authoring.md',
        'docs/product-contract.md',
        'docs/getting-started.md',
        'docs/golden-path-cli.md',
        'docs/cli-compatibility.md',
        'docs/contracts',
        'docs/operational-reliability.md',
        'docs/evidence',
        'docs/schemas',
        'docs/real-data-e2e.md',
        'docs/external-first-map-validation.md',
        'configs/real_data_e2e/driving_slam_mid360_v1.json',
        'docs/distribution.md',
        'docs/rosdistro-release.md',
        'docs/roadmap/v0.9.md',
        'docs/autoware-foxglove.md',
        'docs/social/autoware_map_authoring_post_v0.2.2.md',
        'docs/workflows.md',
        'lidarslam/images/autoware_map_loader_proof.png',
        'lidarslam/images/dynamic_object_filter_bag6_summary.svg',
        'lidarslam/images/social_autoware_map_authoring.png',
        'lidarslam/images/social_autoware_map_authoring_demo.mp4',
    ):
        assert bundled_path in release_bundle_script
    for policy in (
        'SECURITY.md',
        'SUPPORT.md',
        'CODE_OF_CONDUCT.md',
        'GOVERNANCE.md',
        'CITATION.cff',
    ):
        assert policy in release_bundle_script
    assert 'scripts/check_external_first_map_readiness.py' in (
        release_bundle_script
    )
    assert 'actions/configure-pages@v5' in docs_site_workflow
    assert 'actions/upload-pages-artifact@v4' in docs_site_workflow
    assert 'actions/deploy-pages@v4' in docs_site_workflow
    assert 'python3 -m mkdocs build --strict' in docs_site_workflow
    assert 'README.md' in docs_site_workflow

    rosdistro_release = ROSDISTRO_RELEASE_DOC.read_text(encoding='utf-8')
    assert f'maintained through v{version}' in rosdistro_release
    for package_name in (
        'lidarslam_msgs',
        'scanmatcher',
        'graph_based_slam',
        'lidarslam',
    ):
        assert f'| `{package_name}` | {version} |' in rosdistro_release

    package_paths = [
        REPO_ROOT / 'lidarslam' / 'package.xml',
        REPO_ROOT / 'graph_based_slam' / 'package.xml',
        REPO_ROOT / 'lidarslam_msgs' / 'package.xml',
        REPO_ROOT / 'scanmatcher' / 'package.xml',
    ]
    for path in package_paths:
        package_xml = path.read_text(encoding='utf-8')
        assert f'<version>{version}</version>' in package_xml
        changelog_rst = (path.parent / 'CHANGELOG.rst').read_text(encoding='utf-8')
        assert f'Changelog for package {path.parent.name}' in changelog_rst
        assert f'{version} (' in changelog_rst

    assert 'site_name: lidarslam_ros2 Docs' in mkdocs_config
    assert 'site_url: https://rsasaki0109.github.io/lidar_slam_ros2/' in mkdocs_config
    assert 'repo_url: https://github.com/rsasaki0109/lidar_slam_ros2' in mkdocs_config
    assert 'name: material' in mkdocs_config
    assert 'assets/stylesheets/extra.css' in mkdocs_config
    assert 'Getting Started: getting-started.md' in mkdocs_config
    assert 'Product Contract: product-contract.md' in mkdocs_config
    assert 'Golden-path CLI: golden-path-cli.md' in mkdocs_config
    assert 'CLI compatibility: cli-compatibility.md' in mkdocs_config
    assert 'Distribution and installed CLI: distribution.md' in mkdocs_config
    assert 'Operational reliability: operational-reliability.md' in mkdocs_config
    assert (
        'Named-hardware soak evidence: evidence/real-data-soak-2026-07-28.md'
        in mkdocs_config
    )
    assert (
        'Docker first-map evidence: evidence/docker-first-map-2026-07-28.md'
        in mkdocs_config
    )
    assert (
        'Independent first-map validation: external-first-map-validation.md'
        in mkdocs_config
    )
    assert (
        'CLI install evidence: evidence/cli-v1-install-2026-07-28.md'
        in mkdocs_config
    )
    assert 'Pinned real-data E2E: real-data-e2e.md' in mkdocs_config
    assert 'Autoware-Compatible Map Authoring: autoware-map-authoring.md' in mkdocs_config
    assert 'Autoware Foxglove: autoware-foxglove.md' in mkdocs_config
    assert 'Benchmarking And Release Gate: benchmarking.md' in mkdocs_config
    assert f'v{version}: releases/v{version}.md' in mkdocs_config
    assert 'v0.2.2: releases/v0.2.2.md' in mkdocs_config
    assert 'v0.2.2 Post Kit: social/autoware_map_authoring_post_v0.2.2.md' in mkdocs_config
    assert 'rosdistro Binary Release: rosdistro-release.md' in mkdocs_config
    assert 'v0.9 Product Foundation: roadmap/v0.9.md' in mkdocs_config

    citation = CITATION_PATH.read_text(encoding='utf-8')
    assert f'version: {version}' in citation
    assert 'license: BSD-2-Clause' in citation


def test_docs_cover_autoware_and_release_gate_keywords():
    """The adoption docs should mention the supported operator workflows."""
    autoware_doc = AUTOWARE_QUICKSTART.read_text(encoding='utf-8')
    getting_started_doc = GETTING_STARTED.read_text(encoding='utf-8')
    autoware_map_doc = AUTOWARE_MAP_AUTHORING.read_text(encoding='utf-8')
    autoware_foxglove_doc = AUTOWARE_FOXGLOVE.read_text(encoding='utf-8')
    benchmarking_doc = BENCHMARKING_DOC.read_text(encoding='utf-8')
    comparison_doc = COMPARISON_DOC.read_text(encoding='utf-8')
    distribution_doc = DISTRIBUTION_DOC.read_text(encoding='utf-8')
    reliability_doc = OPERATIONAL_RELIABILITY_DOC.read_text(encoding='utf-8')
    timestamp_order_evidence = TIMESTAMP_ORDER_EVIDENCE_DOC.read_text(
        encoding='utf-8'
    )
    real_data_e2e_doc = REAL_DATA_E2E_DOC.read_text(encoding='utf-8')

    assert 'lidarslam-map doctor' in getting_started_doc
    assert 'lidarslam-map run' in getting_started_doc
    assert 'lidarslam-map inspect' in getting_started_doc
    assert 'LIDARSLAM_HOST_UID' in getting_started_doc
    assert 'LIDARSLAM_HOST_GID' in getting_started_doc
    assert 'periodic' in getting_started_doc
    assert 'Bind-mounted output ownership' in distribution_doc
    assert 'Creative Commons Attribution 4.0' in real_data_e2e_doc
    first_map_evidence = DOCKER_FIRST_MAP_EVIDENCE_DOC.read_text(encoding='utf-8')
    assert 'FINAL_' not in first_map_evidence
    assert 'PR #406' in first_map_evidence
    assert 'PR #407' in first_map_evidence
    assert 'independent-user validations' in first_map_evidence
    first_map_program = EXTERNAL_FIRST_MAP_DOC.read_text(encoding='utf-8')
    first_map_ledger = json.loads(
        EXTERNAL_FIRST_MAP_LEDGER.read_text(encoding='utf-8')
    )
    first_map_schema = json.loads(
        EXTERNAL_FIRST_MAP_SCHEMA.read_text(encoding='utf-8')
    )
    assert 'Independent First-map Validation issue form' in first_map_program
    assert '--require-complete' in first_map_program
    assert 'Do not publish map geometry' in first_map_program
    assert first_map_ledger['schema_version'] == 1
    assert first_map_ledger['required_validations'] == 3
    assert isinstance(first_map_ledger['validations'], list)
    assert first_map_schema['properties']['required_validations']['const'] == 3
    assert first_map_schema['definitions']['validation']['properties'][
        'independent_attestation'
    ]['const'] is True
    assert "'docs/evidence'," in RELEASE_BUNDLE_SCRIPT.read_text(
        encoding='utf-8'
    )
    assert (
        'git clone --recursive https://github.com/rsasaki0109/lidar_slam_ros2.git'
        in getting_started_doc
    )
    assert 'run_autoware_quickstart.sh' in autoware_doc
    assert 'Getting Started' in autoware_doc
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
    assert 'run_radarless_tunnel_ab.sh' in benchmarking_doc
    assert 'evaluate_degeneracy_trajectory.py' in benchmarking_doc
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
    assert 'ros2 run lidarslam lidarslam' in distribution_doc
    assert 'ros2 run lidarslam lidarslam-cli' in distribution_doc
    assert 'Humble amd64' in distribution_doc
    assert 'Jazzy amd64' in distribution_doc
    assert ':v<VERSION>-<distro>' in distribution_doc
    assert 'release-image-humble.json' in distribution_doc
    assert 'release-image-jazzy.json' in distribution_doc
    assert 'gh attestation verify' in distribution_doc
    assert 'BuildKit provenance' in distribution_doc
    assert 'There is currently no supported' in distribution_doc
    assert 'rko_lio' in distribution_doc
    assert 'Official PRBonn `rko_lio` packages exist' in (
        distribution_doc
    )
    rosdistro_release_doc = ROSDISTRO_RELEASE_DOC.read_text(encoding='utf-8')
    assert '`rko_lio` | PRBonn `0.3.2-1` is registered' in (
        rosdistro_release_doc
    )
    assert 'main currently has Humble `0.3.0` and Jazzy `0.2.0`' in (
        rosdistro_release_doc
    )
    assert '`PRBonn/rko_lio`' in rosdistro_release_doc
    assert '`ndt_omp_ros2`' in rosdistro_release_doc
    assert 'SIGTERM' in reliability_doc
    assert 'exit `143`' in reliability_doc
    assert 'Automated failure injection' in reliability_doc
    assert 'timestamp reversal' in reliability_doc
    assert '100,000-record per-topic bound' in timestamp_order_evidence
    assert '750,000,000 ns' in timestamp_order_evidence
    assert "'docs/evidence'," in RELEASE_BUNDLE_SCRIPT.read_text(
        encoding='utf-8'
    )
    assert 'disk-pressure' in reliability_doc
    assert '--min-free-space-gib' in reliability_doc
    assert '5 GiB' in reliability_doc
    assert 'No space left on device' in reliability_doc
    assert 'raw_fallocate' in reliability_doc
    assert '32 MiB Docker tmpfs' in reliability_doc
    assert 'run_bounded_filesystem_exhaustion.py' in reliability_doc
    assert 'bounded-filesystem-exhaustion-v1.schema.json' in reliability_doc
    assert 'scripts/run_map_soak.py' in reliability_doc
    assert '--soak-profile one-hour' in reliability_doc
    assert '--hardware-label' in reliability_doc
    assert '--max-peak-rss-mib' in reliability_doc
    assert '--max-iteration-secs' in reliability_doc
    assert '--telemetry-interval-secs' in reliability_doc
    assert 'soak-report-v4.schema.json' in reliability_doc
    assert 'soak-report-v3.schema.json' in reliability_doc
    assert 'iteration_duration_within_budget' in reliability_doc
    assert 'provenance_recorded' in reliability_doc
    assert 'Schemas v1, v2 and v3 remain published' in reliability_doc
    assert 'SIGKILL' in reliability_doc
    assert 'GNU `time`' in reliability_doc
    assert '3,600 or 28,800 seconds' in reliability_doc
    assert '(evidence/real-data-soak-2026-07-28.md)' in reliability_doc
    soak_evidence = SOAK_EVIDENCE_DOC.read_text(encoding='utf-8')
    soak_ledger = json.loads(SOAK_EVIDENCE_JSON.read_text(encoding='utf-8'))
    assert '671 / 671' in soak_evidence
    assert '28,834.115 s' in soak_evidence
    assert soak_ledger['evidence_version'] == 1
    assert soak_ledger['software']['git_commit'] == (
        '0ec55575ffc16eb008e9f24bd6c6f24700bf2f8a'
    )
    assert [run['profile'] for run in soak_ledger['runs']] == [
        'one-hour',
        'eight-hour',
    ]
    assert all(run['status'] == 'passed' for run in soak_ledger['runs'])
    assert all(all(run['checks'].values()) for run in soak_ledger['runs'])
    assert '(operational-reliability.md)' in (
        PRODUCT_CONTRACT_DOC.read_text(encoding='utf-8')
    )
    assert 'driving_slam_mid360_v1' in real_data_e2e_doc
    assert '517088133' in real_data_e2e_doc
    assert '0836c50859bb1af591966b69da166186' in real_data_e2e_doc
    assert 'validate_real_data_e2e.py' in real_data_e2e_doc
    assert '(real-data-e2e.md)' in (
        PRODUCT_CONTRACT_DOC.read_text(encoding='utf-8')
    )


def test_real_data_e2e_workflow_is_pinned_bounded_and_non_geometry():
    """The scheduled public-bag workflow must be pinned and bounded."""
    workflow = (
        REPO_ROOT / '.github' / 'workflows' / 'real-data-e2e.yml'
    ).read_text(encoding='utf-8')

    assert "cron: '17 17 * * *'" in workflow
    assert 'workflow_dispatch:' in workflow
    assert 'container: ros:jazzy-ros-core' in workflow
    assert 'timeout-minutes: 45' in workflow
    assert 'actions/cache@v5' in workflow
    assert '0836c50859bb1af591966b69da166186' in workflow
    assert 'timeout --signal=TERM --kill-after=30s 20m' in workflow
    assert 'lidarslam-map doctor' in workflow
    assert 'lidarslam-map run' in workflow
    assert 'validate_real_data_e2e.py' in workflow
    assert 'output/real-data-e2e/run/map.pcd' not in workflow
    assert 'output/real-data-e2e/run/traj_raw.tum' not in workflow


def test_container_distribution_builds_and_attests_both_supported_distros():
    """Container CI and releases must prove both supported binary paths."""
    docker_workflow = DOCKER_WORKFLOW.read_text(encoding='utf-8')
    release_workflow = RELEASE_WORKFLOW.read_text(encoding='utf-8')

    for workflow in (docker_workflow, release_workflow):
        assert '- humble' in workflow
        assert '- jazzy' in workflow
        assert 'linux/amd64' in workflow or workflow == docker_workflow
        assert 'docker/build-push-action@v7' in workflow
        assert 'actions/attest@v4' in workflow
        assert 'attestations: write' in workflow
        assert 'id-token: write' in workflow
        assert 'sbom:' in workflow
        assert 'provenance:' in workflow
        assert 'lidarslam-map --version' in workflow

    assert "load: ${{ github.event_name == 'pull_request' }}" in docker_workflow
    assert '.github/workflows/release.yml' in docker_workflow
    assert 'needs:' in release_workflow
    assert '- images' in release_workflow
    assert 'v<VERSION>-<distro>' in (
        DISTRIBUTION_DOC.read_text(encoding='utf-8')
    )
    assert '${{ needs.metadata.outputs.tag_name }}-${{ matrix.ros_distro }}' in (
        release_workflow
    )
    assert 'subject-digest: ${{ steps.image.outputs.digest }}' in release_workflow
    assert 'docker buildx imagetools inspect' in release_workflow
    assert 'release_image_evidence/*.json' in release_workflow
    assert 'scripts/plan_image_rollback.py' in release_workflow
    assert 'rollback-plan-${{ matrix.ros_distro }}.json' in release_workflow
    assert 'lidarslam-map rollback-plan' in (
        DISTRIBUTION_DOC.read_text(encoding='utf-8')
    )
