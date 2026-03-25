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

"""Regression tests for the Applanix + Velodyne open-data GNSS smoke flow."""

from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
SMOKE_SCRIPT = (
    REPO_ROOT / 'scripts' / 'run_open_data_applanix_velodyne_gnss_smoke.sh'
)
OVERLAY_SCRIPT = (
    REPO_ROOT / 'scripts' / 'prepare_velodyne_pointcloud_overlay.sh'
)


def test_applanix_velodyne_smoke_script_uses_packet_conversion_and_gnss_sidecar():
    """The wrapper should convert packets and generate GNSS/IMU sidecars."""
    script = SMOKE_SCRIPT.read_text(encoding='utf-8')

    assert 'convert_applanix_gsof_to_navsatfix_bag.py' in script
    assert 'convert_applanix_gsof_to_imu_bag.py' in script
    assert 'velodyne_transform_node' in script
    assert '--gsof49-topic "${GSOF49_TOPIC}"' in script
    assert '--gsof50-topic "${GSOF50_TOPIC}"' in script
    assert '--output-topic "${GNSS_TOPIC}"' in script
    assert '--output-topic "${IMU_TOPIC}"' in script
    assert '--frame-id "${ROBOT_FRAME_ID}"' in script
    assert '--qos-profile-overrides-path "${QOS_FILE}"' in script
    assert '"gnss_topic:=${GNSS_TOPIC}" \\' in script
    assert '"imu_topic:=${IMU_TOPIC}" \\' in script
    assert '"input_cloud:=${POINTS_TOPIC}" \\' in script
    assert '"publish_static_tf:=false" \\' in script
    assert 'POINTS_TOPIC="/open_data/velodyne_points"' in script
    assert '--use-imu BOOL' in script
    assert '--imu-pose-prediction BOOL' in script
    assert 'IMU_TRANSLATION_DESKEW="false"' in script
    assert 'IMU_POSE_PREDICTION="false"' in script
    assert 'if [[ "${USE_IMU,,}" == "true" ]]; then' in script


def test_applanix_velodyne_smoke_script_supports_overlay_bootstrap():
    """The wrapper should be able to bootstrap velodyne_pointcloud."""
    script = SMOKE_SCRIPT.read_text(encoding='utf-8')

    assert 'prepare_velodyne_pointcloud_overlay.sh' in script
    assert '--skip-prepare-overlay' in script
    assert 'ensure_velodyne_overlay' in script
    assert 'resolve_velodyne_msg_dir' in script
    assert 'velodyne_msgs definitions not found' in script
    assert 'velodyne_msgs/msg/VelodyneScan' in script
    assert '${overlay_dir}/src/velodyne/velodyne_msgs/msg' in script
    assert 'default_calibration_for_model' in script
    assert 'VLP16)' in script
    assert '32C|VLP32C)' in script
    assert 'VLS128)' in script


def test_overlay_preparation_script_stays_minimal_and_public():
    """The overlay helper should pull only the minimum public repos/packages."""
    script = OVERLAY_SCRIPT.read_text(encoding='utf-8')

    assert 'https://github.com/ros-drivers/velodyne.git' in script
    assert 'https://github.com/ros/diagnostics.git' in script
    assert '--packages-select diagnostic_updater velodyne_msgs velodyne_pointcloud' in script
