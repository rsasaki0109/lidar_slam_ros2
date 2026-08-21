#!/usr/bin/env bash
# Copyright 2026 Sasaki
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#  * Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# This file is sourced by each GT-blind container wrapper. It measures the
# wrapper's current cgroup, whose accounting includes descendants in the
# container, rather than the host-side docker client.

m6a5_write_container_memory_evidence() {
  local process_exit_status="${1:-$?}"
  local out_dir="${OUT_DIR:-}"
  local final_path="${out_dir}/container_memory.json"
  local part_path="${final_path}.part"
  if [[ -z "${out_dir}" ]]; then
    return 1
  fi
  local cgroup_root=/sys/fs/cgroup
  local timestamp
  timestamp="$(date -u +%Y-%m-%dT%H:%M:%SZ 2>/dev/null || printf 'unavailable')"
  local proc_self_cgroup=''
  proc_self_cgroup="$(cat /proc/self/cgroup 2>/dev/null || true)"
  local cgroup_path=''
  while IFS= read -r line; do
    if [[ "${line}" == 0::* ]]; then
      cgroup_path="${line#0::}"
      break
    fi
  done <<<"${proc_self_cgroup}"

  local cgroup_version=0
  local peak_raw=''
  local current_raw=''
  local max_raw=''
  local measurement_status=invalid
  local status_reason='cgroup_v2_memory_files_unavailable'
  if [[ -r "${cgroup_root}/cgroup.controllers" ]]; then
    cgroup_version=2
  fi
  if [[ "${cgroup_version}" == 2 &&
        -r "${cgroup_root}/memory.peak" &&
        -r "${cgroup_root}/memory.current" &&
        -r "${cgroup_root}/memory.max" ]]; then
    peak_raw="$(cat "${cgroup_root}/memory.peak" 2>/dev/null || true)"
    current_raw="$(cat "${cgroup_root}/memory.current" 2>/dev/null || true)"
    max_raw="$(cat "${cgroup_root}/memory.max" 2>/dev/null || true)"
    if [[ ! "${peak_raw}" =~ ^[0-9]+$ ||
          ! "${current_raw}" =~ ^[0-9]+$ ]]; then
      status_reason='cgroup_memory_peak_or_current_non_numeric'
    elif [[ "${max_raw}" == max ]]; then
      if (( peak_raw < current_raw )); then
        status_reason='cgroup_memory_peak_below_current'
      else
        measurement_status=pass
        status_reason='unlimited_memory_max'
      fi
    elif [[ ! "${max_raw}" =~ ^[0-9]+$ ]]; then
      status_reason='cgroup_memory_max_missing_or_non_numeric'
    elif (( max_raw <= 0 )); then
      status_reason='cgroup_memory_max_non_positive'
    elif (( peak_raw < current_raw )); then
      status_reason='cgroup_memory_peak_below_current'
    elif (( current_raw > max_raw )); then
      status_reason='cgroup_memory_current_above_max'
    else
      measurement_status=pass
      status_reason=''
    fi
  fi

  # Only the output tree is made readable. Never chmod /runner or /input.
  local readability_status=pass
  local readability_reason='output_tree_chmod_a+rX'
  if [[ -z "${out_dir}" || "${out_dir}" == / ||
        "${out_dir}" == /runner || "${out_dir}" == /runner/* ||
        "${out_dir}" == /input || "${out_dir}" == /input/* ]]; then
    readability_status=invalid
    readability_reason='unsafe_or_missing_output_dir'
  elif ! chmod -R a+rX -- "${out_dir}" >/dev/null 2>&1; then
    readability_status=invalid
    readability_reason='chmod_output_tree_failed'
  fi
  if [[ "${readability_reason}" == 'unsafe_or_missing_output_dir' ]]; then
    return 1
  fi

  if [[ -e "${final_path}" ]]; then
    return 1
  fi
  mkdir -p "${out_dir}" >/dev/null 2>&1 || true
  if ! python3 - "${part_path}" "${final_path}" \
      "${timestamp}" "${process_exit_status}" "${cgroup_version}" \
      "${cgroup_path}" "${proc_self_cgroup}" "${peak_raw}" \
      "${current_raw}" "${max_raw}" "${measurement_status}" \
      "${status_reason}" "${readability_status}" "${readability_reason}" <<'PY'
import json
import os
from pathlib import Path
import sys


def integer_or_none(value):
    return int(value) if value.isdigit() else None


(part_name, final_name, timestamp, process_status, cgroup_version,
 cgroup_path, proc_self_cgroup, peak_raw, current_raw, max_raw,
 measurement_status, status_reason, readability_status,
 readability_reason) = sys.argv[1:]
part = Path(part_name)
final = Path(final_name)
data = {
    'schema_version': 1,
    'measurement_version': 'm6a5-cgroup-v2-memory-v1',
    'status': measurement_status,
    'status_reason': status_reason or None,
    'measurement_scope': 'container_cgroup_v2',
    'children_included': True,
    'timestamp_utc': timestamp,
    'process_exit_status': integer_or_none(process_status),
    'cgroup_version': int(cgroup_version),
    'cgroup_mount': '/sys/fs/cgroup',
    'cgroup_path': cgroup_path,
    'proc_self_cgroup': proc_self_cgroup,
    'memory_files': {
        'peak': '/sys/fs/cgroup/memory.peak',
        'current': '/sys/fs/cgroup/memory.current',
        'max': '/sys/fs/cgroup/memory.max',
    },
    'memory_peak_raw': peak_raw,
    'memory_current_raw': current_raw,
    'memory_max_raw': max_raw,
    'container_cgroup_peak_bytes': integer_or_none(peak_raw),
    'memory_current_bytes': integer_or_none(current_raw),
    'memory_max_bytes': integer_or_none(max_raw),
    'memory_max_unlimited': max_raw == 'max',
    'output_readability': {
        'status': readability_status,
        'reason': readability_reason,
        'scope': 'OUT_DIR_only',
    },
    'atomic': True,
}
part.parent.mkdir(parents=True, exist_ok=True)
part.write_text(json.dumps(data, indent=2, sort_keys=True) + '\n', encoding='utf-8')
os.replace(part, final)
PY
  then
    return 1
  fi
  chmod a+rX -- "${final_path}" >/dev/null 2>&1 || true
  return 0
}

m6a5_container_exit_trap() {
  local exit_status="${1:-$?}"
  set +e
  m6a5_write_container_memory_evidence "${exit_status}" || true
  return "${exit_status}"
}
