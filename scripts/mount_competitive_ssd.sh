#!/usr/bin/env bash

# Copyright 2026 Sasaki
# All rights reserved.

# Mount the frozen competitive-evidence SSD at its profile-bound path.
#
# The UUID and target are deliberately constants.  ``--check`` performs only
# read-only observations and never invokes sudo, udisksctl, or mount.  The
# default action is conservative: a wrong existing mount, an unsafe target,
# or any device/filesystem mismatch fails before an unmount or mount is
# attempted.

set -euo pipefail

readonly SSD_UUID="3b5dc9b7-c4de-4cf2-a892-00b2c063f34e"
readonly TARGET="/media/sasaki/aiueo1"
readonly UUID_PATH="/dev/disk/by-uuid/${SSD_UUID}"
readonly MOUNT_OPTIONS="rw,nosuid,nodev"


die() {
    printf 'mount_competitive_ssd: ERROR: %s\n' "$*" >&2
    exit 1
}


usage() {
    cat <<'EOF'
Usage: mount_competitive_ssd.sh [--check|--apply]

  --check  Verify the UUID, filesystem, mount target, and options without
           changing mount state.  It exits non-zero when the SSD is not
           already mounted at the required target.
  --apply  Mount the pinned UUID at the required target (the default).
EOF
}


MODE="apply"
case "${1:-}" in
    "") ;;
    --check) MODE="check" ;;
    --apply) MODE="apply" ;;
    --help|-h)
        usage
        exit 0
        ;;
    *)
        usage >&2
        die "unknown option: $1"
        ;;
esac


stat_kind() {
    local path="$1"
    local value
    value=$(stat -c '%F' -- "$path" 2>/dev/null) || {
        die "cannot stat ${path}"
    }
    [[ "$value" != *$'\n'* && -n "$value" ]] || die "invalid stat result for ${path}"
    printf '%s\n' "$value"
}


validate_fixed_path_parents() {
    local current="/"
    local component kind
    for component in media sasaki aiueo1; do
        current="${current%/}/${component}"
        kind=$(stat_kind "$current")
        [[ "$kind" == "directory" ]] || {
            die "mount target parent is not a real directory: ${current}"
        }
    done
}


validate_device() {
    local link_kind device device_kind row row_name row_type row_fs row_uuid extra

    link_kind=$(stat_kind "$UUID_PATH")
    [[ "$link_kind" == "symbolic link" ]] || {
        die "UUID path is not a symlink: ${UUID_PATH}"
    }

    device=$(readlink -e -- "$UUID_PATH" 2>/dev/null) || {
        die "UUID symlink does not resolve: ${UUID_PATH}"
    }
    [[ "$device" == /dev/* && "$device" != *$'\n'* ]] || {
        die "UUID symlink resolves outside /dev: ${device}"
    }
    device_kind=$(stat_kind "$device")
    [[ "$device_kind" == "block special file" ]] || {
        die "UUID symlink does not resolve to a block device: ${device}"
    }

    row=$(lsblk -nrpo NAME,TYPE,FSTYPE,UUID -- "$UUID_PATH" 2>/dev/null) || {
        die "lsblk could not inspect ${UUID_PATH}"
    }
    [[ -n "$row" && "$row" != *$'\n'* ]] || {
        die "lsblk returned no unique device row for ${UUID_PATH}"
    }
    read -r row_name row_type row_fs row_uuid extra <<< "$row"
    [[ -z "${extra:-}" && "$row_name" == "$device" ]] || {
        die "lsblk device identity mismatch for ${UUID_PATH}"
    }
    [[ -n "$row_type" && "$row_fs" == "ext4" && "$row_uuid" == "$SSD_UUID" ]] || {
        die "UUID/device filesystem metadata is not the pinned ext4 volume"
    }

    printf '%s\n' "$device"
}


validate_target_directory() {
    local target_kind unsafe_entry

    validate_fixed_path_parents
    target_kind=$(stat_kind "$TARGET")
    [[ "$target_kind" == "directory" ]] || {
        die "required mount target is not a real directory: ${TARGET}"
    }

    unsafe_entry=$(find -P "$TARGET" -xdev -mindepth 1 ! -type d -print -quit 2>/dev/null) || {
        die "cannot inspect mount target contents: ${TARGET}"
    }
    [[ -z "$unsafe_entry" ]] || {
        die "mount target is not an empty directory tree: ${unsafe_entry}"
    }
}


mount_at_target() {
    local record rc=0
    record=$(findmnt -rn -M "$TARGET" -o TARGET,SOURCE,FSTYPE,OPTIONS,UUID 2>/dev/null) || rc=$?
    if (( rc != 0 )); then
        (( rc == 1 )) || die "findmnt failed while inspecting ${TARGET} (rc=${rc})"
        return 1
    fi
    [[ -n "$record" && "$record" != *$'\n'* ]] || {
        die "findmnt returned an ambiguous target record"
    }
    printf '%s\n' "$record"
}


mounts_for_device() {
    local device="$1" record rc=0
    record=$(findmnt -rn -S "$device" -o TARGET,SOURCE,FSTYPE,OPTIONS,UUID 2>/dev/null) || rc=$?
    if (( rc != 0 )); then
        (( rc == 1 )) || die "findmnt failed while inspecting ${device} (rc=${rc})"
        return 1
    fi
    [[ -n "$record" ]] || die "findmnt returned an empty mounted-device record"
    printf '%s\n' "$record"
}


has_mount_option() {
    local options="$1" required="$2"
    case ",${options}," in
        *,"${required}",*) return 0 ;;
        *) return 1 ;;
    esac
}


validate_mount_identity() {
    local record="$1" device="$2" expected_target="$3"
    local target source filesystem options uuid extra source_device

    [[ "$record" != *$'\n'* ]] || die "findmnt produced multiple records where one was required"
    read -r target source filesystem options uuid extra <<< "$record"
    [[ -z "${extra:-}" && -n "$target" && -n "$source" && -n "$filesystem" &&
        -n "$options" && -n "$uuid" ]] || {
        die "findmnt record is malformed"
    }
    [[ "$target" == "$expected_target" ]] || {
        die "mount target mismatch: expected ${expected_target}, observed ${target}"
    }
    [[ "$filesystem" == "ext4" && "$uuid" == "$SSD_UUID" ]] || {
        die "mounted filesystem/UUID is not the pinned ext4 volume"
    }

    source_device=$(readlink -e -- "$source" 2>/dev/null) || {
        die "mounted source does not resolve: ${source}"
    }
    [[ "$source_device" == "$device" ]] || {
        die "mounted source mismatch: expected ${device}, observed ${source_device}"
    }
}


validate_mount_record() {
    local record="$1" device="$2" expected_target="$3"
    local target source filesystem options uuid extra

    validate_mount_identity "$record" "$device" "$expected_target"
    read -r target source filesystem options uuid extra <<< "$record"
    has_mount_option "$options" rw || die "mounted filesystem is not read-write"
    has_mount_option "$options" nosuid || die "mounted filesystem lacks nosuid"
    has_mount_option "$options" nodev || die "mounted filesystem lacks nodev"
    has_mount_option "$options" ro && die "mounted filesystem is read-only"
    has_mount_option "$options" suid && die "mounted filesystem allows suid"
    has_mount_option "$options" dev && die "mounted filesystem allows device nodes"
    return 0
}


validate_target_mount() {
    local device="$1" record
    record=$(mount_at_target) || die "SSD is not mounted at required target ${TARGET}"
    validate_mount_record "$record" "$device" "$TARGET"
    printf '%s\n' "$record"
}


assert_target_unmounted() {
    local record rc=0
    record=$(mount_at_target) || rc=$?
    if (( rc == 0 )); then
        die "mount target became occupied before the mount operation"
    fi
    (( rc == 1 )) || die "unable to prove mount target is unmounted (rc=${rc})"
}


source_has_mounts() {
    local device="$1" records rc=0
    records=$(mounts_for_device "$device") || rc=$?
    if (( rc == 1 )); then
        return 1
    fi
    [[ "$rc" == 0 ]] || die "unable to inspect mounts for ${device}"
    while IFS= read -r record; do
        [[ -n "$record" ]] || continue
        # A desktop automounter may use weaker options.  Prove that this is
        # the pinned device before unmounting it; enforce the hardened options
        # only after mounting it at the profile-bound target ourselves.
        validate_mount_identity "$record" "$device" "${record%% *}"
    done <<< "$records"
    return 0
}


main() {
    local device target_record target_rc=0 source_mounted=0 final_record

    device=$(validate_device)

    target_record=$(mount_at_target) || target_rc=$?
    if (( target_rc != 0 && target_rc != 1 )); then
        die "unable to inspect required target mount (rc=${target_rc})"
    fi
    if (( target_rc == 0 )); then
        validate_mount_record "$target_record" "$device" "$TARGET"
        final_record=$(validate_target_mount "$device")
        printf '%s\n' "$final_record"
        return 0
    fi

    if source_has_mounts "$device"; then
        source_mounted=1
    fi

    # This is read-only and intentionally runs for --check as well: a
    # successful check must prove that the eventual mount point is safe to
    # use, not merely that it is currently unmounted.
    validate_target_directory

    if [[ "$MODE" == "check" ]]; then
        if (( source_mounted )); then
            die "pinned SSD is mounted elsewhere; --check is read-only and will not move it"
        fi
        die "pinned SSD is not mounted at required target ${TARGET}; --check is read-only"
    fi

    if (( source_mounted )); then
        command -v udisksctl >/dev/null 2>&1 || die "udisksctl is required to unmount the pinned SSD"
        printf 'Unmounting pinned SSD before moving it to %s\n' "$TARGET" >&2
        udisksctl unmount --block-device "$UUID_PATH" || {
            die "udisksctl could not unmount ${UUID_PATH}; no mount was attempted"
        }
        source_has_mounts "$device" && {
            die "pinned SSD remains mounted after udisksctl unmount"
        }
    fi

    assert_target_unmounted
    validate_target_directory
    command -v sudo >/dev/null 2>&1 || die "sudo is required for the mount operation"
    printf 'Mounting %s at %s\n' "$UUID_PATH" "$TARGET" >&2
    sudo mount -o "$MOUNT_OPTIONS" "$UUID_PATH" "$TARGET" || {
        die "mount failed; target was not changed by this script"
    }

    final_record=$(validate_target_mount "$device")
    printf '%s\n' "$final_record"
}


main "$@"
