# Bash completion for the installed lidarslam-map product command.

_LIDARSLAM_MAP_COMMANDS='doctor run inspect view'
_LIDARSLAM_MAP_GLOBAL_OPTIONS='--help --help-all --version'
_LIDARSLAM_MAP_DOCTOR_OPTIONS='--help --help-all --json'
_LIDARSLAM_MAP_RUN_OPTIONS='--help --help-all --profile --output-dir --min-free-space-gib --dry-run --resume --viewer --autoware-core-dir --work-dir --viewer-run-dir --viewer-rebuild --auto-exit-secs --verification --no-verify-map'
_LIDARSLAM_MAP_INSPECT_OPTIONS='--help --help-all --bag --json --write'
_LIDARSLAM_MAP_VIEW_OPTIONS='--help --help-all --viewer --autoware-core-dir --work-dir --runtime-dir --rebuild --auto-exit-secs'

_lidarslam_map_complete_directories() {
  local current="$1"
  if declare -F _filedir >/dev/null 2>&1; then
    _filedir -d
  else
    COMPREPLY=($(compgen -d -- "$current"))
  fi
}

_lidarslam_map_complete() {
  local current previous command options
  COMPREPLY=()
  current="${COMP_WORDS[COMP_CWORD]}"
  previous="${COMP_WORDS[COMP_CWORD - 1]:-}"
  command="${COMP_WORDS[1]:-}"

  if (( COMP_CWORD == 1 )); then
    COMPREPLY=($(compgen -W \
      "${_LIDARSLAM_MAP_COMMANDS} ${_LIDARSLAM_MAP_GLOBAL_OPTIONS}" \
      -- "$current"))
    return
  fi

  case "$previous" in
    --profile)
      COMPREPLY=($(compgen -W \
        'rko_lio_graph_public_path rko_lio_graph_mid360_preset pointcloud_gnss_smoke packet_applanix_smoke' \
        -- "$current"))
      return
      ;;
    --verification)
      COMPREPLY=($(compgen -W 'required off' -- "$current"))
      return
      ;;
    --viewer)
      if [[ "$command" == 'view' ]]; then
        COMPREPLY=($(compgen -W 'autoware foxglove' -- "$current"))
      else
        COMPREPLY=($(compgen -W 'none autoware foxglove' -- "$current"))
      fi
      return
      ;;
    --output-dir|--bag|--autoware-core-dir|--work-dir|--viewer-run-dir|--runtime-dir)
      _lidarslam_map_complete_directories "$current"
      return
      ;;
  esac

  case "$command" in
    doctor)
      options="$_LIDARSLAM_MAP_DOCTOR_OPTIONS"
      ;;
    run)
      options="$_LIDARSLAM_MAP_RUN_OPTIONS"
      ;;
    inspect)
      options="$_LIDARSLAM_MAP_INSPECT_OPTIONS"
      ;;
    view)
      options="$_LIDARSLAM_MAP_VIEW_OPTIONS"
      ;;
    *)
      return
      ;;
  esac

  if [[ "$current" == -* ]]; then
    COMPREPLY=($(compgen -W "$options" -- "$current"))
    return
  fi
  _lidarslam_map_complete_directories "$current"
}

complete -F _lidarslam_map_complete lidarslam-map
