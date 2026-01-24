#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
# shellcheck source=Tools/px4_lockstep_julia/scripts/_lockstep_common.sh
source "${SCRIPT_DIR}/_lockstep_common.sh"

SYS_DIR="${SYSIMAGE_DIR}"
SYS_PATH="${SYSIMAGE}"
SYS_STAMP="${SYSIMAGE_STAMP}"
JULIA_DEPOT_PATH=${JULIA_DEPOT_PATH:-"${REPO_ROOT}/Tools/px4_lockstep_julia/.julia_depot"}
export JULIA_DEPOT_PATH

if [[ "${PX4_LOCKSTEP_SYSIMAGE:-0}" != "1" ]]; then
  echo "PX4_LOCKSTEP_SYSIMAGE is not enabled; skipping sysimage build." >&2
  exit 0
fi

if ! julia --project="${REPO_ROOT}/Tools/px4_lockstep_julia" -e 'using PackageCompiler' >/dev/null 2>&1; then
  echo "PackageCompiler not available in the project; skipping sysimage build." >&2
  exit 1
fi

mkdir -p "${SYS_DIR}"

julia --project="${REPO_ROOT}/Tools/px4_lockstep_julia" -e "using PackageCompiler; create_sysimage([:PX4Lockstep]; sysimage_path=\"${SYS_PATH}\")"

if sys_hash=$(hash_sysimage_inputs); then
  tmp_stamp=$(mktemp "${SYS_STAMP}.tmp.XXXXXX")
  printf '%s\n' "${sys_hash}" > "${tmp_stamp}"
  mv "${tmp_stamp}" "${SYS_STAMP}"
fi

echo "Wrote sysimage: ${SYS_PATH}"
