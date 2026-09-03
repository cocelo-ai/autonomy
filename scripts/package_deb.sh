#!/usr/bin/env bash
# Build one installable Debian package for the LiDAR-only autonomy runtime.
#
# The package contains this workspace's three ROS packages (autonomy_light,
# Super-LIO, and Livox Driver2), the Livox-SDK2 runtime library, and a private
# Nav2 prefix.  Ubuntu and the base ROS installation remain normal Debian
# dependencies so apt can keep security updates and ABI-compatible libraries.
set -Eeuo pipefail

usage() {
  cat <<'EOF'
Usage: ./scripts/package_deb.sh [options]

Create dist/cocelo-autonomy-light_<version>-<revision>_<arch>.deb.

Options:
  --ros-distro NAME       ROS distribution to package for (default: ROS_DISTRO or humble).
  --revision N            Debian package revision (default: 1).
  --skip-build            Reuse the current ./install tree instead of staging a fresh build.
  --no-nav2-bundle        Do not embed the private Nav2 runtime prefix.
  --output PATH           Output .deb path (default: dist/).
  -h, --help              Show this help.

The build host must have /opt/ros/<distro>, the Super-LIO build dependencies,
and Livox-SDK2 installed.  The generated package still declares the base ROS
and Ubuntu shared-library dependencies in its Debian control metadata.
EOF
}

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
ROS_DISTRO_NAME="${ROS_DISTRO:-humble}"
REVISION=1
SKIP_BUILD=false
BUNDLE_NAV2=true
OUTPUT_PATH=""
INSTALL_PREFIX="/opt/cocelo/autonomy-light"
PACKAGE_NAME="cocelo-autonomy-light"
NAV2_EXTERNAL_DEPENDS=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --ros-distro) ROS_DISTRO_NAME="${2:?--ros-distro requires a name}"; shift 2 ;;
    --ros-distro=*) ROS_DISTRO_NAME="${1#*=}"; shift ;;
    --revision) REVISION="${2:?--revision requires a number}"; shift 2 ;;
    --revision=*) REVISION="${1#*=}"; shift ;;
    --skip-build) SKIP_BUILD=true; shift ;;
    --no-nav2-bundle) BUNDLE_NAV2=false; shift ;;
    --output) OUTPUT_PATH="${2:?--output requires a path}"; shift 2 ;;
    --output=*) OUTPUT_PATH="${1#*=}"; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "error: unsupported option: $1" >&2; usage >&2; exit 2 ;;
  esac
done

require_command() {
  command -v "$1" >/dev/null 2>&1 || {
    echo "error: required command not found: $1" >&2
    exit 1
  }
}

[[ "${REVISION}" =~ ^[0-9][A-Za-z0-9.+~]*$ ]] || {
  echo "error: invalid Debian revision: ${REVISION}" >&2
  exit 2
}
[[ -f "/opt/ros/${ROS_DISTRO_NAME}/setup.bash" ]] || {
  echo "error: /opt/ros/${ROS_DISTRO_NAME}/setup.bash not found" >&2
  exit 1
}
for command in colcon cmake dpkg-deb dpkg-architecture fakeroot python3; do
  require_command "${command}"
done

VERSION="$(python3 - "${ROOT}/package.xml" <<'PY'
import sys
import xml.etree.ElementTree as ET
print(ET.parse(sys.argv[1]).getroot().findtext('version'))
PY
)"
[[ -n "${VERSION}" ]] || { echo "error: could not read package version" >&2; exit 1; }
ARCHITECTURE="$(dpkg-architecture -qDEB_HOST_ARCH)"
if [[ -z "${OUTPUT_PATH}" ]]; then
  OUTPUT_PATH="${ROOT}/dist/${PACKAGE_NAME}_${VERSION}-${REVISION}_${ARCHITECTURE}.deb"
elif [[ "${OUTPUT_PATH}" != *.deb ]]; then
  OUTPUT_PATH="${OUTPUT_PATH}/${PACKAGE_NAME}_${VERSION}-${REVISION}_${ARCHITECTURE}.deb"
fi
mkdir -p "$(dirname -- "${OUTPUT_PATH}")"
OUTPUT_PATH="$(cd -- "$(dirname -- "${OUTPUT_PATH}")" && pwd)/$(basename -- "${OUTPUT_PATH}")"

WORK_DIR="$(mktemp -d "${TMPDIR:-/tmp}/autonomy_light_deb.XXXXXX")"
STAGE_DIR="${WORK_DIR}/stage"
BUILD_DIR="${WORK_DIR}/build"
trap 'rm -rf -- "${WORK_DIR}"' EXIT
mkdir -p "${STAGE_DIR}${INSTALL_PREFIX}" "${STAGE_DIR}/DEBIAN" "${BUILD_DIR}"

set +u
# shellcheck source=/dev/null
source "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
set -u

if [[ "${SKIP_BUILD}" == false ]]; then
  echo "autonomy-light: staging ROS install tree"
  colcon build --merge-install \
    --install-base "${STAGE_DIR}${INSTALL_PREFIX}/install" \
    --build-base "${BUILD_DIR}/colcon" \
    --log-base "${BUILD_DIR}/log" \
    --packages-select basic livox_ros_driver2 super_lio autonomy_light \
    --base-paths \
      "${ROOT}" \
      "${ROOT}/src/third_party/livox_ros_driver2" \
      "${ROOT}/src/third_party/super_lio_ros2" \
    --cmake-args \
      -DROS_EDITION=ROS2 \
      -DDISTRO_ROS="${ROS_DISTRO_NAME}" \
      -DPYTHON_EXECUTABLE=/usr/bin/python3 \
      -DPython3_EXECUTABLE=/usr/bin/python3
else
  [[ -f "${ROOT}/install/setup.bash" ]] || {
    echo "error: --skip-build requires ${ROOT}/install/setup.bash" >&2
    exit 1
  }
  echo "autonomy-light: reusing current install tree"
  cp -a "${ROOT}/install" "${STAGE_DIR}${INSTALL_PREFIX}/install"
fi

# Keep the same include/parameter and scripts runtime tree beside install/ as
# the development checkout; launch.sh resolves all non-generated assets there.
cp -a "${ROOT}/include" "${ROOT}/scripts" "${ROOT}/package.xml" \
  "${STAGE_DIR}${INSTALL_PREFIX}/"

bundle_livox_sdk() {
  local sdk_dir="${STAGE_DIR}${INSTALL_PREFIX}/runtime/lib"
  local source_dir=""
  local candidate
  for candidate in /usr/local/lib /usr/local/lib/aarch64-linux-gnu /usr/lib /usr/lib/aarch64-linux-gnu; do
    if compgen -G "${candidate}/liblivox_lidar_sdk_shared.so*" >/dev/null; then
      source_dir="${candidate}"
      break
    fi
  done
  [[ -n "${source_dir}" ]] || {
    echo "error: Livox-SDK2 shared library is not installed; run ./scripts/build.sh first" >&2
    exit 1
  }
  mkdir -p "${sdk_dir}"
  cp -a "${source_dir}"/liblivox_lidar_sdk_shared.so* "${sdk_dir}/"
}

bundle_nav2() {
  require_command apt-cache
  require_command apt-get
  local cache_dir="${BUILD_DIR}/nav2-debs"
  local package_list="${BUILD_DIR}/nav2-packages.txt"
  local runtime_root="${STAGE_DIR}${INSTALL_PREFIX}/runtime"
  local package deb_file
  local -a roots=(
    "ros-${ROS_DISTRO_NAME}-nav2-behaviors"
    "ros-${ROS_DISTRO_NAME}-nav2-bt-navigator"
    "ros-${ROS_DISTRO_NAME}-nav2-controller"
    "ros-${ROS_DISTRO_NAME}-nav2-lifecycle-manager"
    "ros-${ROS_DISTRO_NAME}-nav2-map-server"
    "ros-${ROS_DISTRO_NAME}-nav2-mppi-controller"
    "ros-${ROS_DISTRO_NAME}-nav2-navfn-planner"
    "ros-${ROS_DISTRO_NAME}-nav2-planner"
    "ros-${ROS_DISTRO_NAME}-nav2-regulated-pure-pursuit-controller"
    "ros-${ROS_DISTRO_NAME}-nav2-rviz-plugins"
    "ros-${ROS_DISTRO_NAME}-nav2-smoother"
    "ros-${ROS_DISTRO_NAME}-nav2-smac-planner"
    "ros-${ROS_DISTRO_NAME}-nav2-velocity-smoother"
  )

  echo "autonomy-light: bundling Nav2 runtime packages"
  mkdir -p "${cache_dir}" "${runtime_root}"
  apt-cache depends --recurse --no-recommends --no-suggests \
    --no-conflicts --no-breaks --no-replaces --no-enhances "${roots[@]}" |
    sed -n 's/^[ |]*Depends: //p' |
    grep -E "^ros-${ROS_DISTRO_NAME}-" |
    grep -Ev 'rmw-connext|rmw-cyclonedds|rti-connext|iceoryx|cyclonedds' \
    > "${package_list}" || true
  printf '%s\n' "${roots[@]}" >> "${package_list}"
  sort -u -o "${package_list}" "${package_list}"

  while IFS= read -r package; do
    [[ -n "${package}" ]] || continue
    deb_file="$(find "${cache_dir}" -maxdepth 1 -type f -name "${package}_*.deb" -print -quit)"
    if [[ -z "${deb_file}" ]]; then
      echo "  downloading ${package}"
      (cd "${cache_dir}" && apt-get download "${package}")
      deb_file="$(find "${cache_dir}" -maxdepth 1 -type f -name "${package}_*.deb" -print -quit)"
    fi
    [[ -n "${deb_file}" ]] || { echo "error: could not download ${package}" >&2; exit 1; }
    # Nav2 ROS packages themselves live in the private prefix. Keep only
    # their non-ROS Debian prerequisites in the outer package metadata, so
    # apt installs system libraries without installing a duplicate Nav2 tree.
    local external_depends
    external_depends="$(python3 - "${ROS_DISTRO_NAME}" "$(dpkg-deb -f "${deb_file}" Depends 2>/dev/null || true)" <<'PY'
import sys

distro = sys.argv[1]
text = sys.argv[2].strip()
for clause in (part.strip() for part in text.split(',') if part.strip()):
    alternatives = [item.strip().split()[0] for item in clause.split('|')]
    if alternatives and all(item.startswith(f'ros-{distro}-') for item in alternatives):
        continue
    print(clause)
PY
)"
    [[ -z "${external_depends}" ]] || \
      NAV2_EXTERNAL_DEPENDS+="${NAV2_EXTERNAL_DEPENDS:+, }${external_depends//$'\n'/, }"
    dpkg-deb -x "${deb_file}" "${runtime_root}"
  done < "${package_list}"
}

bundle_livox_sdk
if [[ "${BUNDLE_NAV2}" == true ]]; then
  bundle_nav2
else
  NAV2_EXTERNAL_DEPENDS="ros-${ROS_DISTRO_NAME}-nav2-behaviors, ros-${ROS_DISTRO_NAME}-nav2-bt-navigator, ros-${ROS_DISTRO_NAME}-nav2-controller, ros-${ROS_DISTRO_NAME}-nav2-lifecycle-manager, ros-${ROS_DISTRO_NAME}-nav2-map-server, ros-${ROS_DISTRO_NAME}-nav2-mppi-controller, ros-${ROS_DISTRO_NAME}-nav2-navfn-planner, ros-${ROS_DISTRO_NAME}-nav2-planner, ros-${ROS_DISTRO_NAME}-nav2-regulated-pure-pursuit-controller, ros-${ROS_DISTRO_NAME}-nav2-rviz-plugins, ros-${ROS_DISTRO_NAME}-nav2-smac-planner, ros-${ROS_DISTRO_NAME}-nav2-smoother, ros-${ROS_DISTRO_NAME}-nav2-velocity-smoother"
fi

cat > "${STAGE_DIR}${INSTALL_PREFIX}/autonomy-light" <<EOF
#!/usr/bin/env bash
set -Eeuo pipefail
PREFIX="${INSTALL_PREFIX}"
ROS_DISTRO_NAME="${ROS_DISTRO_NAME}"
set +u
source "/opt/ros/\${ROS_DISTRO_NAME}/setup.bash"
set -u
RUNTIME_PREFIX="\${PREFIX}/runtime/opt/ros/\${ROS_DISTRO_NAME}"
if [[ -d "\${RUNTIME_PREFIX}" ]]; then
  export AMENT_PREFIX_PATH="\${RUNTIME_PREFIX}\${AMENT_PREFIX_PATH:+:\${AMENT_PREFIX_PATH}}"
  export CMAKE_PREFIX_PATH="\${RUNTIME_PREFIX}\${CMAKE_PREFIX_PATH:+:\${CMAKE_PREFIX_PATH}}"
  export LD_LIBRARY_PATH="\${RUNTIME_PREFIX}/lib\${LD_LIBRARY_PATH:+:\${LD_LIBRARY_PATH}}"
  for runtime_lib in "\${RUNTIME_PREFIX}"/lib/*-linux-gnu; do
    [[ ! -d "\${runtime_lib}" ]] || \
      export LD_LIBRARY_PATH="\${runtime_lib}\${LD_LIBRARY_PATH:+:\${LD_LIBRARY_PATH}}"
  done
fi
export LD_LIBRARY_PATH="\${PREFIX}/runtime/lib\${LD_LIBRARY_PATH:+:\${LD_LIBRARY_PATH}}"
set +u
source "\${PREFIX}/install/setup.bash"
set -u
exec "\${PREFIX}/scripts/launch.sh" "\$@"
EOF
chmod 0755 "${STAGE_DIR}${INSTALL_PREFIX}/autonomy-light"

cat > "${STAGE_DIR}${INSTALL_PREFIX}/autonomy-light-doctor" <<EOF
#!/usr/bin/env bash
set -Eeuo pipefail
PREFIX="${INSTALL_PREFIX}"
"\${PREFIX}/autonomy-light" --check
source /opt/ros/${ROS_DISTRO_NAME}/setup.bash
source "\${PREFIX}/install/setup.bash"
ros2 interface show autonomy_light/msg/HeightMap
EOF
chmod 0755 "${STAGE_DIR}${INSTALL_PREFIX}/autonomy-light-doctor"
mkdir -p "${STAGE_DIR}/usr/bin" "${STAGE_DIR}/usr/share/doc/${PACKAGE_NAME}"
ln -s "${INSTALL_PREFIX}/autonomy-light" "${STAGE_DIR}/usr/bin/autonomy-light"
ln -s "${INSTALL_PREFIX}/autonomy-light-doctor" "${STAGE_DIR}/usr/bin/autonomy-light-doctor"
cp "${ROOT}/README.md" "${STAGE_DIR}/usr/share/doc/${PACKAGE_NAME}/README.md"

SHLIB_DEPENDS=""
EXECUTABLES=()
if [[ -d "${STAGE_DIR}${INSTALL_PREFIX}/install/lib" ]]; then
  mapfile -t EXECUTABLES < <(find "${STAGE_DIR}${INSTALL_PREFIX}/install/lib" -type f -perm -0100 -print)
fi
if [[ "${#EXECUTABLES[@]}" -gt 0 ]] && command -v dpkg-shlibdeps >/dev/null 2>&1; then
  SHLIB_DEPENDS="$(dpkg-shlibdeps -O \
    -l"${STAGE_DIR}${INSTALL_PREFIX}/install/lib" \
    -l"${STAGE_DIR}${INSTALL_PREFIX}/runtime/lib" \
    "${EXECUTABLES[@]}" 2>/dev/null |
    sed -n 's/^shlibs:Depends=//p' | paste -sd, - || true)"
fi
BASE_DEPENDS="bash (>= 4.4), python3, python3-yaml, python3-numpy, python3-opencv, ros-${ROS_DISTRO_NAME}-ros-base, ros-${ROS_DISTRO_NAME}-cyclonedds, ros-${ROS_DISTRO_NAME}-pcl-ros, ros-${ROS_DISTRO_NAME}-pcl-conversions, ros-${ROS_DISTRO_NAME}-gtsam"
[[ -z "${SHLIB_DEPENDS}" ]] || BASE_DEPENDS="${BASE_DEPENDS}, ${SHLIB_DEPENDS}"
[[ -z "${NAV2_EXTERNAL_DEPENDS}" ]] || BASE_DEPENDS="${BASE_DEPENDS}, ${NAV2_EXTERNAL_DEPENDS}"
if [[ "${BUNDLE_NAV2}" == true ]]; then
  PACKAGE_DESCRIPTION="Bundled autonomy_light, Super-LIO, Livox Driver2, Livox-SDK2, and Nav2 runtime."
else
  PACKAGE_DESCRIPTION="Bundled autonomy_light, Super-LIO, Livox Driver2, and Livox-SDK2; Nav2 is a dependency."
fi

cat > "${STAGE_DIR}/DEBIAN/control" <<EOF
Package: ${PACKAGE_NAME}
Version: ${VERSION}-${REVISION}
Section: misc
Priority: optional
Architecture: ${ARCHITECTURE}
Maintainer: Cocelo Autonomy Team <sh.ryoo@cocelo.ai>
Depends: ${BASE_DEPENDS}
Description: Cocelo LiDAR-only Super-LIO autonomy runtime
 ${PACKAGE_DESCRIPTION}
EOF

echo "autonomy-light: creating ${OUTPUT_PATH}"
fakeroot dpkg-deb --build --root-owner-group "${STAGE_DIR}" "${OUTPUT_PATH}"
echo "autonomy-light: package created"
echo "  ${OUTPUT_PATH}"
echo "  install: sudo apt install ${OUTPUT_PATH}"
