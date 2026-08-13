#!/usr/bin/env bash
set -euo pipefail

NODE_VERSION="22.18.0"
NODE_ARCHIVE="node-v${NODE_VERSION}-linux-x64.tar.xz"
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORK_ROOT="/tmp/gongxun-software-link-$$"
CACHE_ROOT="/tmp/gongxun-node-cache"
NODE_ROOT="${WORK_ROOT}/node-v${NODE_VERSION}-linux-x64"
HOST_ROOT="${WORK_ROOT}/map-console"
BUILD_ROOT="${WORK_ROOT}/host-tests"

cleanup() {
  rm -rf "${WORK_ROOT}"
}
trap cleanup EXIT

mkdir -p "${WORK_ROOT}" "${HOST_ROOT}" "${CACHE_ROOT}"

printf 'Preparing verified Node.js %s...\n' "${NODE_VERSION}"
if [[ ! -s "${CACHE_ROOT}/${NODE_ARCHIVE}" ||
      ! -s "${CACHE_ROOT}/SHASUMS256-${NODE_VERSION}.txt" ]] ||
   ! (
     cd "${CACHE_ROOT}"
     grep "  ${NODE_ARCHIVE}$" "SHASUMS256-${NODE_VERSION}.txt" |
       sha256sum --check --strict >/dev/null 2>&1
   ); then
  curl --connect-timeout 15 --max-time 180 --retry 2 -fL \
    -o "${CACHE_ROOT}/${NODE_ARCHIVE}" \
    "https://nodejs.org/dist/v${NODE_VERSION}/${NODE_ARCHIVE}"
  curl --connect-timeout 15 --max-time 60 --retry 2 -fL \
    -o "${CACHE_ROOT}/SHASUMS256-${NODE_VERSION}.txt" \
    "https://nodejs.org/dist/v${NODE_VERSION}/SHASUMS256.txt"
fi
(
  cd "${CACHE_ROOT}"
  grep "  ${NODE_ARCHIVE}$" "SHASUMS256-${NODE_VERSION}.txt" |
    sha256sum --check --strict
  tar -xJf "${NODE_ARCHIVE}" -C "${WORK_ROOT}"
)

cmake -S "${PROJECT_ROOT}" -B "${BUILD_ROOT}" -G Ninja \
  -DBUILD_HOST_TESTS=ON
cmake --build "${BUILD_ROOT}"
ctest --test-dir "${BUILD_ROOT}" --output-on-failure

cp "${PROJECT_ROOT}/host/map-console/package.json" \
   "${PROJECT_ROOT}/host/map-console/package-lock.json" \
   "${PROJECT_ROOT}/host/map-console/index.html" \
   "${PROJECT_ROOT}/host/map-console/tsconfig.json" \
   "${PROJECT_ROOT}/host/map-console/tsconfig.app.json" \
   "${PROJECT_ROOT}/host/map-console/tsconfig.node.json" \
   "${PROJECT_ROOT}/host/map-console/vite.config.ts" \
   "${HOST_ROOT}/"
cp -a "${PROJECT_ROOT}/host/map-console/src" \
      "${PROJECT_ROOT}/host/map-console/tests" \
      "${HOST_ROOT}/"

export PATH="${NODE_ROOT}/bin:${PATH}"
export H750_SIMULATOR_PATH="${BUILD_ROOT}/tests/h750_simulator"
cd "${HOST_ROOT}"
npm ci --no-audit --no-fund
npm test
npm run build

printf '\nSoftware link verified:\n'
printf '  Browser Web Serial framing and CRC: OK\n'
printf '  H750 route dispatch and navigation: OK\n'
printf '  ZDT X-firmware CAN commands: OK\n'
printf '  Motor position reply parsing: OK\n'
printf '  Heartbeat fail-safe stop: OK\n'
