#!/usr/bin/env bash
# Build the ros2os system image for comma devices (comma four "mici", comma 3X "tizi").
#
#   ros2os/build.sh                      # full build
#   ROS2OS_BASE_URL=https://host/ros2os ros2os/build.sh   # embed a download URL in the manifest
#   EXTRA_APT_PACKAGES="ros-jazzy-nav2-bringup" ros2os/build.sh  # bake more packages into the image
#
# Steps (each is skipped when its output already exists; `--clean` starts over):
#   1. fetch the AGNOS release pinned in openpilot/system/hardware/comma/agnos.json,
#      verify its hashes and export the system partition's root filesystem
#   2. import it as the arm64 Docker base image
#   3. docker build: ROS 2 + openpilot camerad/panda + the comma ROS 2 drivers + services
#   4. turn the container filesystem into an ext4 system image (same size/features as AGNOS),
#      sparse + xz compress it and write an AGNOS-format manifest for the installer
#
# Requirements: Linux, Docker with BuildKit, ~40 GB free disk. On x86_64 hosts arm64
# emulation must be enabled once:  docker run --privileged --rm tonistiigi/binfmt --install arm64
set -euo pipefail

ROS2OS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$ROS2OS_DIR/.." && pwd)"
OUT_DIR="${OUT_DIR:-$ROS2OS_DIR/build}"
AGNOS_MANIFEST="${AGNOS_MANIFEST:-$REPO_DIR/openpilot/system/hardware/comma/agnos.json}"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
ROS2OS_BASE_URL="${ROS2OS_BASE_URL:-}"
RUN_TESTS="${RUN_TESTS:-1}"
VERSION="$(cat "$ROS2OS_DIR/VERSION")"
GIT_COMMIT="$(git -C "$REPO_DIR" rev-parse --short HEAD 2>/dev/null || echo unknown)"
TOOLS_IMAGE="ros2os-tools:latest"

log() { printf '\n\033[1;34m==> %s\033[0m\n' "$*"; }
die() { printf '\033[1;31merror: %s\033[0m\n' "$*" >&2; exit 1; }

if [[ "${1:-}" == "--clean" ]]; then
  rm -rf "$OUT_DIR"
  shift
fi
[[ $# -eq 0 ]] || die "usage: $0 [--clean]"

# ---------- preflight ----------
log "preflight"
command -v docker >/dev/null || die "docker is required"
command -v python3 >/dev/null || die "python3 is required"
docker info >/dev/null 2>&1 || die "cannot talk to the docker daemon"
[[ -f "$AGNOS_MANIFEST" ]] || die "AGNOS manifest not found: $AGNOS_MANIFEST"
for sub in msgq_repo/SConscript panda/SConscript opendbc_repo/pyproject.toml rednose_repo/rednose/SConscript; do
  [[ -e "$REPO_DIR/$sub" ]] || die "submodule file $sub missing; run: git submodule update --init --recursive"
done
if [[ "$(uname -m)" != "aarch64" ]]; then
  if ! docker run --rm --platform linux/arm64 ubuntu:24.04 uname -m 2>/dev/null | grep -q aarch64; then
    die "arm64 emulation is not available; run: docker run --privileged --rm tonistiigi/binfmt --install arm64"
  fi
fi
mkdir -p "$OUT_DIR"

log "building tools image"
docker build -q -t "$TOOLS_IMAGE" -f "$ROS2OS_DIR/builder/Dockerfile.tools" "$ROS2OS_DIR/builder" >/dev/null

tools() {  # run a script from builder/scripts in the host-arch tools container
  docker run --rm "$@"
}

# ---------- 1. AGNOS root filesystem ----------
eval "$(python3 "$ROS2OS_DIR/builder/scripts/manifest.py" system-env "$AGNOS_MANIFEST")"
AGNOS_TAG="ros2os-agnos:${AGNOS_HASH_RAW:0:12}"
if [[ ! -s "$OUT_DIR/agnos/rootfs.tar" || ! -s "$OUT_DIR/agnos/dumpe2fs.txt" ]]; then
  log "fetching and extracting AGNOS system image (${AGNOS_HASH_RAW:0:12})"
  # privileged for the read-only loop mount (keeps xattrs such as file capabilities)
  tools --privileged -v "$OUT_DIR:/out" -v "$ROS2OS_DIR/builder/scripts:/scripts:ro" \
    -e URL="$AGNOS_URL" -e HASH="$AGNOS_HASH" -e HASH_RAW="$AGNOS_HASH_RAW" -e SPARSE="$AGNOS_SPARSE" -e OUT=/out \
    "$TOOLS_IMAGE" /scripts/extract_agnos.sh
fi
AGNOS_VERSION="$(cat "$OUT_DIR/agnos/VERSION" 2>/dev/null || echo unknown)"
echo "AGNOS version: $AGNOS_VERSION"

# ---------- 2. base image ----------
if ! docker image inspect "$AGNOS_TAG" >/dev/null 2>&1; then
  log "importing AGNOS root filesystem as $AGNOS_TAG"
  docker import --platform linux/arm64 --change 'CMD ["/bin/bash"]' "$OUT_DIR/agnos/rootfs.tar" "$AGNOS_TAG"
fi

# ---------- 3. root filesystem ----------
log "building the ros2os root filesystem (ROS 2 $ROS_DISTRO); this takes a while under emulation"
ROOTFS_TAG="ros2os-rootfs:$VERSION"
DOCKER_BUILDKIT=1 docker build --platform linux/arm64 \
  -f "$ROS2OS_DIR/builder/Dockerfile" \
  --build-arg BASE_IMAGE="$AGNOS_TAG" \
  --build-arg ROS_DISTRO="$ROS_DISTRO" \
  --build-arg EXTRA_APT_PACKAGES="${EXTRA_APT_PACKAGES:-}" \
  --build-arg ROS2OS_VERSION="$VERSION" \
  --build-arg GIT_COMMIT="$GIT_COMMIT" \
  --build-arg AGNOS_VERSION="$AGNOS_VERSION" \
  --build-arg RUN_TESTS="$RUN_TESTS" \
  -t "$ROOTFS_TAG" "$REPO_DIR"

log "exporting the root filesystem"
cid="$(docker create --platform linux/arm64 "$ROOTFS_TAG")"
trap 'docker rm -f "$cid" >/dev/null 2>&1 || true' EXIT
docker export "$cid" -o "$OUT_DIR/rootfs.tar"
docker rm "$cid" >/dev/null
trap - EXIT

# ---------- 4. system image + manifest ----------
log "creating the ext4 system image"
tools -v "$OUT_DIR:/out" -v "$ROS2OS_DIR/builder/scripts:/scripts:ro" -e OUT=/out -e SIZE="$AGNOS_SIZE" \
  "$TOOLS_IMAGE" /scripts/make_image.sh

log "writing the install manifest"
python3 "$ROS2OS_DIR/builder/scripts/manifest.py" write "$AGNOS_MANIFEST" "$OUT_DIR/system.json" \
  --base-url "$ROS2OS_BASE_URL" --version "$VERSION" --output "$OUT_DIR/ros2os-manifest.json"

SYSTEM_FILE="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["file"])' "$OUT_DIR/system.json")"
log "done"
cat <<EOF
  system image : $OUT_DIR/$SYSTEM_FILE
  raw image    : $OUT_DIR/system.img (for EDL/fastboot flashing)
  manifest     : $OUT_DIR/ros2os-manifest.json
  based on     : AGNOS $AGNOS_VERSION, ROS 2 $ROS_DISTRO, ros2os $VERSION ($GIT_COMMIT)

Install on a device running AGNOS (see ros2os/README.md):
  scp $OUT_DIR/ros2os-manifest.json $OUT_DIR/$SYSTEM_FILE ros2os/installer/install.py comma@<device>:/data/tmp/
  ssh comma@<device> 'cd /data/tmp && python3 install.py ros2os-manifest.json --reboot'
EOF
