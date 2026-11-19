#!/usr/bin/env bash
# Runs in the tools container. Downloads the pinned AGNOS system image, verifies both
# manifest hashes and exports the root filesystem as $OUT/agnos/rootfs.tar.
set -euo pipefail
: "${URL:?}" "${HASH:?}" "${HASH_RAW:?}" "${SPARSE:?}" "${OUT:?}"

cache="$OUT/cache"
mkdir -p "$cache" "$OUT/agnos"
xzf="$cache/$(basename "$URL")"

if [[ ! -s "$xzf" ]]; then
  echo "downloading $URL"
  curl -fL --retry 5 --retry-delay 5 -o "$xzf.part" "$URL"
  mv "$xzf.part" "$xzf"
fi

echo "verifying $(basename "$xzf")"
got="$(xz -dc "$xzf" | sha256sum | cut -d' ' -f1)"
if [[ "$got" != "$HASH" ]]; then
  echo "hash mismatch: got $got, manifest says $HASH" >&2
  rm -f "$xzf"
  exit 1
fi

raw="$cache/agnos-system.raw.img"
if [[ "$SPARSE" == "true" ]]; then
  xz -dc "$xzf" > "$cache/agnos-system.sparse.img"
  simg2img "$cache/agnos-system.sparse.img" "$raw"
  rm -f "$cache/agnos-system.sparse.img"
else
  xz -dc "$xzf" > "$raw"
fi
got="$(sha256sum "$raw" | cut -d' ' -f1)"
if [[ "$got" != "$HASH_RAW" ]]; then
  echo "raw image hash mismatch: got $got, manifest says $HASH_RAW" >&2
  exit 1
fi

# filesystem parameters, reused so the kernel sees the same ext4 feature set
dumpe2fs -h "$raw" > "$OUT/agnos/dumpe2fs.txt" 2>/dev/null

tar_opts=(--numeric-owner --xattrs --xattrs-include='*' --acls)
mnt="${MNT_DIR:-/mnt/agnos}"
mkdir -p "$mnt"
if mount -o loop,ro "$raw" "$mnt" 2>/dev/null; then
  trap 'umount "$mnt"' EXIT
  src="$mnt"
else
  # no loop devices (rootless docker): debugfs keeps owners and modes but not xattrs
  echo "WARNING: loop mount failed, extracting with debugfs (file capabilities are not preserved)" >&2
  src="$cache/agnos-root"
  rm -rf "$src"
  mkdir -p "$src"
  debugfs -R "rdump / $src" "$raw" >/dev/null
fi

cp "$src/VERSION" "$OUT/agnos/VERSION" 2>/dev/null || echo unknown > "$OUT/agnos/VERSION"
echo "exporting AGNOS $(cat "$OUT/agnos/VERSION") root filesystem"
tar "${tar_opts[@]}" -C "$src" -cpf "$OUT/agnos/rootfs.tar.part" .
mv "$OUT/agnos/rootfs.tar.part" "$OUT/agnos/rootfs.tar"
rm -f "$raw"
