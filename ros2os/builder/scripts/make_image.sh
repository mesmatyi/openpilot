#!/usr/bin/env bash
# Runs in the tools container. Turns $OUT/rootfs.tar (docker export of the ros2os root
# filesystem) into an ext4 system image with the AGNOS partition image size and ext4
# feature set, then produces the sparse + xz image and its hashes ($OUT/system.json).
set -euo pipefail
: "${OUT:?}" "${SIZE:?}"

root="${ROOTFS_DIR:-/tmp/rootfs}"
rm -rf "$root"
mkdir -p "$root"
tar --numeric-owner --xattrs --xattrs-include='*' -xpf "$OUT/rootfs.tar" -C "$root"
rm -f "$root/.dockerenv"
# docker bind-mounts these during the build and exports placeholders; take AGNOS' versions
for f in etc/hosts etc/hostname etc/resolv.conf; do
  rm -f "$root/$f"
  tar --numeric-owner -xpf "$OUT/agnos/rootfs.tar" -C "$root" "./$f" 2>/dev/null || true
done

info="$OUT/agnos/dumpe2fs.txt"
field() { sed -n "s/^$1:[[:space:]]*//p" "$info" | head -1; }
features="$(field 'Filesystem features' | tr ' ' '\n' | grep -vE '^(needs_recovery|orphan_present|)$' | paste -sd, -)"
block_size="$(field 'Block size')"
inode_size="$(field 'Inode size')"
label="$(field 'Filesystem volume name')"
[[ "$label" == "<none>" ]] && label=""
uuid="$(field 'Filesystem UUID')"  # same UUID as AGNOS, in case anything mounts by UUID
[[ -n "$features" && -n "$block_size" && -n "$inode_size" ]] || { echo "could not read AGNOS filesystem parameters" >&2; exit 1; }
(( SIZE % block_size == 0 )) || { echo "size $SIZE is not a multiple of the block size $block_size" >&2; exit 1; }

files="$(find "$root" -xdev | wc -l)"
inodes=$(( files * 3 / 2 + 50000 ))
used_kb="$(du -sk --apparent-size "$root" | cut -f1)"
echo "root filesystem: $files files, $((used_kb / 1024)) MiB; image: $((SIZE / 1024 / 1024)) MiB"
echo "ext4: block size $block_size, inode size $inode_size, label '${label}', features $features"

img="$OUT/system.img"
rm -f "$img"
truncate -s "$SIZE" "$img"
if ! mkfs.ext4 -F -q -t ext4 -b "$block_size" -I "$inode_size" -N "$inodes" -O "none,$features" \
     ${label:+-L "$label"} ${uuid:+-U "$uuid"} -E root_owner=0:0 -d "$root" "$img" $(( SIZE / block_size )); then
  echo "mkfs.ext4 failed. If the root filesystem does not fit, build with fewer packages (see README, 'Image size')." >&2
  exit 1
fi
[[ "$(stat -c %s "$img")" == "$SIZE" ]] || { echo "unexpected image size" >&2; exit 1; }
e2fsck -fn "$img" >/dev/null
echo "free space in image: $(dumpe2fs -h "$img" 2>/dev/null | awk -F: '/^Free blocks/ {gsub(/ /,"",$2); print int($2*'"$block_size"'/1048576)}') MiB"

echo "sparsing and compressing"
img2simg "$img" "$OUT/system.sparse.img"
# openpilot's agnos.py (and older comma flashing tools) do not expand DONT_CARE chunks
python3 - "$OUT/system.sparse.img" <<'EOF'
import struct, sys
with open(sys.argv[1], "rb") as f:
  _, _, _, file_hdr, _, _, _, chunks, _ = struct.unpack("<IHHHHIIII", f.read(28))
  f.seek(file_hdr)
  for _ in range(chunks):
    kind, _, _, total = struct.unpack("<HHII", f.read(12))
    if kind == 0xCAC3:
      sys.exit("sparse image contains DONT_CARE chunks")
    f.seek(total - 12, 1)
EOF
hash_raw="$(sha256sum "$img" | cut -d' ' -f1)"
hash="$(sha256sum "$OUT/system.sparse.img" | cut -d' ' -f1)"
file="system-$hash_raw.img.xz"
xz -T0 -6 -c "$OUT/system.sparse.img" > "$OUT/$file.part"
mv "$OUT/$file.part" "$OUT/$file"
rm -f "$OUT/system.sparse.img"
rm -rf "$root"

cat > "$OUT/system.json" <<EOF
{"file": "$file", "hash": "$hash", "hash_raw": "$hash_raw", "size": $SIZE}
EOF
echo "system image: $file"
