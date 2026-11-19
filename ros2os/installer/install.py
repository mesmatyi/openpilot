#!/usr/bin/env python3
"""Install ros2os on a comma device running AGNOS, without EDL.

  python3 install.py ros2os-manifest.json [--reboot] [--yes]

Writes every partition of the manifest (AGNOS firmware + boot, ros2os system) to the
*inactive* A/B slot, verifies the hashes, and marks that slot active. The slot you are
running now (AGNOS + openpilot) is not touched, and /data (openpilot, Wi-Fi, params) is
kept, so `ros2os-rollback` (or `abctl --set_active <old slot>`) returns to it.

Partition URLs may be http(s)://, file:// or a path relative to the manifest.
The flashing and verification logic mirrors openpilot/common/hardware/comma/agnos.py
(same manifest format, same raw-hash marker after non-full-check images). Stdlib only.
"""
import argparse
import datetime
import hashlib
import json
import lzma
import os
import shutil
import struct
import subprocess
import sys
import time
import urllib.parse
import urllib.request
from collections.abc import Callable, Iterator

SPARSE_MAGIC = 0xED26FF3A
CHUNK_RAW, CHUNK_FILL, CHUNK_DONT_CARE, CHUNK_CRC32 = 0xCAC1, 0xCAC2, 0xCAC3, 0xCAC4
READ_SIZE = 1024 * 1024
STATE_DIR = "/data/ros2os"


# ---------- reading ----------

def resolve(url: str, base: str) -> str:
  """Resolve a manifest URL against the manifest's location (a directory or a URL)."""
  if urllib.parse.urlparse(url).scheme:
    return url
  if urllib.parse.urlparse(base).scheme:
    return urllib.parse.urljoin(base.rstrip("/") + "/", url)
  return url if os.path.isabs(url) else os.path.join(base, url)


def open_source(url: str, base: str = ""):
  """Binary file-like object for an http(s) URL, file:// URL or (relative) path."""
  url = resolve(url, base)
  parsed = urllib.parse.urlparse(url)
  if parsed.scheme in ("http", "https"):
    req = urllib.request.Request(url, headers={"Accept-Encoding": "identity", "User-Agent": "ros2os-installer"})
    return urllib.request.urlopen(req, timeout=60)
  if parsed.scheme not in ("", "file"):
    raise ValueError(f"unsupported URL scheme in {url}")
  return open(parsed.path if parsed.scheme == "file" else url, "rb")


class Decompressed:
  """Streams an .xz (or uncompressed) source and hashes every decompressed byte that is read."""

  def __init__(self, f):
    self.f = f
    self.buf = bytearray()
    self.sha256 = hashlib.sha256()
    self.eof = False
    head = f.read(6)
    self.lzma = lzma.LZMADecompressor() if head == b"\xfd7zXZ\x00" else None
    self._push(head)

  def _push(self, data: bytes) -> None:
    if self.lzma is not None:
      data = self.lzma.decompress(data)
    self.buf += data

  def read(self, n: int) -> bytes:
    while len(self.buf) < n and not self.eof:
      chunk = self.f.read(READ_SIZE)
      if not chunk:
        self.eof = True
        break
      self._push(chunk)
    out = bytes(self.buf[:n])
    del self.buf[:n]
    self.sha256.update(out)
    return out

  def read_exact(self, n: int) -> bytes:
    out = self.read(n)
    if len(out) != n:
      raise ValueError("unexpected end of image")
    return out


def unsparse(src: Decompressed) -> Iterator[bytes]:
  """Android sparse image -> raw bytes (https://source.android.com/devices/bootloader/images#sparse-format)."""
  magic, major, minor, file_hdr_sz, chunk_hdr_sz, blk_sz, _total_blks, total_chunks, _crc = \
    struct.unpack("<IHHHHIIII", src.read_exact(28))
  if magic != SPARSE_MAGIC or major != 1:
    raise ValueError("not an Android sparse image")
  src.read_exact(file_hdr_sz - 28)
  for _ in range(total_chunks):
    chunk_type, _reserved, chunk_blks, total_sz = struct.unpack("<HHII", src.read_exact(12))
    src.read_exact(chunk_hdr_sz - 12)
    size = chunk_blks * blk_sz
    if chunk_type == CHUNK_RAW:
      while size > 0:
        n = min(size, READ_SIZE)
        yield src.read_exact(n)
        size -= n
    elif chunk_type in (CHUNK_FILL, CHUNK_DONT_CARE):
      # don't-care blocks are written as zeros so the partition matches the raw image hash
      pattern = src.read_exact(4) if chunk_type == CHUNK_FILL else b"\x00\x00\x00\x00"
      block = pattern * (READ_SIZE // 4)
      while size > 0:
        n = min(size, READ_SIZE)
        yield block[:n]
        size -= n
    elif chunk_type == CHUNK_CRC32:
      src.read_exact(total_sz - chunk_hdr_sz)
    else:
      raise ValueError(f"unknown sparse chunk type 0x{chunk_type:04X}")


def passthrough(src: Decompressed) -> Iterator[bytes]:
  while chunk := src.read(READ_SIZE):
    yield chunk


# ---------- partitions ----------

def partition_path(p: dict, slot: str, root: str = "/dev/disk/by-partlabel") -> str:
  return os.path.join(root, p["name"] + (slot if p.get("has_ab", True) else ""))


def verify(path: str, p: dict) -> bool:
  try:
    with open(path, "rb") as f:
      if p.get("full_check", False):
        h = hashlib.sha256()
        remaining = p["size"]
        while remaining > 0:
          data = f.read(min(READ_SIZE, remaining))
          if not data:
            return False
          h.update(data)
          remaining -= len(data)
        return h.hexdigest() == p["hash_raw"].lower()
      f.seek(p["size"])
      return f.read(64) == p["hash_raw"].lower().encode()
  except OSError:
    return False


def flash(path: str, p: dict, base_dir: str, progress: Callable[[str, int], None]) -> None:
  full_check = p.get("full_check", False)
  if not full_check:  # invalidate the marker first so an interrupted write never verifies
    with open(path, "r+b") as out:
      out.seek(p["size"])
      out.write(b"\x00" * 64)
      os.fsync(out.fileno())

  with open_source(p["url"], base_dir) as f:
    src = Decompressed(f)
    raw = hashlib.sha256()
    written, last = 0, -1
    with open(path, "r+b") as out:
      for chunk in (unsparse if p.get("sparse") else passthrough)(src):
        if written + len(chunk) > p["size"]:
          raise ValueError(f"{p['name']}: image larger than {p['size']} bytes")
        raw.update(chunk)
        out.write(chunk)
        written += len(chunk)
        pct = written * 100 // p["size"]
        if pct != last:
          progress(p["name"], pct)
          last = pct
      out.flush()
      os.fsync(out.fileno())
      if written != p["size"]:
        raise ValueError(f"{p['name']}: wrote {written} bytes, expected {p['size']}")
      if raw.hexdigest() != p["hash_raw"].lower():
        raise ValueError(f"{p['name']}: raw hash mismatch")
      if src.sha256.hexdigest() != p["hash"].lower():
        raise ValueError(f"{p['name']}: image hash mismatch")
      if not full_check:
        out.seek(p["size"])
        out.write(p["hash_raw"].lower().encode())
        out.flush()
        os.fsync(out.fileno())


# ---------- device ----------

def run(cmd: list[str]) -> str:
  return subprocess.run(cmd, check=True, capture_output=True, text=True).stdout.strip()


def current_slot() -> str:
  return run(["abctl", "--boot_slot"])


def device_type() -> str:
  try:
    with open("/sys/firmware/devicetree/base/model") as f:
      return f.read().strip("\x00\n ").split("comma ")[-1]
  except OSError:
    return "unknown"


def save_ssh_keys() -> int:
  """Carry the SSH keys you use today over to ros2os (sshd also reads /data/ros2os/authorized_keys)."""
  keys: list[str] = []
  for src in ("/data/params/d/GithubSshKeys", os.path.expanduser("~comma/.ssh/authorized_keys"),
              os.path.join(STATE_DIR, "authorized_keys")):
    try:
      with open(src) as f:
        keys += [k.strip() for k in f if k.strip() and not k.startswith("#")]
    except OSError:
      pass
  keys = list(dict.fromkeys(keys))
  if keys:
    dst = os.path.join(STATE_DIR, "authorized_keys")
    with open(dst, "w") as f:
      f.write("\n".join(keys) + "\n")
    os.chmod(dst, 0o644)
    try:
      shutil.chown(dst, "comma")
    except (LookupError, OSError):
      pass
  return len(keys)


def main(argv=None) -> int:
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("manifest", help="ros2os-manifest.json (path or URL)")
  ap.add_argument("--reboot", action="store_true", help="reboot into ros2os when done")
  ap.add_argument("--yes", "-y", action="store_true", help="do not ask for confirmation")
  ap.add_argument("--allow-non-ab", action="store_true", help="also write partitions without A/B slots (dangerous)")
  args = ap.parse_args(argv)

  if os.geteuid() != 0:
    os.execvp("sudo", ["sudo", sys.executable, os.path.abspath(__file__)] + (argv if argv is not None else sys.argv[1:]))

  if urllib.parse.urlparse(args.manifest).scheme:
    base_dir = args.manifest.rsplit("/", 1)[0]
  else:
    base_dir = os.path.dirname(os.path.abspath(args.manifest))
  with open_source(args.manifest, os.getcwd()) as f:
    manifest = json.load(f)

  dev = device_type()
  if dev not in ("mici", "tizi"):
    print(f"warning: device type '{dev}'; ros2os targets the comma four (mici) and comma 3X (tizi)")
  slot = current_slot()
  target_slot, target_num = ("_b", 1) if slot == "_a" else ("_a", 0)
  version = next((p.get("ros2os_version") for p in manifest if p["name"] == "system"), None)

  partitions = []
  for p in manifest:
    if not p.get("has_ab", True) and not args.allow_non_ab:
      print(f"skipping {p['name']}: no A/B slot (writing it would affect the running system)")
      continue
    partitions.append(p)

  print(f"device {dev}, running slot {slot}; installing ros2os {version or '?'} into slot {target_slot}:")
  for p in partitions:
    print(f"  {p['name']:<12} {p['size'] / 1e6:9.1f} MB  -> {partition_path(p, target_slot)}")
  if not args.yes and input("continue? [y/N] ").strip().lower() not in ("y", "yes"):
    print("aborted")
    return 1

  subprocess.run(["abctl", "--set_unbootable", str(target_num)], check=False)

  def progress(name: str, pct: int) -> None:
    print(f"\r  writing {name}: {pct:3d}%", end="", flush=True)

  for p in partitions:
    path = partition_path(p, target_slot)
    if verify(path, p):
      print(f"  {p['name']}: already up to date")
      continue
    for attempt in range(5):
      try:
        flash(path, p, base_dir, progress)
        print()
        break
      except (OSError, ValueError) as e:
        print(f"\n  {p['name']}: {e} (attempt {attempt + 1}/5)")
        time.sleep(5)
    else:
      print(f"failed to write {p['name']}; slot {target_slot} stays unbootable, slot {slot} is unchanged")
      return 1
    if not verify(path, p):
      print(f"verification of {p['name']} failed")
      return 1

  os.makedirs(STATE_DIR, exist_ok=True)
  n_keys = save_ssh_keys()
  with open(os.path.join(STATE_DIR, "install.json"), "w") as f:
    json.dump({"version": version, "from_slot": slot, "to_slot": target_slot,
               "installed": datetime.datetime.now(datetime.UTC).isoformat()}, f)
  print(f"saved {n_keys} SSH key(s) to {STATE_DIR}/authorized_keys")

  for _ in range(10):
    out = subprocess.run(["abctl", "--set_active", str(target_num)], capture_output=True, text=True)
    text = out.stdout + out.stderr
    # agnos.py treats "lun as boot lun" in the output as success
    if "No such file or directory" not in text and ("lun as boot lun" in text or out.returncode == 0):
      break
    time.sleep(1)
  else:
    print(f"abctl --set_active {target_num} failed:\n{text}")
    return 1

  print(f"ros2os is installed in slot {target_slot}. To go back later: ros2os-rollback")
  if args.reboot:
    subprocess.run(["reboot"], check=False)
  else:
    print("reboot to start it: sudo reboot")
  return 0


if __name__ == "__main__":
  sys.exit(main())
