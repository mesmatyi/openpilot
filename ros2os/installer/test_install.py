"""Tests for the ros2os installer and manifest tooling (no device needed).

  python3 -m pytest ros2os/installer/test_install.py
"""
import hashlib
import importlib.util
import json
import lzma
import os
import shutil
import struct
import subprocess
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "builder", "scripts"))
import install
import manifest as manifest_tool


def make_raw(size: int) -> bytes:
  # mix of data, zero runs and repeated words so img2simg emits RAW and FILL chunks
  block = 4096
  out = bytearray(size)
  for i in range(0, size, block):
    kind = (i // block) % 4
    if kind == 0:
      out[i:i + block] = hashlib.sha256(str(i).encode()).digest() * (block // 32)
    elif kind == 2:
      out[i:i + block] = b"\xab\xcd\xef\x01" * (block // 4)
  return bytes(out)


def sparse_by_hand(raw: bytes, blk: int = 4096) -> bytes:
  """Sparse image with RAW, FILL and DONT_CARE chunks (DONT_CARE regions must be zero in `raw`)."""
  chunks = []
  nblk = len(raw) // blk
  for b in range(nblk):
    data = raw[b * blk:(b + 1) * blk]
    if data == bytes(blk):
      chunks.append(struct.pack("<HHII", 0xCAC3, 0, 1, 12))
    elif data == data[:4] * (blk // 4):
      chunks.append(struct.pack("<HHII", 0xCAC2, 0, 1, 16) + data[:4])
    else:
      chunks.append(struct.pack("<HHII", 0xCAC1, 0, 1, 12 + blk) + data)
  header = struct.pack("<IHHHHIIII", 0xED26FF3A, 1, 0, 28, 12, blk, nblk, len(chunks), 0)
  return header + b"".join(chunks)


def entry(name: str, image: bytes, raw: bytes, url: str, sparse: bool, full_check: bool) -> dict:
  return {"name": name, "url": url, "hash": hashlib.sha256(image).hexdigest(), "hash_raw": hashlib.sha256(raw).hexdigest(),
          "size": len(raw), "sparse": sparse, "full_check": full_check, "has_ab": True}


def flash_and_verify(tmp_path, image: bytes, raw: bytes, sparse: bool, full_check: bool, compress: bool = True):
  name = "system.img.xz" if compress else "system.img"
  (tmp_path / name).write_bytes(lzma.compress(image, format=lzma.FORMAT_XZ) if compress else image)
  p = entry("system", image, raw, name, sparse, full_check)
  part = tmp_path / "system_b"
  part.write_bytes(os.urandom(len(raw) + 128))  # partition is larger than the image
  assert not install.verify(str(part), p)
  install.flash(str(part), p, str(tmp_path), lambda *_: None)
  assert install.verify(str(part), p)
  assert part.read_bytes()[:len(raw)] == raw
  return p, part


def test_unsparse_handwritten_chunks(tmp_path):
  raw = make_raw(64 * 4096)
  flash_and_verify(tmp_path, sparse_by_hand(raw), raw, sparse=True, full_check=False)


@pytest.mark.skipif(shutil.which("img2simg") is None, reason="img2simg not installed")
def test_unsparse_img2simg_output(tmp_path):
  raw = make_raw(256 * 4096)
  (tmp_path / "raw.img").write_bytes(raw)
  subprocess.run(["img2simg", str(tmp_path / "raw.img"), str(tmp_path / "sparse.img")], check=True)
  image = (tmp_path / "sparse.img").read_bytes()
  flash_and_verify(tmp_path, image, raw, sparse=True, full_check=False)


@pytest.mark.skipif(shutil.which("img2simg") is None, reason="img2simg not installed")
def test_matches_openpilot_agnos_updater(tmp_path):
  # openpilot's own updater must produce the same partition contents and hashes from our images
  spec = importlib.util.spec_from_file_location("agnos", os.path.join(REPO, "openpilot/common/hardware/comma/agnos.py"))
  agnos = importlib.util.module_from_spec(spec)
  try:
    spec.loader.exec_module(agnos)
  except ImportError as e:
    pytest.skip(f"agnos.py dependencies missing: {e}")
  raw = make_raw(128 * 4096)
  (tmp_path / "raw.img").write_bytes(raw)
  subprocess.run(["img2simg", str(tmp_path / "raw.img"), str(tmp_path / "sparse.img")], check=True)
  image = (tmp_path / "sparse.img").read_bytes()

  class Stream:  # agnos.StreamingDecompressor without the HTTP download
    def __init__(self, data):
      self.data, self.pos, self.sha256 = data, 0, hashlib.sha256()

    def read(self, n):
      out = self.data[self.pos:self.pos + n]
      self.pos += n
      self.sha256.update(out)
      return out

  s = Stream(image)
  out = b"".join(agnos.unsparsify(s))
  assert out == raw
  assert s.sha256.hexdigest() == hashlib.sha256(image).hexdigest()


def test_raw_full_check_partition(tmp_path):
  raw = os.urandom(3 * 4096 + 17)
  flash_and_verify(tmp_path, raw, raw, sparse=False, full_check=True)


def test_uncompressed_source(tmp_path):
  raw = make_raw(16 * 4096)
  flash_and_verify(tmp_path, sparse_by_hand(raw), raw, sparse=True, full_check=False, compress=False)


def test_hash_mismatch_is_rejected_and_marker_cleared(tmp_path):
  raw = make_raw(16 * 4096)
  p, part = flash_and_verify(tmp_path, sparse_by_hand(raw), raw, sparse=True, full_check=False)
  bad = dict(p, hash_raw="00" * 32)
  with pytest.raises(ValueError):
    install.flash(str(part), bad, str(tmp_path), lambda *_: None)
  assert not install.verify(str(part), p)  # interrupted/failed writes never verify


def test_resolve():
  assert install.resolve("system.img.xz", "/data/tmp") == "/data/tmp/system.img.xz"
  assert install.resolve("/abs/x", "/data/tmp") == "/abs/x"
  assert install.resolve("system.img.xz", "https://example.com/ros2os") == "https://example.com/ros2os/system.img.xz"
  assert install.resolve("https://a/b", "/data") == "https://a/b"


def test_partition_path():
  assert install.partition_path({"name": "boot"}, "_b") == "/dev/disk/by-partlabel/boot_b"
  assert install.partition_path({"name": "persist", "has_ab": False}, "_b") == "/dev/disk/by-partlabel/persist"


def test_build_manifest_keeps_agnos_partitions():
  agnos = json.load(open(os.path.join(REPO, "openpilot/system/hardware/comma/agnos.json")))
  system = {"file": "system-abc.img.xz", "hash": "11" * 32, "hash_raw": "22" * 32,
            "size": next(p for p in agnos if p["name"] == "system")["size"]}
  out = manifest_tool.build_manifest(agnos, system, "https://example.com/r", "0.1.0")
  assert [p["name"] for p in out] == [p["name"] for p in agnos]
  for a, b in zip(agnos, out, strict=True):
    if a["name"] != "system":
      assert a == b
  sys_entry = next(p for p in out if p["name"] == "system")
  assert sys_entry["url"] == "https://example.com/r/system-abc.img.xz"
  assert sys_entry["hash_raw"] == "22" * 32 and sys_entry["sparse"] and not sys_entry["full_check"]
  local = manifest_tool.build_manifest(agnos, system, "", "0.1.0")
  assert next(p for p in local if p["name"] == "system")["url"] == "system-abc.img.xz"


def test_system_env_quotes_values(capsys):
  manifest_tool.main(["system-env", os.path.join(REPO, "openpilot/system/hardware/comma/agnos.json")])
  out = capsys.readouterr().out
  env = dict(line.split("=", 1) for line in out.strip().splitlines())
  assert env["AGNOS_SPARSE"] == "true"
  assert int(env["AGNOS_SIZE"]) % 4096 == 0
