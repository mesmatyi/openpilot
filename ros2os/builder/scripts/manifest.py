#!/usr/bin/env python3
"""AGNOS manifest helpers for the ros2os build.

  manifest.py system-env AGNOS_JSON
      print the pinned AGNOS system partition as shell variables
  manifest.py write AGNOS_JSON SYSTEM_JSON [--base-url URL] [--version V] --output OUT
      write the ros2os install manifest: every AGNOS partition (firmware, boot) unchanged,
      the system partition replaced by the ros2os image
"""
import argparse
import json
import shlex
import sys


def load(path: str) -> list[dict]:
  with open(path) as f:
    return json.load(f)


def system_entry(manifest: list[dict]) -> dict:
  entries = [p for p in manifest if p["name"] == "system"]
  if len(entries) != 1:
    raise SystemExit("AGNOS manifest must contain exactly one 'system' partition")
  return entries[0]


def system_env(args) -> None:
  s = system_entry(load(args.agnos))
  values = {
    "AGNOS_URL": s["url"],
    "AGNOS_HASH": s["hash"],
    "AGNOS_HASH_RAW": s["hash_raw"],
    "AGNOS_SIZE": str(s["size"]),
    "AGNOS_SPARSE": "true" if s.get("sparse") else "false",
  }
  for k, v in values.items():
    print(f"{k}={shlex.quote(v)}")


def build_manifest(agnos: list[dict], system: dict, base_url: str, version: str) -> list[dict]:
  url = f"{base_url.rstrip('/')}/{system['file']}" if base_url else system["file"]
  out = []
  for p in agnos:
    if p["name"] != "system":
      out.append(dict(p))
      continue
    out.append({
      "name": "system",
      "url": url,
      "hash": system["hash"],
      "hash_raw": system["hash_raw"],
      "size": system["size"],
      "sparse": True,
      "full_check": False,
      "has_ab": True,
      "ondevice_hash": system["hash_raw"],
      "ros2os_version": version,
    })
  return out


def write(args) -> None:
  agnos = load(args.agnos)
  with open(args.system) as f:
    system = json.load(f)
  agnos_system = system_entry(agnos)
  if system["size"] != agnos_system["size"]:
    raise SystemExit(f"system image size {system['size']} differs from the AGNOS partition image size {agnos_system['size']}")
  manifest = build_manifest(agnos, system, args.base_url, args.version)
  with open(args.output, "w") as f:
    json.dump(manifest, f, indent=2)
    f.write("\n")
  print(f"wrote {args.output} ({len(manifest)} partitions)")


def main(argv=None) -> None:
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  sub = ap.add_subparsers(dest="cmd", required=True)
  p = sub.add_parser("system-env")
  p.add_argument("agnos")
  p.set_defaults(func=system_env)
  p = sub.add_parser("write")
  p.add_argument("agnos")
  p.add_argument("system")
  p.add_argument("--base-url", default="")
  p.add_argument("--version", default="dev")
  p.add_argument("--output", required=True)
  p.set_defaults(func=write)
  args = ap.parse_args(argv)
  args.func(args)


if __name__ == "__main__":
  main(sys.argv[1:])
