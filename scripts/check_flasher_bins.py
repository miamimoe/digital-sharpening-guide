#!/usr/bin/env python3
"""Validate every firmware binary the browser flasher serves.

Run from the repo root:  python3 scripts/check_flasher_bins.py

Every manifest in docs/ flashes a single image at offset 0, which means each
.bin MUST be a MERGED image (bootloader + partition table + boot_app0 + app),
not a bare application image. A bare app flashed at offset 0 leaves the device
with no second-stage bootloader: black screen, never boots, USB still
enumerates because the USB bridge is independent of the ESP32 running.

That exact failure shipped once (v0.2.1 / v0.3.0-beta.1, 2026-08-22): the
published bins were `.pio/build/<env>/firmware.bin` copied verbatim — app
images — and every device flashed from the page was left unbootable until
reflashed. The ~55 KB size difference against v0.2.0 was the missing
bootloader region, misread at the time as dependency drift. This script exists
so that mistake is structurally impossible to repeat: it fails loudly on any
manifest that points at a bare app image (magic 0xE9 at offset 0x0 with an app
image at 0x10000 absent, etc.).

Expected merged layout (Arduino-ESP32 defaults, both chip families):
  ESP32    : 0x0000 0xFF padding | 0x1000 bootloader (0xE9) | 0x8000 partition
             table (0xAA 0x50) | 0xE000 boot_app0 | 0x10000 app (0xE9)
  ESP32-S3 : 0x0000 bootloader (0xE9) | same from 0x8000 onward
"""
import json
import sys
from pathlib import Path

DOCS = Path(__file__).resolve().parent.parent / "docs"

APP_OFF, PART_OFF, BOOT_APP0_OFF, ESP32_BL_OFF = 0x10000, 0x8000, 0xE000, 0x1000


def peek(data: bytes, off: int, n: int) -> bytes:
    return data[off:off + n]


def check_bin(path: Path, chip_family: str) -> list[str]:
    errs = []
    if not path.is_file():
        return [f"missing file: {path}"]
    data = path.read_bytes()
    if len(data) <= APP_OFF:
        return [f"{path.name}: only {len(data)} bytes — no room for an app at 0x10000; "
                f"this is not a merged image"]

    if chip_family == "ESP32":
        if peek(data, 0, 4) != b"\xff\xff\xff\xff":
            errs.append(f"{path.name}: offset 0x0 is not 0xFF padding — on ESP32 the "
                        f"bootloader belongs at 0x1000; a 0xE9 here means this is a BARE "
                        f"APP image and will not boot when flashed at offset 0")
        if peek(data, ESP32_BL_OFF, 1) != b"\xe9":
            errs.append(f"{path.name}: no image magic at 0x1000 — bootloader missing")
    elif chip_family == "ESP32-S3":
        if peek(data, 0, 1) != b"\xe9":
            errs.append(f"{path.name}: no image magic at 0x0 — S3 bootloader missing")
    else:
        errs.append(f"{path.name}: unknown chipFamily {chip_family!r}")

    if peek(data, PART_OFF, 2) != b"\xaa\x50":
        errs.append(f"{path.name}: no partition-table magic (AA 50) at 0x8000")
    if peek(data, BOOT_APP0_OFF, 4) not in (b"\x01\x00\x00\x00", b"\x00\x00\x00\x00"):
        errs.append(f"{path.name}: 0xE000 does not look like boot_app0")
    if peek(data, APP_OFF, 1) != b"\xe9":
        errs.append(f"{path.name}: no app image magic at 0x10000")
    return errs


def main() -> int:
    manifests = sorted(DOCS.glob("manifest*.json"))
    if not manifests:
        print("ERROR: no manifests found under docs/")
        return 1
    failures, checked = [], 0
    for m in manifests:
        doc = json.loads(m.read_text())
        for build in doc.get("builds", []):
            fam = build.get("chipFamily", "?")
            parts = build.get("parts", [])
            for part in parts:
                if part.get("offset", None) != 0:
                    failures.append(f"{m.name}: part offset {part.get('offset')!r} != 0 — "
                                    f"this repo's convention is merged images at offset 0")
                    continue
                checked += 1
                errs = check_bin(DOCS / part["path"], fam)
                for e in errs:
                    failures.append(f"{m.name}: {e}")
    for f in failures:
        print(f"FAIL  {f}")
    print(f"\n{checked} manifest part(s) checked across {len(manifests)} manifest(s): "
          f"{'ALL OK' if not failures else f'{len(failures)} problem(s)'}")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
