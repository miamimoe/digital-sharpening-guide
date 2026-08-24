#!/usr/bin/env bash
# Build the three release firmwares and package each as a MERGED flash image
# (bootloader + partition table + boot_app0 + app) for the browser flasher,
# which flashes a single file at offset 0. See scripts/check_flasher_bins.py
# for why a bare app image here bricks the device until reflashed.
#
# Usage: scripts/build_flasher_bins.sh <version>      e.g. 0.3.0
# Output: docs/firmware/sharpening-guide-{plus,plus2,s3}-v<version>.bin
set -euo pipefail
VER="${1:?usage: $0 <version>}"
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
PIO="${PIO:-$HOME/.local/bin/pio}"
ESPTOOL="${ESPTOOL:-$HOME/.platformio/packages/tool-esptoolpy/esptool.py}"
PY="${PY:-$HOME/.local/pipx/venvs/platformio/bin/python}"
BOOT_APP0="$HOME/.platformio/packages/framework-arduinoespressif32/tools/partitions/boot_app0.bin"
OUT="$ROOT/docs/firmware"

# flash-size must match the board's REAL flash and its partition table: merge_bin
# rewrites the bootloader header with this value, and the second-stage bootloader
# validates every partition entry against it. Declaring 4MB on the S3 (whose
# default_8MB.csv table spans the full 8MB) makes the bootloader reject the
# table and software-reset in a silent loop (issue #1, v0.3.0..v1.0.0 S3 bins).
merge() {  # env name chip bootloader-offset flash-size
    local env="$1" name="$2" chip="$3" bl_off="$4" fsize="$5"
    local b="$ROOT/.pio/build/$env"
    "$PIO" run -d "$ROOT" -e "$env"
    "$PY" "$ESPTOOL" --chip "$chip" merge_bin -o "$OUT/sharpening-guide-$name-v$VER.bin" \
        --flash_mode dio --flash_freq 40m --flash_size "$fsize" \
        "$bl_off" "$b/bootloader.bin" 0x8000 "$b/partitions.bin" \
        0xe000 "$BOOT_APP0" 0x10000 "$b/firmware.bin"
}

merge m5stick-c-plus  plus  esp32   0x1000 4MB
merge m5stick-c-plus2 plus2 esp32   0x1000 4MB
merge m5stick-s3      s3    esp32s3 0x0    8MB
ls -l "$OUT"/sharpening-guide-*-v"$VER".bin
