#!/bin/bash
set -euo pipefail

SOURCE="${1:-main.c}"

if [ ! -f "$SOURCE" ]; then
    echo "Error: source file not found: $SOURCE"
    exit 1
fi

echo "Compiling: $SOURCE"

# Compile and link for the RV32I processor.
riscv64-unknown-elf-gcc \
    -O0 -march=rv32i -mabi=ilp32 -mno-relax \
    -T link.ld -nostartfiles -nostdlib \
    -Wl,-e,_start,-Map=main.map \
    -o main.elf start.s "$SOURCE"

# Export information for verification and reproducibility.
riscv64-unknown-elf-objdump -d main.elf > main.dis
riscv64-unknown-elf-nm -n main.elf > main.symbols
riscv64-unknown-elf-size main.elf

# Export the loadable image.
riscv64-unknown-elf-objcopy -O binary main.elf firmware.bin

# Convert little-endian bytes into 4096 hexadecimal 32-bit words.
python3 - <<'PY'
from pathlib import Path

capacity = 16 * 1024
image = Path("firmware.bin").read_bytes()

if len(image) > capacity:
    raise SystemExit("Error: firmware exceeds 16 KiB")

image = image.ljust(capacity, b"\x00")

with Path("firmware.mem").open("w") as output:
    for offset in range(0, capacity, 4):
        word = int.from_bytes(image[offset:offset + 4], "little")
        output.write(f"{word:08x}\n")

print("Generated firmware.mem: 4096 words")
PY

echo "Build complete."