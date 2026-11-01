#!/usr/bin/env bash
# Stage5 QEMU/GDB repeatability: alternating AB/BA order, same ELF per arm.
# This is a synthetic, immobile-feedback RTOS task benchmark, NOT full PyBullet load.
set -euo pipefail
if [[ $# -lt 3 || $# -gt 4 ]]; then
  echo "usage: run_stage5_perf_repeats.sh BASE_ELF OPT_ELF OUT_DIR [PAIRS=6]" >&2
  exit 2
fi
baseline="$1"
optimized="$2"
outdir="$3"
pairs="${4:-6}"
[[ -f "$baseline" && -f "$optimized" ]] || { echo "ELF missing" >&2; exit 2; }
[[ "$pairs" =~ ^[1-9][0-9]*$ ]] && (( pairs <= 20 )) || {
  echo "PAIRS must be 1..20" >&2; exit 2;
}
mkdir -p "$outdir"
for ((i=1;i<=pairs;i++)); do
  n=$(printf '%02d' "$i")
  # Keep all trials sequential; invert order in every second pair to expose
  # machine warmup/load drift instead of always favoring the second ELF.
  if (( i % 2 == 1 )); then
    order=(baseline optimized)
  else
    order=(optimized baseline)
  fi
  for mode in "${order[@]}"; do
    if [[ "$mode" == baseline ]]; then elf="$baseline"; else elf="$optimized"; fi
    prefix="$outdir/pair-${n}-${mode}"
    echo "[Stage5 PERF] pair=$i/$pairs mode=$mode ELF=$elf"
    bash tools/ci_stage5_cpu_trace_qemu.sh "$elf" "$prefix"
  done
done
python3 tools/aggregate_stage5_perf_repeats.py "$outdir" --pairs "$pairs" --output "$outdir/repeated-ab.json"
