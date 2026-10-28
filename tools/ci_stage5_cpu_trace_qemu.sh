#!/usr/bin/env bash
# Inspect FreeRTOS switch/tick CPU accounting on real QEMU Cortex-M4.
set -euo pipefail
elf="${1:?usage: ci_stage5_cpu_trace_qemu.sh path/to/elf [output-prefix]}"
prefix="${2:-build/stage5-cpu-trace}"
mkdir -p "$(dirname "$prefix")"
gdb_file=$(mktemp)
qemu_pid=""
cleanup() {
  if [[ -n "$qemu_pid" ]]; then kill "$qemu_pid" 2>/dev/null || true; wait "$qemu_pid" 2>/dev/null || true; fi
  rm -f "$gdb_file"
}
trap cleanup EXIT
cat > "$gdb_file" <<'GDB'
set pagination off
set confirm off
set remotetimeout 20
target remote localhost:1235
break stage4_rtos_selftest_done
continue
printf "elapsed_ticks=%llu\n", g_stage5_cpu_trace_report.elapsed_ticks
printf "timer_frequency_hz=%u\n", g_stage5_cpu_trace_report.timer_frequency_hz
printf "observed_tasks=%u\n", g_stage5_cpu_trace_report.observed_tasks
printf "overflow_count=%u\n", g_stage5_cpu_trace_report.overflow_count
set $i=0
while $i<g_stage5_cpu_trace_report.observed_tasks
    printf "task_%u_name=%s\n", $i, g_stage5_cpu_trace_report.tasks[$i].name
    printf "task_%u_active_ticks=%llu\n", $i, g_stage5_cpu_trace_report.tasks[$i].active_ticks
    printf "task_%u_cpu_bp=%u\n", $i, g_stage5_cpu_trace_report.tasks[$i].cpu_share_basis_points
    set $i=$i+1
end
disconnect
quit
GDB
qemu-system-arm -M mps2-an386 -kernel "$elf" -S \
  -gdb tcp::1235 -serial null -monitor none -display none &
qemu_pid=$!
sleep 2
test -f "$elf"
timeout 120s gdb-multiarch -nx -batch -x "$gdb_file" "$elf" > "${prefix}.log" 2>&1
cat "${prefix}.log"
grep -Eq '^Breakpoint [0-9]+, stage4_rtos_selftest_done' "${prefix}.log"
python3 tools/analyze_stage5_cpu_trace.py "${prefix}.log" "${prefix}.json"
