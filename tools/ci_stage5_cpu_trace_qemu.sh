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
printf "timing_frequency_hz=%u\\n", g_stage4_timing_frequency_hz
set $j=0
while $j<3
    printf "timing_%u_interval_sample_count=%u\\n", $j, g_stage4_timing_summary[$j].interval_sample_count
    printf "timing_%u_execution_sample_count=%u\\n", $j, g_stage4_timing_summary[$j].execution_sample_count
    printf "timing_%u_expected_period_counter_ticks=%u\\n", $j, g_stage4_timing_summary[$j].expected_period_counter_ticks
    printf "timing_%u_maximum_abs_period_jitter_ticks=%u\\n", $j, g_stage4_timing_summary[$j].maximum_abs_period_jitter_ticks
    printf "timing_%u_p99_abs_period_jitter_ticks=%u\\n", $j, g_stage4_timing_summary[$j].p99_abs_period_jitter_ticks
    printf "timing_%u_maximum_processing_ticks=%u\\n", $j, g_stage4_timing_summary[$j].maximum_processing_ticks
    printf "timing_%u_p99_processing_ticks=%u\\n", $j, g_stage4_timing_summary[$j].p99_processing_ticks
    set $j=$j+1
end
printf "stack_control_free_words=%u\\n", g_motion_runtime.diagnostics.control_stack_free_words
printf "stack_trajectory_free_words=%u\\n", g_motion_runtime.diagnostics.trajectory_stack_free_words
printf "stack_monitor_free_words=%u\\n", g_motion_runtime.diagnostics.monitor_stack_free_words
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

python3 tools/analyze_stage5_task_timing.py "${prefix}.log" "${prefix}-timing.json"
