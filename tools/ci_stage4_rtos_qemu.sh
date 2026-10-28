#!/usr/bin/env bash
# Actual ARM/QEMU/GDB execution of the gated, non-actuating Stage4 RTOS
# motion self-test. Unlike host-only CI, this verifies DWARF types AND
# that FreeRTOS trajectory/control/monitor counters changed at runtime.
set -euo pipefail

elf="${1:?usage: ci_stage4_rtos_qemu.sh path/to/freertos_demo.elf}"
out="${2:-build/stage4-rtos-ci-gdb.log}"
mkdir -p "$(dirname "$out")"
command -v qemu-system-arm >/dev/null
command -v gdb-multiarch >/dev/null
test -f "$elf"

gdb_script=$(mktemp)
qemu_pid=""
cleanup() {
    if [[ -n "$qemu_pid" ]]; then
        kill "$qemu_pid" 2>/dev/null || true
        wait "$qemu_pid" 2>/dev/null || true
    fi
    rm -f "$gdb_script"
}
trap cleanup EXIT

cat > "$gdb_script" <<'GDB'
set pagination off
set confirm off
set remotetimeout 20
target remote localhost:1234
break stage4_rtos_selftest_done
continue
printf "finished=%u\n", g_stage4_rtos_test_finished
printf "polynomial_checks=%u\n", g_stage4_polynomial_rtos_report.checks
printf "polynomial_failed=%u\n", g_stage4_polynomial_rtos_report.failed
printf "polynomial_rejected_before_publish=%u\n", g_stage4_polynomial_rtos_report.rejected_before_publish
printf "polynomial_valid_started=%u\n", g_stage4_polynomial_rtos_report.valid_execution_started
printf "polynomial_stack_free_words=%u\n", g_stage4_polynomial_rtos_report.trajectory_stack_free_words
printf "healthy_duration_ms=%u\n", g_stage4_rtos_healthy_duration_ms
printf "checks=%u\n", g_stage4_rtos_test_report.checks
printf "failed=%u\n", g_stage4_rtos_test_report.failed
printf "accepted=%u\n", g_stage4_rtos_test_report.requests_accepted
printf "trajectory_samples=%u\n", g_stage4_rtos_test_report.trajectory_samples
printf "controller_steps=%u\n", g_stage4_rtos_test_report.controller_steps
printf "monitoring_ticks=%u\n", g_stage4_rtos_test_report.monitoring_ticks
printf "controller_faults=%u\n", g_stage4_rtos_test_report.controller_faults
printf "planner_faults=%u\n", g_stage4_rtos_test_report.planner_faults
printf "feedback_stale_count=%u\n", g_stage4_rtos_test_report.feedback_stale_count
printf "stack_words_control=%u\n", g_stage4_rtos_test_report.control_stack_free_words
printf "stack_words_trajectory=%u\n", g_stage4_rtos_test_report.trajectory_stack_free_words
printf "stack_words_monitor=%u\n", g_stage4_rtos_test_report.monitor_stack_free_words
printf "timing_frequency_hz=%u\n", g_stage4_timing_frequency_hz
printf "control_interval_samples=%u\n", g_stage4_timing_summary[0].interval_sample_count
printf "control_processing_samples=%u\n", g_stage4_timing_summary[0].execution_sample_count
printf "control_expected_period_ticks=%u\n", g_stage4_timing_summary[0].expected_period_counter_ticks
printf "control_jitter_p99_ticks=%u\n", g_stage4_timing_summary[0].p99_abs_period_jitter_ticks
printf "control_jitter_max_ticks=%u\n", g_stage4_timing_summary[0].maximum_abs_period_jitter_ticks
printf "control_processing_p99_ticks=%u\n", g_stage4_timing_summary[0].p99_processing_ticks
printf "control_processing_max_ticks=%u\n", g_stage4_timing_summary[0].maximum_processing_ticks
printf "trajectory_interval_samples=%u\n", g_stage4_timing_summary[1].interval_sample_count
printf "trajectory_processing_samples=%u\n", g_stage4_timing_summary[1].execution_sample_count
printf "trajectory_expected_period_ticks=%u\n", g_stage4_timing_summary[1].expected_period_counter_ticks
printf "trajectory_jitter_p99_ticks=%u\n", g_stage4_timing_summary[1].p99_abs_period_jitter_ticks
printf "trajectory_jitter_max_ticks=%u\n", g_stage4_timing_summary[1].maximum_abs_period_jitter_ticks
printf "trajectory_processing_p99_ticks=%u\n", g_stage4_timing_summary[1].p99_processing_ticks
printf "trajectory_processing_max_ticks=%u\n", g_stage4_timing_summary[1].maximum_processing_ticks
printf "monitor_interval_samples=%u\n", g_stage4_timing_summary[2].interval_sample_count
printf "monitor_processing_samples=%u\n", g_stage4_timing_summary[2].execution_sample_count
printf "monitor_expected_period_ticks=%u\n", g_stage4_timing_summary[2].expected_period_counter_ticks
printf "monitor_jitter_p99_ticks=%u\n", g_stage4_timing_summary[2].p99_abs_period_jitter_ticks
printf "monitor_jitter_max_ticks=%u\n", g_stage4_timing_summary[2].maximum_abs_period_jitter_ticks
printf "monitor_processing_p99_ticks=%u\n", g_stage4_timing_summary[2].p99_processing_ticks
printf "monitor_processing_max_ticks=%u\n", g_stage4_timing_summary[2].maximum_processing_ticks
disconnect
quit
GDB

qemu-system-arm -M mps2-an386 -kernel "$elf" -S \
  -gdb tcp::1234 -serial null -monitor none -display none &
qemu_pid=$!
sleep 2
if ! kill -0 "$qemu_pid" 2>/dev/null; then
    echo "QEMU exited before GDB attachment" >&2
    exit 1
fi

set +e
timeout "${STAGE4_QEMU_GDB_TIMEOUT_SEC:-100}s" gdb-multiarch -nx -batch -x "$gdb_script" "$elf" > "$out" 2>&1
result=$?
set -e
cat "$out"
if [[ "$result" -ne 0 ]]; then
    echo "GDB exited with status $result" >&2
    exit "$result"
fi
grep -Eq '^Breakpoint [0-9]+, stage4_rtos_selftest_done' "$out"
grep -qx 'finished=1' "$out"
if [[ -n "${STAGE4_QEMU_EXPECT_HEALTHY_MS:-}" ]]; then
    grep -qx "healthy_duration_ms=${STAGE4_QEMU_EXPECT_HEALTHY_MS}" "$out"
fi
grep -qx 'checks=29' "$out"
grep -qx 'failed=0' "$out"
grep -qx 'polynomial_checks=8' "$out"
grep -qx 'polynomial_failed=0' "$out"
grep -qx 'polynomial_rejected_before_publish=1' "$out"
grep -qx 'polynomial_valid_started=1' "$out"
grep -qx 'accepted=1' "$out"
for label in trajectory_samples controller_steps monitoring_ticks; do
    value=$(sed -n "s/^$label=//p" "$out" | tail -n 1)
    if [[ ! "$value" =~ ^[0-9]+$ ]] || (( value == 0 )); then
        echo "$label is missing or zero" >&2
        exit 1
    fi
done
# The Python analyzer verifies exact field presence, p99<=max and the
# Timer0 frequency, then saves a machine-readable result next to GDB logs.
python3 tools/analyze_stage4_timing.py "$out" "${out%.log}.timing.json"
echo "PASS: ARM/QEMU RTOS motion selftest and 3-task timing report."
