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
printf "checks=%u\n", g_stage4_rtos_test_report.checks
printf "failed=%u\n", g_stage4_rtos_test_report.failed
printf "accepted=%u\n", g_stage4_rtos_test_report.requests_accepted
printf "trajectory_samples=%u\n", g_stage4_rtos_test_report.trajectory_samples
printf "controller_steps=%u\n", g_stage4_rtos_test_report.controller_steps
printf "monitoring_ticks=%u\n", g_stage4_rtos_test_report.monitoring_ticks
printf "controller_faults=%u\n", g_stage4_rtos_test_report.controller_faults
printf "planner_faults=%u\n", g_stage4_rtos_test_report.planner_faults
printf "feedback_stale_count=%u\n", g_stage4_rtos_test_report.feedback_stale_count
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
timeout 100s gdb-multiarch -nx -batch -x "$gdb_script" "$elf" > "$out" 2>&1
result=$?
set -e
cat "$out"
if [[ "$result" -ne 0 ]]; then
    echo "GDB exited with status $result" >&2
    exit "$result"
fi
grep -Eq '^Breakpoint [0-9]+, stage4_rtos_selftest_done' "$out"
grep -qx 'finished=1' "$out"
grep -qx 'checks=13' "$out"
grep -qx 'failed=0' "$out"
grep -qx 'accepted=1' "$out"
for label in trajectory_samples controller_steps monitoring_ticks; do
    value=$(sed -n "s/^$label=//p" "$out" | tail -n 1)
    if [[ ! "$value" =~ ^[0-9]+$ ]] || (( value == 0 )); then
        echo "$label is missing or zero" >&2
        exit 1
    fi
done
echo "PASS: ARM/QEMU RTOS motion selftest reached all task counters."
