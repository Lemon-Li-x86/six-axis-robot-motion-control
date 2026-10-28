#!/usr/bin/env bash
# Runs production UART TX, motor and GPIO abstractions on Cortex-M4 QEMU.
# GPIO uses its documented shadow backend; no physical pins or motors actuate.
set -euo pipefail

elf_dir=${1:?usage: ci_stage2_driver_qemu.sh <arm-elf-dir> <output-dir>}
out_dir=${2:?usage: ci_stage2_driver_qemu.sh <arm-elf-dir> <output-dir>}
mkdir -p "$out_dir"

run_test() (
  set -euo pipefail
  local target="$1" breakpoint="$2" report="$3" expected_total="$4"
  local elf="$elf_dir/$target.elf" gdb_script="$out_dir/$target.gdb"
  local gdb_log="$out_dir/$target-gdb.log" qemu_log="$out_dir/$target-qemu.log"
  test -f "$elf"
  cat > "$gdb_script" <<GDB
set pagination off
set confirm off
target remote localhost:1236
break $breakpoint
continue
printf "total=%u\\n", $report.total
printf "passed=%u\\n", $report.passed
printf "failed=%u\\n", $report.failed
detach
quit
GDB

  qemu-system-arm -M mps2-an386 -kernel "$elf" -S \
    -gdb tcp::1236 -serial null -monitor none -display none \
    >"$qemu_log" 2>&1 &
  local qemu_pid=$!
  cleanup() {
    kill "$qemu_pid" 2>/dev/null || true
    wait "$qemu_pid" 2>/dev/null || true
  }
  trap cleanup EXIT
  sleep 1
  kill -0 "$qemu_pid" 2>/dev/null || {
    cat "$qemu_log"
    echo "QEMU exited before driver test attachment: $target" >&2
    exit 1
  }

  timeout 30s gdb-multiarch -nx -batch -x "$gdb_script" "$elf" \
    >"$gdb_log" 2>&1
  cat "$gdb_log"
  grep -qx "total=$expected_total" "$gdb_log"
  grep -qx "passed=$expected_total" "$gdb_log"
  grep -qx 'failed=0' "$gdb_log"
  echo "PASS: $target ($expected_total/$expected_total)."
)

run_test qemu_uart_tx_tests qemu_uart_tx_test_done g_uart_tx_test_report 7
run_test qemu_motor_tests qemu_motor_test_done g_motor_test_report 11
run_test qemu_gpio_tests qemu_gpio_test_done g_gpio_test_report 13
