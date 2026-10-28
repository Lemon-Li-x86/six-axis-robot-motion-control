#!/usr/bin/env bash
set -euo pipefail

elf=${1:?usage: ci_stage2_protocol_qemu.sh <protocol-test.elf> <output-dir>}
out_dir=${2:?usage: ci_stage2_protocol_qemu.sh <protocol-test.elf> <output-dir>}
mkdir -p "$out_dir"
elf=$(realpath "$elf")
gdb_script="$out_dir/protocol.gdb"
gdb_log="$out_dir/protocol-gdb.log"
qemu_log="$out_dir/protocol-qemu.log"

cat > "$gdb_script" <<'GDB'
set pagination off
set confirm off
target remote localhost:1235
break qemu_target_test_done
continue
printf "magic=%u\n", g_qemu_target_test_report.magic
printf "total=%u\n", g_qemu_target_test_report.total
printf "passed=%u\n", g_qemu_target_test_report.passed
printf "failed=%u\n", g_qemu_target_test_report.failed
printf "timeout_recovery=%u\n", g_qemu_target_test_report.results[7]
detach
quit
GDB

qemu-system-arm -M mps2-an386 -kernel "$elf" -S \
  -gdb tcp::1235 -serial null -monitor none -display none \
  >"$qemu_log" 2>&1 &
qemu_pid=$!
cleanup() {
  kill "$qemu_pid" 2>/dev/null || true
  wait "$qemu_pid" 2>/dev/null || true
}
trap cleanup EXIT
sleep 1
kill -0 "$qemu_pid" 2>/dev/null || {
  cat "$qemu_log"
  echo 'QEMU exited before protocol test attachment' >&2
  exit 1
}

timeout 30s gdb-multiarch -nx -batch -x "$gdb_script" "$elf" \
  >"$gdb_log" 2>&1
cat "$gdb_log"
grep -qx 'magic=1363496277' "$gdb_log"
grep -qx 'total=8' "$gdb_log"
grep -qx 'passed=8' "$gdb_log"
grep -qx 'failed=0' "$gdb_log"
grep -qx 'timeout_recovery=1' "$gdb_log"
echo "PASS: Stage2 Cortex-M4/QEMU protocol and ring-buffer tests (8/8)."
