#!/usr/bin/env bash
# Executes the three existing Stage3/4 standalone ARM/QEMU algorithm tests
# and checks actual GDB diagnostics. Software-only, no motor actuation.
set -euo pipefail
if (( $# < 1 )); then echo "Usage: $0 ELF_DIR [OUT_DIR]" >&2; exit 2; fi
elf_dir="$1"
out_dir="build/stage4-spec-qemu"
if (( $# >= 2 )); then out_dir="$2"; fi
mkdir -p "$out_dir"
command -v qemu-system-arm >/dev/null
command -v gdb-multiarch >/dev/null

run_qemu_test() (
    set -euo pipefail
    local target="$1" breakpoint="$2"
    shift 2
    local elf="$elf_dir/$target.elf"
    local transcript="$out_dir/$target.log"
    local instructions pid exitcode
    test -f "$elf"
    instructions=$(mktemp)
    pid=""
    cleanup() {
        if [[ -n "$pid" ]]; then
            kill "$pid" 2>/dev/null || true
            wait "$pid" 2>/dev/null || true
        fi
        rm -f "$instructions"
    }
    trap cleanup EXIT
    {
        printf 'set pagination off\nset confirm off\nset remotetimeout 20\n'
        printf 'target remote localhost:1234\n'
        printf 'break %s\ncontinue\n' "$breakpoint"
        printf '%s\n' "$@"
        printf 'disconnect\nquit\n'
    } > "$instructions"

    qemu-system-arm -M mps2-an386 -kernel "$elf" -S \
      -gdb tcp::1234 -serial null -monitor none -display none &
    pid=$!
    sleep 2
    kill -0 "$pid" 2>/dev/null || {
        echo "QEMU exited before GDB: $target" >&2
        return 1
    }
    set +e
    timeout 180s gdb-multiarch -nx -batch -x "$instructions" "$elf" \
      > "$transcript" 2>&1
    exitcode=$?
    set -e
    cat "$transcript"
    if [[ "$exitcode" -ne 0 ]]; then
        echo "$target: GDB returned $exitcode" >&2
        return "$exitcode"
    fi
    grep -Eq "^Breakpoint [0-9]+, $breakpoint" "$transcript" || {
        echo "$target did not hit the required breakpoint" >&2
        return 1
    }
    echo "Breakpoint hit: $target"
)

check_field() {
    local target="$1" name="$2" expected="$3"
    local log="$out_dir/$target.log"
    local actual
    actual=$(sed -n "s/^$name=//p" "$log" | tail -n 1)
    if [[ "$actual" != "$expected" ]]; then
        echo "$target: $name expected $expected, actual '$actual'" >&2
        return 1
    fi
}

stage4=qemu_stage4_trajectory_tests
run_qemu_test "$stage4" qemu_stage4_test_done \
  'printf "finished=%u\n", g_stage4_test_finished' \
  'printf "checks=%u\n", g_stage4_test_report.checks' \
  'printf "passed=%u\n", g_stage4_test_report.passed' \
  'printf "failed=%u\n", g_stage4_test_report.failed'
check_field "$stage4" finished 1
check_field "$stage4" checks 14
check_field "$stage4" passed 14
check_field "$stage4" failed 0

kin=qemu_kinematics_perf
run_qemu_test "$kin" qemu_kinematics_perf_done \
  'printf "finished=%u\n", g_kin_perf_finished' \
  'printf "sample_count=%u\n", g_kin_perf_report.sample_count' \
  'printf "fk_success=%u\n", g_kin_perf_report.fk_success' \
  'printf "ik_success=%u\n", g_kin_perf_report.ik_success' \
  'printf "selection_success=%u\n", g_kin_perf_report.selection_success' \
  'printf "roundtrip_passed=%u\n", g_kin_perf_report.roundtrip_passed' \
  'printf "roundtrip_failed=%u\n", g_kin_perf_report.roundtrip_failed' \
  'printf "roundtrip_skipped=%u\n", g_kin_perf_report.roundtrip_skipped' \
  'printf "worst_position_error_mm=%g\n", (double)g_kin_perf_report.worst_position_error_mm' \
  'printf "worst_rotation_element_error=%g\n", (double)g_kin_perf_report.worst_rotation_max_element_error'
check_field "$kin" finished 1
check_field "$kin" fk_success 128
check_field "$kin" ik_success 128
check_field "$kin" selection_success 128
check_field "$kin" roundtrip_passed 128
check_field "$kin" roundtrip_failed 0
check_field "$kin" roundtrip_skipped 0

ik=qemu_ik_near_singular_trace
run_qemu_test "$ik" qemu_ik_trace_done \
  'printf "finished=%u\n", g_ik_trace_done' \
  'printf "checks=%u\n", g_ik_reference_regression_checks' \
  'printf "failed=%u\n", g_ik_reference_regression_failures'
check_field "$ik" finished 1
check_field "$ik" checks 2
check_field "$ik" failed 0

echo "PASS: QEMU Stage4 trajectory 14/14, FK-IK 128/128, near-singular 2/2."
