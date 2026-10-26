/*
 * Minimal newlib libm errno hook for Cortex-M4 freestanding targets.
 *
 * Build contract:
 * - The ARM firmware and standalone QEMU programs use -nostdlib.
 * - We explicitly link libm but do not link a full C runtime/newlib libc.
 * - Some GNU Arm Toolchain libm wrappers (e.g., sqrt, acos) reference
 *   __errno() even when all expected inputs are within their domains.
 *
 * This shim satisfies that ABI symbol without importing libc/syscalls.
 * It is NOT a general thread-local errno implementation. The project
 * checks math domains and numerical validity explicitly and never uses
 * errno for control decisions. Only the libm wrappers need this storage.
 *
 * If the project later enables a full newlib runtime or adds code that
 * consumes errno, remove this shim and adopt properly reentrant
 * per-task errno via the runtime/FreeRTOS integration instead.
 *
 * Kept in platform/, not algorithms/, Robot Model or task config.
 */

static int freestanding_libm_errno_storage = 0;

int *__errno(void)
{
    return &freestanding_libm_errno_storage;
}
