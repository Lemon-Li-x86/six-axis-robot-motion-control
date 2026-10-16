# Native motion limits regression test

This is a **host-only** CMake project. It compiles the existing
`tests/motion/motion_limits_test_main.c` and the production
`algorithms/motion/motion_limits.c` against the same UR5 model used
by the QEMU firmware. No duplicate test cases or algorithm copy is added.

Run from repository root:

```powershell
cmake -S firmware/freertos_demo/tests/host -B build/motion-host
cmake --build build/motion-host
ctest --test-dir build/motion-host --output-on-failure
```

The existing test entry point returns a nonzero exit code on failures
when `MOTION_LIMITS_HOST_TEST` is enabled. It covers null arguments,
position boundaries, ±180° wrap, J3 limits, tie-breaking and output
preservation on failure.

This does **not** replace the Cortex-M4/QEMU target or end-to-end
UART/PyBullet regression. It adds a fast native gate before changing
motion algorithms.

The test does not require Unity. Existing Stage 2 Unity tests are a
separate build project under `tests/unity`.

If a build fails, do not mark it as passed: capture the compiler output
and the test report before proceeding.

The test report has a fixed capacity of 40 assertions. Check that the
recorded total does not exceed this capacity when adding new cases;
otherwise the report records overflow as a failure.
