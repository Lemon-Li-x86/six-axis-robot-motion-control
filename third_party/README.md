# Third-party Dependencies

## FreeRTOS Kernel

The firmware uses the upstream FreeRTOS Kernel, but does not vendor
its source into this repository.

For reproducible development, use the same tagged upstream release
as the ARM build job: `V11.1.0`.

From the project root:

```powershell
git clone --depth 1 --branch V11.1.0 https://github.com/FreeRTOS/FreeRTOS-Kernel.git third_party/FreeRTOS-Kernel
```

When an existing local clone uses another revision, check it before
building; do not assume the kernel headers and port match the CI version.
The source directory expected by CMake is `third_party/FreeRTOS-Kernel`.
