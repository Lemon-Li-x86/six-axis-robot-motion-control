# System Design

## Overall Architecture

Cortex-M4 / QEMU
    ↓
FreeRTOS
    ↓
Embedded Control Firmware
    ↓
UART Binary Protocol
    ↓
Python
    ↓
PyBullet
    ↓
UR5 Robot Simulation

## Layers

- Driver Layer: GPIO, UART, Timer
- Communication Layer: binary protocol and command parsing
- Algorithm Layer: kinematics, trajectory planning, PID
- Application Layer: robot task logic
