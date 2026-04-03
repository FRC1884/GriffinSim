# GriffinSim HALSIM Native Extension Scaffold

This directory now contains a **buildable source scaffold** for a future GriffinSim HALSIM native extension.

Current contents:
- CMake build entry point
- exported ABI header
- queue-safe PWM event ring buffer implementation
- extension configuration storage
- placeholder extension lifecycle hooks

## What currently works

The scaffold can be compiled as a standalone C++ library without WPILib native dependencies.
It exposes a deterministic queue API that mirrors the Java-side bridge assumptions:
- callback-side enqueue only
- bounded queue capacity
- explicit dequeue/reset from the bridge/runtime side

## What is still missing

- real WPILib HALSIM callback registration against native HAL APIs
- native bridge to Java or external IPC runtime
- platform-specific release binaries
- validation inside an actual desktop WPILib simulation launch
