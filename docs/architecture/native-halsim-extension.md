# Native HALSIM Extension

## Purpose

The native HALSIM extension is the final low-level insertion point for GriffinSim into WPILib desktop simulation.

It will eventually be responsible for:
- registering HALSIM callbacks for PWM/CAN/sensor state
- enforcing queue-only callback behavior on the robot thread
- bridging native HAL state to the deterministic GriffinSim control-host/runtime pipeline

## Current repository status

This repository now contains a **packaged, buildable scaffold** under `native/halsim-extension/`.
That scaffold is included in rebuilt-platform release artifacts. It now supports two build modes:

- default standalone scaffold build
- optional WPILib HALSIM callback build via `GRIFFINSIM_ENABLE_WPILIB_HALSIM=ON`

## Current exported ABI

Lifecycle:
- `HALSIM_InitExtension`
- `HALSIM_ShutdownExtension`
- `GriffinSim_HalSimExtensionName`

Configuration:
- `GriffinSim_SetExtensionConfig`
- `GriffinSim_GetExtensionConfig`

Queue operations:
- `GriffinSim_EnqueuePwmEvent`
- `GriffinSim_DequeuePwmEvent`
- `GriffinSim_ResetPwmQueue`
- `GriffinSim_QueueCapacity`
- `GriffinSim_QueueSize`

Callback registration:
- `GriffinSim_RegisterPwmSpeedCallbacks`
- `GriffinSim_UnregisterPwmSpeedCallbacks`
- `GriffinSim_RegisteredPwmCallbackCount`

## Build path today

See also `docs/architecture/native-loading.md`.

Standalone scaffold:

```bash
cmake -S native/halsim-extension -B build/native-halsim-extension
cmake --build build/native-halsim-extension
```

Optional WPILib-native callback path:

```bash
cmake -S native/halsim-extension -B build/native-halsim-extension-wpilib -DGRIFFINSIM_ENABLE_WPILIB_HALSIM=ON
cmake --build build/native-halsim-extension-wpilib
```

When `GRIFFINSIM_ENABLE_WPILIB_HALSIM=ON`, the consumer must provide WPILib HAL/HALSIM native include and link settings.

## Current implementation scope

The scaffold now includes:
- deterministic bounded PWM queue state
- extension configuration storage
- optional PWM speed callback registration against `HALSIM_RegisterPWMSpeedCallback`
- callback unregistration and callback-count inspection
- WPILib-linked runtime validation that the extension can be auto-loaded via `HALSIM_EXTENSIONS` and receive live HALSIM PWM speed updates into the native queue
- callback registration logic that avoids holding the extension mutex across HALSIM registration/cancellation calls, preventing initial-notify deadlocks in real WPILib-linked runtime

## Remaining work

- extend native registration beyond PWM speed callbacks
- connect native queue state to the Java/runtime bridge flow
- prove full robot-output propagation from live WPILib user code into HALSIM PWM speed data in the same runtime harness
- package platform-specific binaries in release artifacts
