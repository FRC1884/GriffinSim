# Native Extension Loading Notes

## Build modes

### Standalone scaffold verification

```bash
cmake -S native/halsim-extension -B build/native-halsim-extension
cmake --build build/native-halsim-extension
```

### WPILib-linked build

```bash
./scripts/build_native_extension_with_wpilib.sh
```

If `GRIFFINSIM_WPILIB_HAL_INCLUDE_DIR` / `GRIFFINSIM_WPILIB_HAL_LIBRARY` are not set, the script now auto-discovers the HAL headers zip and host native library from the local WPILib Maven cache on supported desktop platforms.

## Expected desktop simulation integration flow

1. Build the native extension with WPILib HAL native headers/libs.
2. Make the resulting library discoverable by the desktop simulation process.
3. Ensure the extension is loaded before robot code begins producing actuator outputs.
4. Allow callback-side PWM events to flow into the native queue API.
5. Bridge queue data into GriffinSim's deterministic runtime.

## Current status

This repo now supports step 1 as a scriptable build path and documents steps 2–5, but it does not yet automate the desktop-sim loader wiring.

## Java-side runtime planning

The rebuilt platform now includes Java-side planning classes for native loading:

- `NativeExtensionArtifactLocator`
- `NativeExtensionLoadPlan`
- `NativeExtensionRuntimeConfig`
- `NativeExtensionRuntimeBridge`

These classes let the Java bridge reason about where a native artifact should live and whether it can be loaded, even before full WPILib-linked runtime validation is available.

## Java binding path

The rebuilt platform now includes a Java Foreign Function API binding layer in `modules/frc-bridge`:

- `PanamaNativeExtensionBindings`
- `NativeExtensionRuntimeBridge`

This currently validates the standalone native ABI end-to-end and provides the future seam for invoking WPILib-enabled native builds from Java.

## Java/native bridge composition

The rebuilt platform now includes a concrete composition path:

- `PanamaNativeExtensionBindings`
- `NativeBackedQueuedHalSimBridge`
- `NativeExtensionIntegrationSupport`

This path validates the standalone native queue ABI from Java and gives the repository a concrete starting point for future real HALSIM callback runtime integration.

`NativeExtensionIntegrationSupport.createBridgeWithPwmCallbacks(...)` now also provides a single Java-side seam that:
- loads the native extension
- initializes the queue-backed bridge
- requests native PWM callback registration
- keeps callback teardown coupled to bridge shutdown

That still stops short of a true desktop simulation proof, because it does not by itself make the library discoverable to or preloaded inside a live WPILib simulation process.

## Runtime validation path

The repository now also includes a WPILib desktop-runtime validation test:

- `PanamaWpilibNativeExtensionRuntimeTest`

That test forks a fresh JVM, sets `HALSIM_EXTENSIONS` to the WPILib-linked GriffinSim extension, initializes HAL on desktop, verifies the extension was auto-initialized by HAL before Java-side symbol access, registers native PWM callbacks, and confirms a live HALSIM PWM speed update reaches the native queue in a true WPILib-linked process.

This is runtime proof for the WPILib-linked callback path, but it is still narrower than a full user robot application launched through the standard desktop simulation UI stack, and it does not yet prove robot-side PWM writes propagate into HALSIM PWM speed data in the same harness.
