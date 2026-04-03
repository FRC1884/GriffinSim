# Rebuilt Platform Release Process

## Goal

Produce a repeatable, deterministic milestone bundle for the rebuilt GriffinSim platform.

## Primary commands

### 1. Full verification

```bash
./gradlew releaseReadiness
```

### 2. Release smoke run

```bash
./scripts/release_smoke.sh
```

This command:
- runs the full test suite
- executes example scenarios through `scenario-runner`
- writes replay files under `build/release-smoke/`
- uses `replay-diff` to prove repeated runs match byte-for-byte

### 3. Milestone bundle

```bash
./gradlew milestoneBundle
```

Produces a zip under `build/distributions/` containing:
- milestone docs
- architecture docs
- contracts
- example scenarios
- release helper scripts

### 4. End-to-end prep

```bash
./scripts/prepare_rebuilt_release.sh
```

This is the closest thing to a current release cut workflow for the rebuilt platform.

## Current release caveat

This process prepares a deterministic milestone package, not a final production simulator release. The native HALSIM extension artifact and full rigid-body solver are still future work.

## Artifact staging helpers

```bash
./scripts/stage_rebuilt_artifacts.sh
./scripts/publish_rebuilt_artifacts.sh
```

The publish helper only stages artifacts to a directory inside the repository boundary (default: `build/publish-staging/`).

## Native extension scaffold bundle

```bash
./gradlew nativeExtensionBundle
./scripts/prepare_native_extension.sh
```

This packages the HALSIM native extension source scaffold and accompanying integration docs.

## WPILib-linked native extension build

```bash
export GRIFFINSIM_WPILIB_HAL_INCLUDE_DIR=/path/to/wpilib/hal/include
export GRIFFINSIM_WPILIB_HAL_LIBRARY=/path/to/libwpiHal.so
./scripts/build_native_extension_with_wpilib.sh
```

This is an opt-in path for environments that already have WPILib native HAL artifacts available.

## Native binary staging in release artifacts

If `build/native-halsim-extension/` or `build/native-halsim-extension-wpilib/` contains a built library, `./gradlew releaseArtifacts` will copy it into `build/release-artifacts/native-bin/` and include checksum/manifest entries.

## Optional WPILib-linked runtime validation

```bash
export GRIFFINSIM_WPILIB_HAL_INCLUDE_DIR=/path/to/wpilib/hal/include
export GRIFFINSIM_WPILIB_HAL_LIBRARY=/path/to/libwpiHal.so
./gradlew nativeWpilibValidation
```

This builds the WPILib-linked native extension and runs the Java/FFI validation test against the produced binary.
