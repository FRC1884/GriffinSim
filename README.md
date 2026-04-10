# GriffinSim

GriffinSim is now a **deterministic FRC-compatible simulation monorepo**.

The active repository no longer builds the legacy Maple-centric 2.5D simulator from the root. That implementation has been archived under `legacy/griffinsim-v0/` so the new work can enforce a process boundary, fixed-step scheduling, replayability, and contract-first development.

## Active layout

- `contracts/schema/` — versioned protocol schemas
- `contracts/spec/` — architecture and determinism contracts
- `docs/transformation/` — audit report, gap analysis, migration plan, and rewrite rationale
- `modules/sim-contracts/` — deterministic frame and snapshot contracts plus binary codecs
- `modules/deterministic-runtime/` — fixed-step scheduler, bounded queues, replay log, and diff logic
- `modules/frc-bridge/` — control-host and WPILib lockstep bridge scaffolding
- `modules/physics-core/` — authoritative rigid-body/contact stepping scaffold, field presets, and actuator-command mappers
- `modules/sensor-emulation/` — deterministic sensor pipelines with seeded noise and latency
- `modules/rendering-spi/` — subscriber-only render interfaces exercised by scenario execution
- `apps/scenario-runner/` — CLI for deterministic scenario execution
- `apps/replay-diff/` — CLI for replay log comparison
- `legacy/griffinsim-v0/` — archived pre-rebuild implementation; not part of the active build

## Build

```bash
./gradlew test
```

## Current rebuilt capabilities

- deterministic replay and diff-ready logs
- bounded non-blocking bridge queues
- lockstep and real-time host abstractions
- season-aware rebuilt-2026 field contact presets
- material-aware contact telemetry
- multi-body, gamepiece, and multi-robot interaction scaffolding
- external scenario loading from properties files
- controller layers for PWM, 1D waypoint, 2D holonomic, pose, and velocity-aware pose scenarios

## Example scenarios

- `scenarios/examples/two-robot-head-on.properties`
- `scenarios/examples/holonomic-pose-controller.properties`
- `scenarios/examples/velocity-pose-controller.properties`

See also:
- `docs/architecture/scenario-spec.md`
- `docs/architecture/controller-layers.md`
- `docs/milestone/rebuilt-platform-milestone.md`
- `docs/milestone/release-process.md`

## Release workflow

- `./gradlew releaseReadiness` — module/app verification suite
- `./scripts/release_smoke.sh` — deterministic example-scenario smoke run
- `./gradlew milestoneBundle` — zip milestone docs, contracts, scenarios, and scripts
- `./gradlew releaseArtifacts` — stage milestone bundle + CLI distributions + checksums + manifest
- `./gradlew nativeExtensionBundle` — package the HALSIM native extension scaffold
- `./scripts/prepare_native_extension.sh` — stage the native extension scaffold bundle
- `./scripts/build_native_extension_with_wpilib.sh` — build the native scaffold against WPILib HAL when local native deps are available
- `./gradlew nativeWpilibValidation` — optional WPILib-linked native build + Java binding validation
- `./scripts/validate_native_extension_with_wpilib.sh` — script wrapper for the WPILib-linked validation flow
- `./scripts/stage_rebuilt_artifacts.sh` — local artifact staging helper
- `./scripts/publish_rebuilt_artifacts.sh` — publish artifacts into a repo-local staging directory
- `./scripts/prepare_rebuilt_release.sh` — end-to-end rebuilt-platform prep flow


## Scenario runner CLI

Run a properties-backed scenario for a fixed number of control ticks:

```bash
./gradlew :apps:scenario-runner:run --args="scenarios/examples/two-robot-head-on.properties 2 build/example-replay.bin"
```

This prints a summary of the final world state and optionally writes replay bytes to the output path.
It also reports how many authoritative world snapshots were emitted and consumed by the headless renderer subscriber during the run.

Compare replay outputs with:

```bash
./gradlew :apps:replay-diff:run --args="build/example-replay-a.bin build/example-replay-b.bin"
```

## Why the rebuild happened

The archived system violated the target architecture in four fundamental ways:

1. It depended on Maple and dyn4j instead of a process-separated authoritative simulator.
2. It advanced simulation from wall-clock timestamps instead of deterministic lockstep.
3. It had no HALSIM/WebSocket/IPC bridge, no replay log, and no schema-versioned transport.
4. Rendering/integration concerns lived alongside simulation math instead of behind strict contracts.

See `docs/transformation/griffinsim-audit-report.md` and `docs/transformation/original-vs-rebuilt.md` for the full rationale.
