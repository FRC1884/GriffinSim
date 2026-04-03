# GriffinSim Migration Plan

## Phase 0 — Controlled destruction and contract freeze

Completed in this changeset:
- Archive the legacy implementation under `legacy/griffinsim-v0/`
- Remove legacy code from the active build
- Create the new monorepo structure
- Freeze architecture in `contracts/spec/` and `contracts/schema/`
- Add deterministic runtime, replay, and bridge scaffolding

Risk mitigation:
- Legacy code remains available for read-only reference but is impossible to compile accidentally from the active root.

## Phase 1 — Deterministic transport and replay backbone

Completed in this changeset:
- Add versioned binary frame/snapshot contracts
- Add deterministic binary codecs
- Add bounded non-blocking ring buffer
- Add replay log writer/reader/diff tooling
- Add tests for byte-identical encoding and replay comparison

Risk mitigation:
- The transport is deliberately minimal and headered with protocol version + step ID before physics or HALSIM integration expands it.

## Phase 2 — Control-host and HALSIM bridge

Started in this changeset:
- Add `frc-bridge` module with control-host config, lockstep coordinator, and `SimHooks`-backed time controller
- Add tests that prove sensor application precedes robot stepping and actuator capture is step-tagged

Next work:
- Replace placeholder robot/sensor interfaces with a concrete HALSIM extension and queue-backed callback plumbing
- Add real actuator extraction and sensor injection against HAL/SimDevice state

Risk mitigation:
- The control host is already structured around non-blocking queues so the JNI/HAL layer can slot in without changing scheduler semantics.

## Phase 3 — Authoritative physics and sensor emulation

Next work:
- Implement `modules/physics-core` as the only authoritative world-state owner
- Rebuild 2026 field geometry inside a season/world module under the new contracts
- Implement seeded latency/noise pipelines in `modules/sensor-emulation`
- Add deterministic replay tests against identical operator inputs and seeds

Risk mitigation:
- Physics core will target canonical ordering first and performance second.

## Phase 4 — Rendering, compatibility, and release

Next work:
- Implement rendering subscriber interfaces and headless adapters
- Add optional WebSocket compatibility bridge for external tools
- Reintroduce release packaging only after replay stability is proven
- Publish rewrite documentation comparing archived vs rebuilt systems

Risk mitigation:
- Rendering remains downstream-only and cannot become authoritative by construction.
