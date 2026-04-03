# GriffinSim Gap Analysis

| Area | Legacy state | Required state | Severity |
|---|---|---|---|
| Process model | Single in-process Java library | Split control host, bridge, authoritative physics process, subscriber rendering | BLOCKER |
| Control timing | Wall-clock / getter-driven updates via `System.nanoTime()` | Fixed-step, scheduler-owned lockstep with 20 ms control ticks and smaller physics substeps | BLOCKER |
| WPILib integration | No HALSIM extension, no `SimHooks`, no HAL state injection/extraction | Deterministic HALSIM bridge with non-blocking callback queueing and optional real-time mode | BLOCKER |
| Transport | No IPC boundary, no message ordering, no schema versioning | Versioned binary I/O bus plus compatibility mode | BLOCKER |
| Replayability | No replay log or diff tool | Byte-comparable replay logs with deterministic diff reporting | BLOCKER |
| Physics authority | Maple planar drive or experimental wall-clock 6DOF | Authoritative 6DOF physics core with canonical body/contact ordering | BLOCKER |
| Sensor modelling | Mostly direct state exposure, no latency queue architecture | Seeded noise, timestamped sensor frames, latency queues, deterministic delivery | BLOCKER |
| Rendering | Visualization helpers bundled with simulation library | Pure subscriber of `WorldSnapshot` data; headless-safe | MAJOR |
| World / season data | 2026 geometry embedded directly in legacy packages | Rewritten season/world modules behind the new contracts | MAJOR |
| Test strategy | Unit tests for legacy heuristics | Determinism tests, replay tests, queue ordering tests, lockstep host tests | BLOCKER |
| Release packaging | Vendordep + static Maven repo for old library | Rebuild-first release process after contracts/runtime/bridge prove deterministic | MAJOR |

## Net assessment

Legacy GriffinSim was an exploratory simulation library. The target system is a deterministic orchestration platform.

That is a **system replacement**, not an extension.
