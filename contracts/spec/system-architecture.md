# GriffinSim System Architecture Contract

## Active modules

- `modules/sim-contracts`: immutable transport and snapshot contracts
- `modules/deterministic-runtime`: fixed-step scheduling, bounded queues, replay log infrastructure
- `modules/frc-bridge`: control-host orchestration and WPILib lockstep hooks
- `modules/physics-core`: authoritative rigid-body/contact stepping scaffold, field presets, and actuator-command mappers
- `modules/sensor-emulation`: deterministic sensor pipelines with seeded noise and latency queues
- `modules/rendering-spi`: subscriber-only render interfaces used by scenario execution
- `apps/replay-diff`: deterministic replay comparison CLI
- `apps/scenario-runner`: deterministic scenario execution CLI

## Determinism rules

1. Physics advances only on scheduler-owned fixed timesteps.
2. Control ticks are tagged with monotonically increasing `step_id` values.
3. Sensor frames are applied before robot stepping for the same `step_id`.
4. Actuator frames are emitted after robot stepping for that `step_id`.
5. Replay logs are append-only binary records with protocol version headers.
6. Rendering and dashboards are subscribers only and may never mutate authoritative state.
