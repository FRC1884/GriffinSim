# rendering-spi

Subscriber-only rendering contracts for GriffinSim.

Active use:
- `WorldSnapshotSubscriber` is the rendering boundary
- `HeadlessRendererSession` can be attached to deterministic co-simulation and scenario runs
- rendering remains downstream-only and cannot mutate authoritative state
