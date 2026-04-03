# Original GriffinSim vs Rebuilt System

## Original system

- Single Java library
- Maple/dyn4j-oriented
- Wall-clock time via `System.nanoTime()`
- No HALSIM extension
- No replay log or deterministic diff tool
- In-process integration factories for autonomous and visualization

## Rebuilt system foundation introduced here

- Multi-project monorepo
- Contract-first binary protocol and snapshot schemas
- Deterministic scheduler and bounded queues
- Replay log writer/reader/diff tool
- Lockstep control-host scaffolding around WPILib `SimHooks`
- Legacy system archived outside the active build

## Why the rewrite was necessary

The old architecture could produce useful exploratory visuals, but it could not prove that the same inputs and seed would yield identical outputs. The rebuilt structure makes determinism, queue ordering, and explicit step ownership the foundation instead of an afterthought.
