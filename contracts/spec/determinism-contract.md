# GriffinSim Determinism Contract

A GriffinSim run is deterministic when the following tuple is identical across runs:

- protocol version
- scenario/world definition
- operator inputs
- random seeds for all noise models
- fixed step durations
- robot program build

## Required invariants

- identical input tuple -> identical replay log bytes
- step IDs are contiguous with no reuse
- sensor frames older than the last applied step are dropped
- bounded queues fail fast instead of blocking the robot thread
- no module may sample wall-clock time to decide simulation progress
