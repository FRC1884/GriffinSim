# Scenario Spec Loader

GriffinSim now supports loading deterministic multi-robot scenarios from external Java-properties style configuration.

## Required keys

- `scenario.name`
- `scenario.robots`
- `world.bodies`

## Control keys

- `control.clock_mode`
- `control.physics_step_nanos`
- `control.control_step_nanos`
- `control.queue_capacity`

## Body keys

Per body in `world.bodies`:
- `world.body.<id>.x`
- `world.body.<id>.y`
- `world.body.<id>.z`
- optional velocities: `vx`, `vy`, `vz`
- collision extents: `collision.<id>.half_extent_x|y|z`
- collision material: `collision.<id>.material`

## Robot endpoint keys

Per robot in `scenario.robots`:
- `robot.<id>.pwm_channel`
- `robot.<id>.pwm_value`
- `robot.<id>.sensor_channel`
- `robot.<id>.sensor_seed`
- `robot.<id>.mass_kg`
- `robot.<id>.inertia`
- `robot.<id>.sensor_latency_nanos`
- `robot.<id>.passive`

## Contact keys

- `contact.field_preset` = `simpleArena` or `rebuilt2026Arena`
- `contact.enable_pairwise` = `true|false`
