# Native 6DOF Backend Plan

## Goal

Add a GriffinSim-native simulation path that no longer depends on Maple for chassis motion and can grow into:

- full rigid-body 6DOF state ownership
- wheel-by-wheel contact force solving
- swerve wheel force generation with steer-angle-dependent traction
- airborne free-flight with yaw/roll/pitch takeover
- body and underbody contact against terrain and field structures

This is a new backend track, not a patch to the current Maple wrapper.

## Constraints

- keep `DriveSimulationAdapter` as the stable robot-facing API
- keep SIM-only usage and avoid any robot deploy assets
- do not break existing Maple-based construction paths
- do not model suspension travel because the target robot is a rigid swerve chassis without suspension

## Architecture

### Existing Path

- `TerrainAwareSwerveSimulation` wraps Maple
- Maple owns planar `x/y/yaw`
- GriffinSim layers terrain/body state on top

### New Path

Add `NativeSixDofSwerveSimulation` implementing `DriveSimulationAdapter` directly.

This new adapter owns:

- world pose and Euler attitude
- linear and angular velocity
- commanded robot-relative chassis speeds
- module steer states
- per-wheel normal and friction forces
- contact-derived traction state

## Phase Breakdown

### Phase 1: Minimum Native Path

- add native config objects for rigid body and wheel geometry
- add a native simulation adapter with wall-clock stepping
- solve per-wheel contact on terrain with compliant normal forces
- generate wheel longitudinal and lateral forces from swerve module steer states
- integrate free-flight when no contacts exist
- expose `DriveSimulationState`, `SimImuSample`, and `SwerveTractionState`

This phase should be enough to validate:

- gravity and landing behavior
- wheel-by-wheel support changes
- airborne yaw/roll persistence
- contact-driven traction distribution

### Phase 2: Hard Contact and Body Contact

- add underbody contact primitives for frame/belly interaction
- add explicit rigid-body contact against hub, trench returns, tower, and other blockers
- move field geometry into reusable contact primitives shared by both terrain/contact classification and native collision

### Phase 3: Command and Module Fidelity

- replace simple chassis-speed tracking with module-level wheel spin and steer dynamics
- add drive torque limits and steering inertia/rate limits
- use friction-circle-based force allocation per wheel

### Phase 4: Integration Selection

- add a SIM-only construction path to choose Maple-backed or native-backed simulation
- keep autonomous and visualization consumers on `DriveSimulationAdapter`

## First Implementation Scope

The first code pass in this branch will implement Phase 1 only:

- native rigid-body state
- per-wheel contact and friction
- free-flight takeover
- focused tests

It will intentionally not yet:

- replace season repo wiring
- remove Maple
- solve mesh-level rigid-body collision
- implement underbody/hub/tower hard contacts in the new native backend

## Key Modeling Assumptions

- rigid chassis, no suspension travel
- compliant wheel contact using spring-damper normal forces
- diagonal inertia approximation
- steer-angle-based wheel force directions
- simple velocity-tracking wheel force controller initially, to be replaced by wheel torque dynamics later
