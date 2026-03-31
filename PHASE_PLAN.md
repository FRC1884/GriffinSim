# GriffinSim Phase Plan

## Goal

Build a separate simulation repository that can grow from a Maple-compatible bridge into a real 6DOF drivetrain and terrain simulator for FRC use.

The end state should provide:
- chassis translation and rotation in 3D
- roll, pitch, and yaw outputs
- terrain interaction that affects wheel loading and traction
- autonomous compatibility without requiring teams to rewrite their path-following code
- desktop-only simulation dependencies, with no bulky simulator payload deployed to the roboRIO

## Phase 0: Repository Bootstrap

Deliverables:
- standalone `GriffinSim` repository
- buildable Java library scaffold
- compatibility contracts for terrain sampling and simulator adapters
- roadmap and architecture notes for the 6DOF transition

Exit criteria:
- repository builds on its own
- can be consumed from another Gradle project as a local dependency

## Phase 1: Compatibility Layer

Deliverables:
- stable simulator-facing API for pose, chassis state, IMU state, and reset hooks
- adapter layer that lets existing robot projects keep using current `Pose2d`-based autonomous stacks
- shared `Pose3d` and terrain sample outputs for visualization

Exit criteria:
- teleop and autonomous code can run in simulation without changing their public interfaces
- robot repos can switch between stock Maple and GriffinSim-backed simulation behind a factory or adapter

## Phase 2: 6DOF Chassis Core

Deliverables:
- explicit rigid-body chassis state: position, velocity, orientation, and angular rates
- roll, pitch, and yaw integrated directly into the simulation state
- desktop-only IMU outputs for yaw, pitch, roll, and angular velocity

Exit criteria:
- simulator no longer depends on a planar pose as the only source of truth for orientation
- simulated IMU data can drive visualization and higher-level robot logic

## Phase 3: Terrain and Contact Model

Deliverables:
- 2026 field terrain model with bump and trench geometry
- contact resolution between the chassis, wheels, and terrain
- support for crossing the bump and passing under field structures where legal

Exit criteria:
- terrain interaction produces meaningful height, roll, and pitch changes
- no false blocking for underpass or traversable terrain features

## Phase 4: Wheel Loading and Friction

Deliverables:
- wheel normal-force estimation based on chassis attitude and terrain contact
- traction and friction behavior that changes with loading
- drivetrain response that differs on flat ground, ramps, and transitions

Exit criteria:
- acceleration, turning, and slip behavior change plausibly with terrain
- autonomous and teleop both see the same terrain-dependent drivetrain response

## Phase 5: Autonomous and Tooling Integration

Deliverables:
- clean hooks for PathPlanner, Choreo, and other `Pose2d`-based autonomous code
- AdvantageScope outputs for `Pose3d`, component poses, terrain markers, and IMU state
- repeatable simulation scenarios for bump crossings and obstacle-adjacent autos

Exit criteria:
- autonomous paths run in GriffinSim without API redesign in robot code
- 3D visualization reflects the same state the simulator uses internally

## Phase 6: Validation and Packaging

Deliverables:
- regression tests for dynamics, terrain traversal, and API behavior
- publishable versioning strategy
- clear documentation for sim-only integration into robot projects

Exit criteria:
- standalone build and tests pass
- robot repos can consume GriffinSim without dragging simulation assets into deploy outputs

## Non-Goals for the Initial Phase

The first phase does not attempt to deliver:
- full game-piece simulation
- a finished native 6DOF solver
- custom field art assets
- roboRIO runtime support for heavy simulation code
