# GriffinSim

GriffinSim is a standalone simulator project for FRC drivetrain and field interaction work.

The starting point in this repository is intentionally narrow:
- a Java library that can plug into existing WPILib and Maple-based robot projects
- shared terrain sampling and 3D pose state for simulation
- the first compatibility layer needed to transition from planar simulation toward a true 6DOF stack

The long-term direction is a desktop-only simulator with:
- 6DOF chassis state
- roll, pitch, and yaw dynamics
- terrain-aware wheel loading and friction
- field obstacle interaction
- AdvantageScope-friendly 3D outputs
- clean integration with autonomous code without changing robot-side control APIs

This repository is kept separate from robot code on purpose:
- simulation work should not bloat roboRIO deploy artifacts
- native or heavyweight desktop-only dependencies stay out of competition builds
- simulator architecture can evolve on its own release cadence

## Current Scope

Today this repository contains:
- a terrain-aware wrapper around Maple's `SwerveDriveSimulation`
- backend abstractions that separate the public simulator API from Maple-specific types
- shared `TerrainModel`, `TerrainSample`, `DriveSimulationState`, `ChassisState3d`, and `SimImuSample` contracts
- the first reusable layer for feeding roll, pitch, height, and IMU state into simulation consumers
- a first-class 3D chassis snapshot with pose, angular rates, and field-relative linear velocity and acceleration
- a standalone 2026 rebuilt field contact model with explicit bump surfaces and trench underpass clearance checks
- a quasi-static swerve load-transfer estimator that converts terrain/contact state into per-wheel normal force and traction capacity estimates
- shared terrain-drive authority laws that robot repos can apply to module and steering response
- a GriffinSim-owned 2026 Maple arena with hub, trench edge, tower, and border obstacles
- default rebuilt-2026 robot profile values for chassis footprint and mass properties
- integration helpers for holonomic autonomous libraries, visualization frames, static field markers, and repeatable 2026 auto scenarios

That is not the final 6DOF engine yet. The phased roadmap is in [PHASE_PLAN.md](PHASE_PLAN.md).

## Build

```bash
./gradlew build
```

## Release And Integration

- Versioning strategy: [VERSIONING.md](VERSIONING.md)
- Robot-repo consumption guidance: [docs/ROBOT_INTEGRATION.md](docs/ROBOT_INTEGRATION.md)
- Vendordep file: [vendordeps/GriffinSim.json](vendordeps/GriffinSim.json)

## Early Integration Goal

Robot repositories should depend on GriffinSim only in desktop simulation configurations. Real hardware and deploy packaging should continue to use normal robot code paths with no simulator-specific assets copied to the roboRIO.
