# Robot Integration

GriffinSim is meant to be consumed only in desktop simulation code paths.

## Goals

- keep simulation-specific dependencies out of roboRIO deploys
- let autonomous code keep using `Pose2d` and robot-relative chassis speeds
- expose richer `Pose3d`, terrain contact, and traction state for visualization and analysis

## Local Composite Build

From a robot repo, add GriffinSim as an included build in `settings.gradle`:

```gradle
includeBuild("../GriffinSim")
```

Then add the dependency in `build.gradle`:

```gradle
dependencies {
    simulationImplementation "org.griffins1884:GriffinSim:${VERSION_NAME}"
}
```

If the robot repo does not use a dedicated `simulationImplementation` configuration, keep the dependency behind a sim-only guard in code and do not package GriffinSim into deploy artifacts.

## SIM-Only Construction

Create GriffinSim-backed objects only when running simulation. Real and replay modes should continue to use normal robot code paths.

Typical pattern:

```java
if (mode == SIM) {
  var simulation = new TerrainAwareSwerveSimulation(...);
}
```

## Autonomous Hooks

Use `DriveSimulationIntegrations.holonomicAutoHooks(...)` to obtain:

- `Pose2d` supplier
- robot-relative `ChassisSpeeds` supplier
- pose reset callback
- optional `ChassisState3d`, terrain-contact, and traction suppliers for advanced tooling

These hooks map cleanly onto PathPlanner- and Choreo-style integrations without changing the autonomous API shape in the robot repo.

## Visualization Hooks

Use `DriveSimulationIntegrations.visualizationFrame(...)` to generate a single snapshot containing:

- robot `Pose3d`
- chassis state
- IMU sample
- terrain contact
- traction state
- static field markers

This is intended to be forwarded to AdvantageScope or any repo-local logger adapter.

## Deploy Safety

Do not place GriffinSim assets under a robot repo's `src/main/deploy`.

Keep GriffinSim usage limited to:

- sim-only Java code
- desktop-only dependency configurations
- build/test tooling

That separation is the primary reason GriffinSim exists as its own repository.

## Released Vendordep

For season repos that should consume a published GriffinSim build instead of a local checkout:

1. Download `GriffinSim.json` from the GitHub release asset.
2. Place it in the robot repo's `vendordeps/` directory.
3. Remove any temporary local `includeBuild("../GriffinSim")` wiring.
4. Remove any hand-written GriffinSim dependency line that was only there for local development.

The vendordep routes GriffinSim through JitPack and keeps Maple's public Maven repository available for the current backend dependency chain.
