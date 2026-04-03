# GriffinSim Starter

This file is the high-context starting point for continuing GriffinSim work.

## What GriffinSim Is

GriffinSim is a sim-only library for FRC robot projects. It keeps Maple as the planar drivetrain backend, then layers field contact, terrain, traction/load transfer, and 3D pose/body behavior on top.

It is not a full native 6DOF rigid-body replacement yet. Current behavior is best described as `2.5D`:

- Maple owns `x`, `y`, and `yaw` drivetrain motion.
- GriffinSim owns terrain sampling, roll/pitch, body height, contact state, traction scaling, and sim-facing 3D pose.
- AdvantageScope can show the resulting `Pose3d`.
- Autonomous still uses the same `Pose2d` hooks as before.

## Current Release State

- Current released version: `0.1.7`
- Release URL: `https://github.com/FRC1884/GriffinSim/releases/tag/v0.1.7`
- Current pushed commit at time of writing: `1c07c5a`

## Current Architecture

Core interfaces and state:

- [DriveSimulationAdapter.java](/Users/jonathanst-georges/Documents/GriffinSim/src/main/java/org/griffins1884/sim3d/DriveSimulationAdapter.java)
- [DriveSimulationState.java](/Users/jonathanst-georges/Documents/GriffinSim/src/main/java/org/griffins1884/sim3d/DriveSimulationState.java)
- [ChassisState3d.java](/Users/jonathanst-georges/Documents/GriffinSim/src/main/java/org/griffins1884/sim3d/ChassisState3d.java)
- [SimImuSample.java](/Users/jonathanst-georges/Documents/GriffinSim/src/main/java/org/griffins1884/sim3d/SimImuSample.java)

Maple backend boundary:

- [SwerveDriveBackend.java](/Users/jonathanst-georges/Documents/GriffinSim/src/main/java/org/griffins1884/sim3d/SwerveDriveBackend.java)
- [MapleSwerveDriveBackend.java](/Users/jonathanst-georges/Documents/GriffinSim/src/main/java/org/griffins1884/sim3d/maple/MapleSwerveDriveBackend.java)

Terrain, contact, and body behavior:

- [TerrainAwareSwerveSimulation.java](/Users/jonathanst-georges/Documents/GriffinSim/src/main/java/org/griffins1884/sim3d/TerrainAwareSwerveSimulation.java)
- [TerrainContactModel.java](/Users/jonathanst-georges/Documents/GriffinSim/src/main/java/org/griffins1884/sim3d/TerrainContactModel.java)
- [TerrainContactSample.java](/Users/jonathanst-georges/Documents/GriffinSim/src/main/java/org/griffins1884/sim3d/TerrainContactSample.java)
- [TerrainDriveLaws.java](/Users/jonathanst-georges/Documents/GriffinSim/src/main/java/org/griffins1884/sim3d/TerrainDriveLaws.java)
- [SwerveLoadTransferEstimator.java](/Users/jonathanst-georges/Documents/GriffinSim/src/main/java/org/griffins1884/sim3d/SwerveLoadTransferEstimator.java)
- [SwerveTractionState.java](/Users/jonathanst-georges/Documents/GriffinSim/src/main/java/org/griffins1884/sim3d/SwerveTractionState.java)

2026 rebuilt field implementation:

- [Rebuilt2026FieldContactModel.java](/Users/jonathanst-georges/Documents/GriffinSim/src/main/java/org/griffins1884/sim3d/seasonspecific/rebuilt2026/Rebuilt2026FieldContactModel.java)
- [Rebuilt2026MapleArena.java](/Users/jonathanst-georges/Documents/GriffinSim/src/main/java/org/griffins1884/sim3d/seasonspecific/rebuilt2026/Rebuilt2026MapleArena.java)
- [Rebuilt2026RobotProfile.java](/Users/jonathanst-georges/Documents/GriffinSim/src/main/java/org/griffins1884/sim3d/seasonspecific/rebuilt2026/Rebuilt2026RobotProfile.java)

## What 0.1.7 Added

Release `0.1.7` tightened the model in four main ways:

1. The rebuilt field geometry was corrected to official coordinates for the hub and bump.
2. The trench-edge collision model now includes the corner return blockers at the open ends.
3. The default robot profile was retuned to a `135 lb` Kraken-based robot.
4. The body model now allows a brief unsupported phase when the robot crests and leaves the bump, instead of staying glued to terrain height.

The key files for that work are:

- [TerrainAwareSwerveSimulation.java](/Users/jonathanst-georges/Documents/GriffinSim/src/main/java/org/griffins1884/sim3d/TerrainAwareSwerveSimulation.java#L21)
- [Rebuilt2026FieldContactModel.java](/Users/jonathanst-georges/Documents/GriffinSim/src/main/java/org/griffins1884/sim3d/seasonspecific/rebuilt2026/Rebuilt2026FieldContactModel.java#L31)
- [Rebuilt2026MapleArena.java](/Users/jonathanst-georges/Documents/GriffinSim/src/main/java/org/griffins1884/sim3d/seasonspecific/rebuilt2026/Rebuilt2026MapleArena.java#L70)
- [Rebuilt2026RobotProfile.java](/Users/jonathanst-georges/Documents/GriffinSim/src/main/java/org/griffins1884/sim3d/seasonspecific/rebuilt2026/Rebuilt2026RobotProfile.java#L16)

## Field Geometry Source of Truth

The rebuilt 2026 field geometry in GriffinSim is based on official FIRST drawings and manual values, not only the CAD drop.

Important official references used:

- Field manual: `https://firstfrc.blob.core.windows.net/frc2026/FieldAssets/field-manual.pdf`
- Field dimensions: `https://firstfrc.blob.core.windows.net/frc2026/FieldAssets/2026-field-dimension-dwgs.pdf`
- Game-specific element drawings: `https://firstfrc.blob.core.windows.net/frc2026/FieldAssets/2026-field-dwg-game-specific.pdf`

Important values currently encoded:

- Field: `650.12 in x 316.64 in`
- Hub centers: blue `181.56 in`, red `468.56 in`, y `158.32 in`
- Hub collision envelope: `58.41 in x 47.00 in`
- Bump profile depth: `48.93 in`
- Bump span: `73.00 in`
- Bump height: `6.513 in`
- Trench footprint: `24.97 in x 47.00 in`
- Trench opening height: `22.25 in`
- Tower opening width: `32.25 in`

## Current Physics Model

What is physically modeled now:

- Planar swerve drivetrain motion from Maple
- Terrain height sampling
- Roll and pitch derived from terrain gradient
- Quasi-static load transfer from acceleration and chassis attitude
- Drive and steer authority scaling from contact/traction state
- Brief airborne body state when leaving the bump
- Reduced or zero traction when unsupported

What is not physically solved yet:

- True 3D rigid-body contact forces
- Wheel-by-wheel suspension travel
- Native 6DOF collision response
- True airborne ballistic yaw/roll coupling from wheel forces after takeoff
- Full mesh-based collision from CAD

That means the current sim is intentionally practical, not a full physics-engine rewrite.

## Season2026 Integration

The current `season2026` consumer uses GriffinSim only in `SIM`.

Important consumer-side files:

- [RobotContainer.java](/Users/jonathanst-georges/Documents/season2026/src/main/java/org/Griffins1884/frc2026/RobotContainer.java)
- [MapleArenaSetup.java](/Users/jonathanst-georges/Documents/season2026/src/main/java/org/Griffins1884/frc2026/simulation/maple/MapleArenaSetup.java)
- [Rebuilt2026FieldModel.java](/Users/jonathanst-georges/Documents/season2026/src/main/java/org/Griffins1884/frc2026/simulation/maple/Rebuilt2026FieldModel.java)
- [SwerveConstants.java](/Users/jonathanst-georges/Documents/season2026/src/main/java/org/Griffins1884/frc2026/subsystems/swerve/SwerveConstants.java#L190)
- [ModuleIOSim.java](/Users/jonathanst-georges/Documents/season2026/src/main/java/org/Griffins1884/frc2026/subsystems/swerve/ModuleIOSim.java)

The season repo currently uses:

- GriffinSim vendordep `0.1.7`
- 135 lb mass assumption
- Kraken drive and steer motor simulation
- Kraken gear ratios for sim modules

## Main Tests

Useful tests to extend first:

- [TerrainAwareSwerveSimulationTest.java](/Users/jonathanst-georges/Documents/GriffinSim/src/test/java/org/griffins1884/sim3d/TerrainAwareSwerveSimulationTest.java)
- [Rebuilt2026FieldContactModelTest.java](/Users/jonathanst-georges/Documents/GriffinSim/src/test/java/org/griffins1884/sim3d/seasonspecific/rebuilt2026/Rebuilt2026FieldContactModelTest.java)

These currently cover:

- composed 3D chassis state
- cache invalidation
- contact and traction exposure
- unsupported body state after cresting a bump
- official field dimensions
- trench corner return collision regions

## Build and Release Workflow

Local build:

```bash
cd /Users/jonathanst-georges/Documents/GriffinSim
./gradlew build
```

Publish local static Maven repo and build:

```bash
./gradlew publish build
```

Release steps used so far:

1. Update `gradle.properties` version.
2. Update `vendordeps/GriffinSim.json`.
3. Run `./gradlew publish build`.
4. Commit `src`, `vendordeps`, and `maven-repo`.
5. Push `main`.
6. Create the GitHub release and upload `vendordeps/GriffinSim.json`.

## Most Likely Next Improvements

If continuing physics work, the highest-value next steps are:

1. Replace the current hub collision envelope with a more exact floor-contact shape if the center still feels too broad.
2. Add explicit terrain edge normals and landing impulse heuristics instead of only vertical unsupported state.
3. Move from quasi-static traction scaling toward per-wheel contact support tests near trench and bump edges.
4. Add richer tower and outpost blocker geometry if driver interaction with those structures matters.
5. Eventually replace the Maple planar backend with a native GriffinSim drivetrain backend if true 6DOF chassis dynamics become necessary.

## Constraints To Remember

- Keep GriffinSim sim-only.
- Do not add bulky deploy assets to robot projects.
- Preserve robot-facing auto APIs where possible.
- Real robot and replay behavior should stay untouched.
- The current architecture is intentionally incremental: better simulation without forcing a full simulator rewrite yet.
