# GriffinSim Audit Report

This audit was performed against the repository state archived under `legacy/griffinsim-v0/` plus the pre-rebuild root files that were active before the restructure.

## Audit summary

The archived GriffinSim implementation is **not** a valid base for the target system. It is a Maple-centric in-process simulation library with wall-clock-driven sampling and no WPILib HALSIM bridge, no lockstep scheduler, no replay log, and no versioned IPC boundary.

### Primary findings

1. **Determinism violations**
   - `TerrainAwareSwerveSimulation` and `NativeSixDofSwerveSimulation` default to `System.nanoTime()` instead of an authoritative step clock.
   - The native 6DOF adapter advances on accessor calls (`getState()`, `getPose2d()`, etc.), which couples simulation progress to observation order.
2. **Threading / integration violations**
   - There is no HALSIM extension, no callback queueing, and no robot-thread-safe IPC seam.
   - There is no `SimHooks.pauseTiming()` / `SimHooks.stepTiming()` orchestration layer.
3. **Architecture violations**
   - Physics, terrain modelling, integration factories, season assets, and release artifacts lived in one library.
   - Rendering/visualization contracts were embedded in the same package tree as simulation logic.
4. **Reproducibility violations**
   - No replay log, no input log, no deterministic diff tool, no schema versioning.
5. **Legacy release liability**
   - `maven-repo/` and `vendordeps/` published the old architecture directly.

## Classification table

| Path | Class | Reason |
|---|---|---|
| `.github/workflows/ci.yml` | MODIFY | CI remains useful, but it must validate the rebuilt multi-project monorepo and determinism tests instead of the legacy single-library build. |
| `build.gradle` | MODIFY | Gradle stays, but the root had to become a monorepo aggregator instead of a Maple-oriented `java-library` project. |
| `settings.gradle` | MODIFY | Useful entry point, but now must include contract/runtime/bridge/apps subprojects. |
| `gradle.properties` | MODIFY | Keep version/group management, but reset versions for the rebuilt architecture. |
| `README.md` | MODIFY | The old README described a standalone terrain-aware wrapper; it had to be rewritten around deterministic process separation. |
| `legacy/griffinsim-v0/src/main/java/org/griffins1884/sim3d/AngularVelocity3d.java` | MODIFY | Value-object idea is reusable, but package location and role must move into contract modules with explicit versioning. |
| `legacy/griffinsim-v0/src/main/java/org/griffins1884/sim3d/ChassisFootprint.java` | MODIFY | Domain concept is reusable, but it belonged to the old monolith and lacked transport/schema ownership. |
| `legacy/griffinsim-v0/src/main/java/org/griffins1884/sim3d/ChassisMassProperties.java` | MODIFY | Same as above; useful concept, wrong ownership boundary. |
| `legacy/griffinsim-v0/src/main/java/org/griffins1884/sim3d/ChassisState3d.java` | MODIFY | Snapshot concept is reusable, but must live behind deterministic snapshot contracts. |
| `legacy/griffinsim-v0/src/main/java/org/griffins1884/sim3d/SimImuSample.java` | MODIFY | Sensor sample concept is reusable, but must be emitted by the sensor layer, not the simulation monolith. |
| `legacy/griffinsim-v0/src/main/java/org/griffins1884/sim3d/TerrainSample.java` | MODIFY | Reusable as a world/sensor concept after relocation to explicit contracts. |
| `legacy/griffinsim-v0/src/main/java/org/griffins1884/sim3d/TerrainContactSample.java` | MODIFY | Reusable concept, but must be regenerated under deterministic physics/sensor contracts. |
| `legacy/griffinsim-v0/src/main/java/org/griffins1884/sim3d/TerrainFeature.java` | MODIFY | Enumerations remain useful, but belong in season/world modules instead of the root monolith. |
| `legacy/griffinsim-v0/src/main/java/org/griffins1884/sim3d/WheelLoadSample.java` | MODIFY | Reusable data only; solver ownership has to move to the new physics core. |
| `legacy/griffinsim-v0/src/main/java/org/griffins1884/sim3d/DriveSimulationAdapter.java` | DELETE | This interface exposed in-process accessors only; it has no notion of IPC, step IDs, or authoritative process boundaries. |
| `legacy/griffinsim-v0/src/main/java/org/griffins1884/sim3d/CommandableDriveSimulationAdapter.java` | DELETE | Commanding via direct method calls violates the required control-host ↔ physics-process split. |
| `legacy/griffinsim-v0/src/main/java/org/griffins1884/sim3d/DriveSimulationState.java` | DELETE | In-process state aggregation without schema versioning or replay headers is incompatible with the new transport. |
| `legacy/griffinsim-v0/src/main/java/org/griffins1884/sim3d/PlanarDriveBackend.java` | DELETE | Legacy planar abstraction is Maple-centric and cannot satisfy 6DOF authoritative physics. |
| `legacy/griffinsim-v0/src/main/java/org/griffins1884/sim3d/SwerveDriveBackend.java` | DELETE | Same issue; backend seam is too narrow and tied to legacy drive simulation ownership. |
| `legacy/griffinsim-v0/src/main/java/org/griffins1884/sim3d/TerrainAwareSwerveSimulation.java` | DELETE | Wall-clock-driven, in-process, cache-coupled wrapper with no deterministic scheduler or IPC boundary. |
| `legacy/griffinsim-v0/src/main/java/org/griffins1884/sim3d/native6dof/NativeSixDofSwerveSimulation.java` | DELETE | Native 6DOF attempt still advances on `System.nanoTime()` and getter access order; cannot be trusted for replay-grade determinism. |
| `legacy/griffinsim-v0/src/main/java/org/griffins1884/sim3d/native6dof/NativeSixDofSwerveConfig.java` | MODIFY | Some rigid-body parameters are reusable, but the class must be re-homed into the future physics core and detached from legacy update semantics. |
| `legacy/griffinsim-v0/src/main/java/org/griffins1884/sim3d/SwerveLoadTransferEstimator.java` | DELETE | Quasi-static load transfer is part of the old 2.5D stack and not a substitute for authoritative contact resolution. |
| `legacy/griffinsim-v0/src/main/java/org/griffins1884/sim3d/SwerveTractionState.java` | MODIFY | The summary concept is reusable for telemetry, but must be recomputed from the new solver, not reused directly. |
| `legacy/griffinsim-v0/src/main/java/org/griffins1884/sim3d/TerrainDriveLaws.java` | DELETE | Terrain authority scaling belongs to the legacy Maple wrapper and bakes convenience heuristics into physics-facing logic. |
| `legacy/griffinsim-v0/src/main/java/org/griffins1884/sim3d/PlanarObstacleContactModel.java` | DELETE | 2D obstacle extrusion is insufficient for the required contact-rich 6DOF engine. |
| `legacy/griffinsim-v0/src/main/java/org/griffins1884/sim3d/PlanarObstacleContactSample.java` | DELETE | Same limitation as above. |
| `legacy/griffinsim-v0/src/main/java/org/griffins1884/sim3d/TerrainContactModel.java` | MODIFY | Terrain/contact sampling remains conceptually useful, but it must be recast as authoritative world geometry and sensor modelling contracts. |
| `legacy/griffinsim-v0/src/main/java/org/griffins1884/sim3d/TerrainModel.java` | MODIFY | World sampling concept is reusable after relocation to physics/sensor modules. |
| `legacy/griffinsim-v0/src/main/java/org/griffins1884/sim3d/integration/*.java` | DELETE | These classes coupled autonomous/visualization concerns directly to legacy in-process simulation objects instead of explicit transport contracts. |
| `legacy/griffinsim-v0/src/main/java/org/griffins1884/sim3d/maple/MapleSwerveDriveBackend.java` | DELETE | Hard dependency on Maple violates the rebuild directive and authoritative split-process physics design. |
| `legacy/griffinsim-v0/src/main/java/org/griffins1884/sim3d/seasonspecific/rebuilt2026/*.java` | MODIFY | Field geometry knowledge is worth preserving, but the old classes are coupled to Maple and legacy package structure. They must be rewritten later under a season/world module. |
| `legacy/griffinsim-v0/src/test/java/org/griffins1884/sim3d/**/*.java` | DELETE | Legacy tests lock in behaviour that the rebuild is intentionally replacing; they are not sufficient to validate determinism, replay, or HALSIM bridge correctness. |
| `legacy/griffinsim-v0/releases/maven-repo/**` | DELETE | Generated release artifacts for the wrong architecture; they must not remain in the active build root. |
| `legacy/griffinsim-v0/releases/vendordeps/**` | DELETE | Same as above; vendordeps published the monolithic legacy library. |
| `legacy/griffinsim-v0/docs/PHASE_PLAN.md` | DELETE | Incremental Maple-first roadmap conflicts with the controlled-reconstruction directive. |
| `legacy/griffinsim-v0/docs/ROBOT_INTEGRATION.md` | DELETE | Integration guidance assumed a direct library dependency instead of a control-host + IPC boundary. |
| `legacy/griffinsim-v0/docs/native-6dof-backend-plan.md` | DELETE | The “add a native backend beside Maple” plan conflicts with the mandatory rebuild away from the legacy architecture. |
| `legacy/griffinsim-v0/docs/VERSIONING.md` | MODIFY | Versioning ideas are still useful, but the active version line now starts at the rebuild monorepo instead of the old library. |

## Immediate audit conclusion

The only safe reuse from legacy GriffinSim is **domain vocabulary and selected immutable data concepts**. Runtime behaviour, update loops, integration seams, and release packaging all had to be removed from the active build.
