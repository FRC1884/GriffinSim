package org.griffins1884.sim3d.integration;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.griffins1884.sim3d.ChassisState3d;
import org.griffins1884.sim3d.SwerveTractionState;
import org.griffins1884.sim3d.TerrainContactSample;

/**
 * Generic simulation hooks for holonomic autonomous libraries.
 *
 * <p>These hooks map directly onto the suppliers and reset callbacks typically required by
 * pose-based path followers such as PathPlanner and Choreo integrations.
 */
public record HolonomicAutoHooks(
    Supplier<Pose2d> poseSupplier,
    Supplier<ChassisSpeeds> robotRelativeChassisSpeedsSupplier,
    Consumer<Pose2d> poseResetter,
    Supplier<ChassisState3d> chassisState3dSupplier,
    Supplier<TerrainContactSample> terrainContactSupplier,
    Supplier<SwerveTractionState> tractionStateSupplier) {}
