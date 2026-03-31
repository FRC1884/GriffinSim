package org.griffins1884.sim3d;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** Snapshot of the simulation state exposed to robot code and visualization. */
public record DriveSimulationState(
    Pose2d pose2d,
    Pose3d pose3d,
    ChassisState3d chassisState3d,
    ChassisSpeeds robotRelativeChassisSpeeds,
    ChassisSpeeds fieldRelativeChassisSpeeds,
    SimImuSample imuSample,
    TerrainSample terrainSample) {}
