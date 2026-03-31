package org.griffins1884.sim3d;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Stable robot-facing view of a drivetrain simulation.
 *
 * <p>Implementations may be backed by Maple, a GriffinSim-native 6DOF solver, or another simulation
 * engine. Consumers should use this interface rather than depending directly on the backend.
 */
public interface DriveSimulationAdapter {
  Pose2d getPose2d();

  Pose3d getPose3d();

  ChassisState3d getChassisState3d();

  TerrainContactSample getTerrainContactSample();

  SwerveTractionState getTractionState();

  ChassisSpeeds getRobotRelativeChassisSpeeds();

  ChassisSpeeds getFieldRelativeChassisSpeeds();

  SimImuSample getImuSample();

  TerrainSample getTerrainSample();

  DriveSimulationState getState();

  void resetPose(Pose2d pose);

  void resetState(Pose2d pose, ChassisSpeeds robotRelativeSpeeds);
}
