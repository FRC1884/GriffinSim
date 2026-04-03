package org.griffins1884.sim3d;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.ironmaple.simulation.drivesims.GyroSimulation;

/**
 * Abstraction for the planar drivetrain backend currently driving XY and yaw motion.
 *
 * <p>Phase 1 uses this boundary to decouple robot integration code from Maple-specific classes. A
 * later GriffinSim-native backend can implement the same contract while carrying richer 6DOF state.
 */
public interface PlanarDriveBackend {
  Pose2d getPose2d();

  ChassisSpeeds getRobotRelativeChassisSpeeds();

  ChassisSpeeds getFieldRelativeChassisSpeeds();

  GyroSimulation getGyroSimulation();

  void setPose(Pose2d pose);

  void setRobotRelativeChassisSpeeds(ChassisSpeeds robotRelativeSpeeds);
}
