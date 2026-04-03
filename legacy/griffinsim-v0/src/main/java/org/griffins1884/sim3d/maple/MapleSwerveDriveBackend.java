package org.griffins1884.sim3d.maple;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.griffins1884.sim3d.SwerveDriveBackend;
import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;

/** Maple-backed implementation of the GriffinSim planar drivetrain backend contract. */
public final class MapleSwerveDriveBackend implements SwerveDriveBackend {
  private final SwerveDriveSimulation mapleSimulation;

  public MapleSwerveDriveBackend(SwerveDriveSimulation mapleSimulation) {
    this.mapleSimulation = mapleSimulation;
  }

  public SwerveDriveSimulation mapleSimulation() {
    return mapleSimulation;
  }

  @Override
  public Pose2d getPose2d() {
    return mapleSimulation.getSimulatedDriveTrainPose();
  }

  @Override
  public ChassisSpeeds getRobotRelativeChassisSpeeds() {
    return mapleSimulation.getDriveTrainSimulatedChassisSpeedsRobotRelative();
  }

  @Override
  public ChassisSpeeds getFieldRelativeChassisSpeeds() {
    return mapleSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative();
  }

  @Override
  public GyroSimulation getGyroSimulation() {
    return mapleSimulation.getGyroSimulation();
  }

  @Override
  public void setPose(Pose2d pose) {
    mapleSimulation.setSimulationWorldPose(pose);
  }

  @Override
  public void setRobotRelativeChassisSpeeds(ChassisSpeeds robotRelativeSpeeds) {
    mapleSimulation.setRobotSpeeds(robotRelativeSpeeds);
  }

  @Override
  public SwerveModuleSimulation[] getModuleSimulations() {
    return mapleSimulation.getModules();
  }
}
