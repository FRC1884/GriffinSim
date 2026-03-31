package org.griffins1884.sim3d;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.Objects;
import java.util.function.DoubleSupplier;
import org.griffins1884.sim3d.maple.MapleSwerveDriveBackend;
import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;

/**
 * Shared terrain-aware wrapper around Maple's swerve simulator.
 *
 * <p>Maple remains the source of truth for planar drivetrain motion. This wrapper adds shared roll,
 * pitch, and height state derived from a caller-provided terrain model so gyro simulation,
 * autonomous, and 3D visualization use the same terrain sample.
 */
public final class TerrainAwareSwerveSimulation implements DriveSimulationAdapter {
  private final SwerveDriveBackend driveBackend;
  private final TerrainModel terrainModel;
  private final DoubleSupplier clockSecondsSupplier;

  private double lastSampleTimestampSec = Double.NaN;
  private TerrainSample cachedSample = null;
  private double rollRateRadPerSec = 0.0;
  private double pitchRateRadPerSec = 0.0;

  public TerrainAwareSwerveSimulation(
      SwerveDriveSimulation mapleSimulation, TerrainModel terrainModel) {
    this(new MapleSwerveDriveBackend(mapleSimulation), terrainModel);
  }

  public TerrainAwareSwerveSimulation(SwerveDriveBackend driveBackend, TerrainModel terrainModel) {
    this(driveBackend, terrainModel, () -> System.nanoTime() * 1.0e-9);
  }

  public TerrainAwareSwerveSimulation(
      SwerveDriveBackend driveBackend,
      TerrainModel terrainModel,
      DoubleSupplier clockSecondsSupplier) {
    this.driveBackend = Objects.requireNonNull(driveBackend);
    this.terrainModel = Objects.requireNonNull(terrainModel);
    this.clockSecondsSupplier = Objects.requireNonNull(clockSecondsSupplier);
  }

  public SwerveDriveSimulation mapleSimulation() {
    if (driveBackend instanceof MapleSwerveDriveBackend mapleBackend) {
      return mapleBackend.mapleSimulation();
    }
    throw new IllegalStateException("Current backend is not backed by Maple SwerveDriveSimulation");
  }

  public GyroSimulation getGyroSimulation() {
    return driveBackend.getGyroSimulation();
  }

  public SwerveModuleSimulation[] getModules() {
    return driveBackend.getModuleSimulations();
  }

  public Pose2d getSimulatedDriveTrainPose() {
    return getPose2d();
  }

  public void setSimulationWorldPose(Pose2d pose) {
    resetPose(pose);
  }

  @Override
  public Pose2d getPose2d() {
    return driveBackend.getPose2d();
  }

  @Override
  public edu.wpi.first.math.geometry.Pose3d getPose3d() {
    return getTerrainSample().pose3d();
  }

  @Override
  public ChassisSpeeds getRobotRelativeChassisSpeeds() {
    return driveBackend.getRobotRelativeChassisSpeeds();
  }

  @Override
  public ChassisSpeeds getFieldRelativeChassisSpeeds() {
    return driveBackend.getFieldRelativeChassisSpeeds();
  }

  @Override
  public synchronized SimImuSample getImuSample() {
    TerrainSample terrainSample = getTerrainSample();
    ChassisSpeeds robotRelativeSpeeds = getRobotRelativeChassisSpeeds();
    return new SimImuSample(
        terrainSample.pose3d().getRotation(),
        robotRelativeSpeeds.omegaRadiansPerSecond,
        pitchRateRadPerSec,
        rollRateRadPerSec);
  }

  @Override
  public synchronized DriveSimulationState getState() {
    TerrainSample terrainSample = getTerrainSample();
    return new DriveSimulationState(
        getPose2d(),
        terrainSample.pose3d(),
        getRobotRelativeChassisSpeeds(),
        getFieldRelativeChassisSpeeds(),
        getImuSample(),
        terrainSample);
  }

  @Override
  public void resetPose(Pose2d pose) {
    driveBackend.setPose(pose);
    invalidateCachedTerrainState();
  }

  @Override
  public void resetState(Pose2d pose, ChassisSpeeds robotRelativeSpeeds) {
    driveBackend.setPose(pose);
    driveBackend.setRobotRelativeChassisSpeeds(robotRelativeSpeeds);
    invalidateCachedTerrainState();
  }

  private void invalidateCachedTerrainState() {
    lastSampleTimestampSec = Double.NaN;
    cachedSample = null;
    rollRateRadPerSec = 0.0;
    pitchRateRadPerSec = 0.0;
  }

  public synchronized TerrainSample getTerrainSample() {
    double now = clockSecondsSupplier.getAsDouble();
    if (cachedSample != null && Math.abs(now - lastSampleTimestampSec) < 1e-6) {
      return cachedSample;
    }

    TerrainSample newSample = terrainModel.sample(getPose2d());
    if (cachedSample != null && Double.isFinite(lastSampleTimestampSec)) {
      double dt = now - lastSampleTimestampSec;
      if (dt > 1e-5) {
        rollRateRadPerSec = (newSample.rollRadians() - cachedSample.rollRadians()) / dt;
        pitchRateRadPerSec = (newSample.pitchRadians() - cachedSample.pitchRadians()) / dt;
      }
    }

    cachedSample = newSample;
    lastSampleTimestampSec = now;
    return cachedSample;
  }

  public synchronized double getRollRateRadPerSec() {
    getTerrainSample();
    return rollRateRadPerSec;
  }

  public synchronized double getPitchRateRadPerSec() {
    getTerrainSample();
    return pitchRateRadPerSec;
  }
}
