package org.griffins1884.sim3d;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
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
  private final TerrainContactModel terrainContactModel;
  private final ChassisFootprint chassisFootprint;
  private final ChassisMassProperties chassisMassProperties;
  private final DoubleSupplier clockSecondsSupplier;

  private double lastSampleTimestampSec = Double.NaN;
  private TerrainSample cachedSample = null;
  private TerrainContactSample cachedTerrainContactSample = null;
  private ChassisState3d cachedChassisState = null;
  private SwerveTractionState cachedTractionState = null;
  private double rollRateRadPerSec = 0.0;
  private double pitchRateRadPerSec = 0.0;

  public TerrainAwareSwerveSimulation(
      SwerveDriveSimulation mapleSimulation, TerrainModel terrainModel) {
    this(new MapleSwerveDriveBackend(mapleSimulation), terrainModel);
  }

  public TerrainAwareSwerveSimulation(SwerveDriveBackend driveBackend, TerrainModel terrainModel) {
    this(driveBackend, terrainModel, null, null, null, () -> System.nanoTime() * 1.0e-9);
  }

  public TerrainAwareSwerveSimulation(
      SwerveDriveBackend driveBackend,
      TerrainContactModel terrainContactModel,
      ChassisFootprint chassisFootprint,
      ChassisMassProperties chassisMassProperties) {
    this(
        driveBackend,
        terrainContactModel,
        terrainContactModel,
        chassisFootprint,
        chassisMassProperties,
        () -> System.nanoTime() * 1.0e-9);
  }

  public TerrainAwareSwerveSimulation(
      SwerveDriveBackend driveBackend,
      TerrainModel terrainModel,
      DoubleSupplier clockSecondsSupplier) {
    this(driveBackend, terrainModel, null, null, null, clockSecondsSupplier);
  }

  public TerrainAwareSwerveSimulation(
      SwerveDriveBackend driveBackend,
      TerrainContactModel terrainContactModel,
      ChassisFootprint chassisFootprint,
      ChassisMassProperties chassisMassProperties,
      DoubleSupplier clockSecondsSupplier) {
    this(
        driveBackend,
        terrainContactModel,
        terrainContactModel,
        chassisFootprint,
        chassisMassProperties,
        clockSecondsSupplier);
  }

  private TerrainAwareSwerveSimulation(
      SwerveDriveBackend driveBackend,
      TerrainModel terrainModel,
      TerrainContactModel terrainContactModel,
      ChassisFootprint chassisFootprint,
      ChassisMassProperties chassisMassProperties,
      DoubleSupplier clockSecondsSupplier) {
    this.driveBackend = Objects.requireNonNull(driveBackend);
    this.terrainModel = Objects.requireNonNull(terrainModel);
    this.terrainContactModel = terrainContactModel;
    this.chassisFootprint = chassisFootprint;
    this.chassisMassProperties = chassisMassProperties;
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
    return getChassisState3d().pose();
  }

  @Override
  public synchronized ChassisState3d getChassisState3d() {
    return sampleChassisState();
  }

  @Override
  public synchronized TerrainContactSample getTerrainContactSample() {
    sampleChassisState();
    return cachedTerrainContactSample;
  }

  @Override
  public synchronized SwerveTractionState getTractionState() {
    sampleChassisState();
    return cachedTractionState;
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
    ChassisState3d chassisState3d = getChassisState3d();
    return new SimImuSample(chassisState3d.pose().getRotation(), chassisState3d.angularVelocityRadPerSec());
  }

  @Override
  public synchronized DriveSimulationState getState() {
    ChassisState3d chassisState3d = getChassisState3d();
    TerrainSample terrainSample = cachedSample;
    return new DriveSimulationState(
        getPose2d(),
        chassisState3d.pose(),
        chassisState3d,
        cachedTerrainContactSample,
        cachedTractionState,
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
    cachedTerrainContactSample = null;
    cachedChassisState = null;
    cachedTractionState = null;
    rollRateRadPerSec = 0.0;
    pitchRateRadPerSec = 0.0;
  }

  public synchronized TerrainSample getTerrainSample() {
    sampleChassisState();
    return cachedSample;
  }

  private ChassisState3d sampleChassisState() {
    double now = clockSecondsSupplier.getAsDouble();
    if (cachedChassisState != null && Math.abs(now - lastSampleTimestampSec) < 1e-6) {
      return cachedChassisState;
    }

    TerrainContactSample newTerrainContactSample = null;
    TerrainSample newSample;
    if (terrainContactModel != null && chassisFootprint != null) {
      newTerrainContactSample = terrainContactModel.sampleContact(getPose2d(), chassisFootprint);
      newSample = newTerrainContactSample.terrainSample();
    } else {
      newSample = terrainModel.sample(getPose2d());
    }
    ChassisSpeeds fieldRelativeChassisSpeeds = getFieldRelativeChassisSpeeds();
    Translation3d previousFieldVelocity =
        cachedChassisState == null
            ? new Translation3d()
            : cachedChassisState.fieldRelativeLinearVelocityMetersPerSec();
    Translation3d fieldVelocity =
        new Translation3d(
            fieldRelativeChassisSpeeds.vxMetersPerSecond,
            fieldRelativeChassisSpeeds.vyMetersPerSecond,
            0.0);
    Translation3d fieldAcceleration = new Translation3d();
    if (cachedSample != null && Double.isFinite(lastSampleTimestampSec)) {
      double dt = now - lastSampleTimestampSec;
      if (dt > 1e-5) {
        rollRateRadPerSec = (newSample.rollRadians() - cachedSample.rollRadians()) / dt;
        pitchRateRadPerSec = (newSample.pitchRadians() - cachedSample.pitchRadians()) / dt;
        fieldVelocity =
            new Translation3d(
                fieldRelativeChassisSpeeds.vxMetersPerSecond,
                fieldRelativeChassisSpeeds.vyMetersPerSecond,
                (newSample.heightMeters() - cachedSample.heightMeters()) / dt);
        fieldAcceleration =
            new Translation3d(
                (fieldVelocity.getX() - previousFieldVelocity.getX()) / dt,
                (fieldVelocity.getY() - previousFieldVelocity.getY()) / dt,
                (fieldVelocity.getZ() - previousFieldVelocity.getZ()) / dt);
      }
    }

    cachedSample = newSample;
    cachedTerrainContactSample = newTerrainContactSample;
    cachedChassisState =
        new ChassisState3d(
            newSample.pose3d(),
            fieldVelocity,
            fieldAcceleration,
            new AngularVelocity3d(
                rollRateRadPerSec,
                pitchRateRadPerSec,
                getRobotRelativeChassisSpeeds().omegaRadiansPerSecond));
    cachedTractionState =
        cachedTerrainContactSample != null && chassisMassProperties != null
            ? SwerveLoadTransferEstimator.estimate(
                cachedChassisState, cachedTerrainContactSample, chassisMassProperties)
            : null;
    lastSampleTimestampSec = now;
    return cachedChassisState;
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
