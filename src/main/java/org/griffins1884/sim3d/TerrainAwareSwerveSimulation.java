package org.griffins1884.sim3d;

import edu.wpi.first.math.geometry.Pose2d;
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
public final class TerrainAwareSwerveSimulation {
  private final SwerveDriveSimulation mapleSimulation;
  private final TerrainModel terrainModel;

  private double lastSampleTimestampSec = Double.NaN;
  private TerrainSample cachedSample = null;
  private double rollRateRadPerSec = 0.0;
  private double pitchRateRadPerSec = 0.0;

  public TerrainAwareSwerveSimulation(
      SwerveDriveSimulation mapleSimulation, TerrainModel terrainModel) {
    this.mapleSimulation = mapleSimulation;
    this.terrainModel = terrainModel;
  }

  public SwerveDriveSimulation mapleSimulation() {
    return mapleSimulation;
  }

  public GyroSimulation getGyroSimulation() {
    return mapleSimulation.getGyroSimulation();
  }

  public SwerveModuleSimulation[] getModules() {
    return mapleSimulation.getModules();
  }

  public Pose2d getSimulatedDriveTrainPose() {
    return mapleSimulation.getSimulatedDriveTrainPose();
  }

  public void setSimulationWorldPose(Pose2d pose) {
    mapleSimulation.setSimulationWorldPose(pose);
    lastSampleTimestampSec = Double.NaN;
    cachedSample = null;
    rollRateRadPerSec = 0.0;
    pitchRateRadPerSec = 0.0;
  }

  public synchronized TerrainSample getTerrainSample() {
    double now = System.nanoTime() * 1.0e-9;
    if (cachedSample != null && Math.abs(now - lastSampleTimestampSec) < 1e-6) {
      return cachedSample;
    }

    TerrainSample newSample = terrainModel.sample(getSimulatedDriveTrainPose());
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
