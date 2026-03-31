package org.griffins1884.sim3d;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.concurrent.atomic.AtomicReference;
import org.junit.jupiter.api.Test;
import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;

class TerrainAwareSwerveSimulationTest {
  @Test
  void composesPlanarStateTerrainAndImuIntoSingleSnapshot() {
    FakeBackend backend =
        new FakeBackend(
            new Pose2d(2.0, 3.0, Rotation2d.fromDegrees(20.0)),
            new ChassisSpeeds(1.5, -0.2, 0.8),
            new ChassisSpeeds(1.6, 0.1, 0.8));
    AtomicReference<Double> timeSec = new AtomicReference<>(0.0);
    TerrainModel terrainModel =
        pose ->
            new TerrainSample(
                new Pose3d(
                    pose.getX(),
                    pose.getY(),
                    0.42,
                    new Rotation3d(0.11, -0.07, pose.getRotation().getRadians())),
                0.11,
                -0.07,
                0.42);

    TerrainAwareSwerveSimulation simulation =
        new TerrainAwareSwerveSimulation(backend, terrainModel, timeSec::get);

    DriveSimulationState state = simulation.getState();

    assertEquals(backend.pose, state.pose2d());
    assertEquals(backend.robotRelativeSpeeds, state.robotRelativeChassisSpeeds());
    assertEquals(backend.fieldRelativeSpeeds, state.fieldRelativeChassisSpeeds());
    assertEquals(0.42, state.pose3d().getZ(), 1e-9);
    assertEquals(0.8, state.imuSample().yawRateRadPerSec(), 1e-9);
    assertEquals(-0.07, state.terrainSample().pitchRadians(), 1e-9);
  }

  @Test
  void resetStateInvalidatesCachedTerrainAndRecomputesRates() {
    FakeBackend backend =
        new FakeBackend(
            new Pose2d(0.0, 0.0, new Rotation2d()),
            new ChassisSpeeds(),
            new ChassisSpeeds());
    AtomicReference<Double> timeSec = new AtomicReference<>(0.0);
    TerrainModel terrainModel =
        pose ->
            new TerrainSample(
                new Pose3d(
                    pose.getX(),
                    pose.getY(),
                    pose.getX(),
                    new Rotation3d(pose.getX(), pose.getY(), pose.getRotation().getRadians())),
                pose.getX(),
                pose.getY(),
                pose.getX());

    TerrainAwareSwerveSimulation simulation =
        new TerrainAwareSwerveSimulation(backend, terrainModel, timeSec::get);

    TerrainSample firstSample = simulation.getTerrainSample();
    assertSame(firstSample, simulation.getTerrainSample());

    backend.pose = new Pose2d(1.0, 0.5, Rotation2d.fromDegrees(15.0));
    timeSec.set(0.2);
    TerrainSample movedSample = simulation.getTerrainSample();

    assertEquals(5.0, simulation.getRollRateRadPerSec(), 1e-9);
    assertEquals(2.5, simulation.getPitchRateRadPerSec(), 1e-9);
    assertEquals(1.0, movedSample.heightMeters(), 1e-9);

    simulation.resetState(
        new Pose2d(0.25, 0.0, new Rotation2d()), new ChassisSpeeds(2.0, 0.0, -1.0));
    timeSec.set(0.3);

    TerrainSample resetSample = simulation.getTerrainSample();
    assertEquals(0.0, simulation.getRollRateRadPerSec(), 1e-9);
    assertEquals(0.0, simulation.getPitchRateRadPerSec(), 1e-9);
    assertEquals(0.25, resetSample.heightMeters(), 1e-9);
    assertEquals(-1.0, simulation.getImuSample().yawRateRadPerSec(), 1e-9);
  }

  private static final class FakeBackend implements SwerveDriveBackend {
    private Pose2d pose;
    private ChassisSpeeds robotRelativeSpeeds;
    private ChassisSpeeds fieldRelativeSpeeds;

    private FakeBackend(
        Pose2d pose, ChassisSpeeds robotRelativeSpeeds, ChassisSpeeds fieldRelativeSpeeds) {
      this.pose = pose;
      this.robotRelativeSpeeds = robotRelativeSpeeds;
      this.fieldRelativeSpeeds = fieldRelativeSpeeds;
    }

    @Override
    public Pose2d getPose2d() {
      return pose;
    }

    @Override
    public ChassisSpeeds getRobotRelativeChassisSpeeds() {
      return robotRelativeSpeeds;
    }

    @Override
    public ChassisSpeeds getFieldRelativeChassisSpeeds() {
      return fieldRelativeSpeeds;
    }

    @Override
    public GyroSimulation getGyroSimulation() {
      return null;
    }

    @Override
    public void setPose(Pose2d pose) {
      this.pose = pose;
    }

    @Override
    public void setRobotRelativeChassisSpeeds(ChassisSpeeds robotRelativeSpeeds) {
      this.robotRelativeSpeeds = robotRelativeSpeeds;
      this.fieldRelativeSpeeds = robotRelativeSpeeds;
    }

    @Override
    public SwerveModuleSimulation[] getModuleSimulations() {
      return new SwerveModuleSimulation[0];
    }
  }
}
