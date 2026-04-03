package org.griffins1884.sim3d;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

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
    assertEquals(0.0, state.chassisState3d().angularVelocityRadPerSec().rollRateRadPerSec(), 1e-9);
    assertEquals(0.0, state.chassisState3d().angularVelocityRadPerSec().pitchRateRadPerSec(), 1e-9);
    assertEquals(0.8, state.chassisState3d().angularVelocityRadPerSec().yawRateRadPerSec(), 1e-9);
    assertEquals(1.6, state.chassisState3d().fieldRelativeLinearVelocityMetersPerSec().getX(), 1e-9);
    assertEquals(0.0, state.chassisState3d().fieldRelativeLinearVelocityMetersPerSec().getZ(), 1e-9);
    assertEquals(0.42, state.pose3d().getZ(), 1e-9);
    assertEquals(0.8, state.imuSample().yawRateRadPerSec(), 1e-9);
    assertEquals(-0.07, state.terrainSample().pitchRadians(), 1e-9);
    assertNull(state.terrainContactSample());
    assertNull(state.tractionState());
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
    ChassisState3d movedState = simulation.getChassisState3d();

    assertEquals(5.0, simulation.getRollRateRadPerSec(), 0.05);
    assertEquals(2.5, simulation.getPitchRateRadPerSec(), 0.05);
    assertEquals(1.0, movedSample.heightMeters(), 1e-9);
    assertEquals(5.0, movedState.fieldRelativeLinearVelocityMetersPerSec().getZ(), 1e-9);
    assertEquals(25.0, movedState.fieldRelativeLinearAccelerationMetersPerSecSq().getZ(), 1e-9);

    simulation.resetState(
        new Pose2d(0.25, 0.0, new Rotation2d()), new ChassisSpeeds(2.0, 0.0, -1.0));
    timeSec.set(0.3);

    TerrainSample resetSample = simulation.getTerrainSample();
    ChassisState3d resetState = simulation.getChassisState3d();
    assertEquals(0.0, simulation.getRollRateRadPerSec(), 1e-9);
    assertEquals(0.0, simulation.getPitchRateRadPerSec(), 1e-9);
    assertEquals(0.25, resetSample.heightMeters(), 1e-9);
    assertEquals(-1.0, simulation.getImuSample().yawRateRadPerSec(), 1e-9);
    assertEquals(0.0, resetState.fieldRelativeLinearVelocityMetersPerSec().getZ(), 1e-9);
    assertEquals(0.0, resetState.fieldRelativeLinearAccelerationMetersPerSecSq().getZ(), 1e-9);
  }

  @Test
  void reusesCachedTerrainStateWhenBackendStateHasNotAdvanced() {
    FakeBackend backend =
        new FakeBackend(
            new Pose2d(0.0, 0.0, new Rotation2d()),
            new ChassisSpeeds(1.0, 0.0, 0.2),
            new ChassisSpeeds(1.0, 0.0, 0.2));
    AtomicReference<Double> timeSec = new AtomicReference<>(0.0);
    int[] terrainSamples = new int[1];
    TerrainModel terrainModel =
        pose -> {
          terrainSamples[0]++;
          return new TerrainSample(
              new Pose3d(
                  pose.getX(),
                  pose.getY(),
                  0.25,
                  new Rotation3d(0.01, 0.02, pose.getRotation().getRadians())),
              0.01,
              0.02,
              0.25);
        };

    TerrainAwareSwerveSimulation simulation =
        new TerrainAwareSwerveSimulation(backend, terrainModel, timeSec::get);

    DriveSimulationState firstState = simulation.getState();
    timeSec.set(0.5);
    DriveSimulationState secondState = simulation.getState();

    assertEquals(firstState.pose2d(), secondState.pose2d());
    assertEquals(firstState.pose3d(), secondState.pose3d());
    assertEquals(
        firstState.chassisState3d().fieldRelativeLinearVelocityMetersPerSec(),
        secondState.chassisState3d().fieldRelativeLinearVelocityMetersPerSec());
    assertEquals(1, terrainSamples[0]);
  }

  @Test
  void exposesContactAndTractionStateWhenConfigured() {
    FakeBackend backend =
        new FakeBackend(
            new Pose2d(4.6, 5.55, new Rotation2d()),
            new ChassisSpeeds(0.5, 0.0, 0.2),
            new ChassisSpeeds(0.5, 0.0, 0.2));
    AtomicReference<Double> timeSec = new AtomicReference<>(0.0);
    FakeTerrainContactModel terrainContactModel = new FakeTerrainContactModel();

    TerrainAwareSwerveSimulation simulation =
        new TerrainAwareSwerveSimulation(
            backend,
            terrainContactModel,
            new ChassisFootprint(0.9, 0.9, 0.5, 0.08),
            new ChassisMassProperties(48.0, 0.30, 0.70, 0.60, 1.1),
            timeSec::get);

    DriveSimulationState state = simulation.getState();

    assertNotNull(state.terrainContactSample());
    assertNotNull(state.tractionState());
    assertEquals(TerrainFeature.BLUE_LEFT_BUMP, state.terrainContactSample().feature());
    assertEquals(
        state.tractionState().totalNormalForceNewtons(),
        state.tractionState().frontLeft().normalForceNewtons()
            + state.tractionState().frontRight().normalForceNewtons()
            + state.tractionState().rearLeft().normalForceNewtons()
            + state.tractionState().rearRight().normalForceNewtons(),
        1e-6);
  }

  @Test
  void goesUnsupportedAfterDrivingOffABumpCrest() {
    FakeBackend backend =
        new FakeBackend(
            new Pose2d(0.0, 0.0, new Rotation2d()),
            new ChassisSpeeds(2.0, 0.0, 0.0),
            new ChassisSpeeds(2.0, 0.0, 0.0));
    AtomicReference<Double> timeSec = new AtomicReference<>(0.0);
    TerrainContactModel contactModel =
        new TerrainContactModel() {
          @Override
          public TerrainSample sample(Pose2d robotPose) {
            double height = robotPose.getX() < 0.6 ? robotPose.getX() : 0.0;
            return new TerrainSample(
                new Pose3d(
                    robotPose.getX(),
                    robotPose.getY(),
                    height,
                    new Rotation3d(0.0, 0.08, robotPose.getRotation().getRadians())),
                0.0,
                0.08,
                height);
          }

          @Override
          public TerrainContactSample sampleContact(
              Pose2d robotPose, ChassisFootprint chassisFootprint) {
            return new TerrainContactSample(
                sample(robotPose),
                TerrainFeature.BLUE_LEFT_BUMP,
                Double.POSITIVE_INFINITY,
                chassisFootprint.groundClearanceMeters(),
                Double.POSITIVE_INFINITY,
                true,
                true);
          }
        };

    TerrainAwareSwerveSimulation simulation =
        new TerrainAwareSwerveSimulation(
            backend,
            contactModel,
            new ChassisFootprint(0.4, 0.4, 0.45, 0.08),
            new ChassisMassProperties(61.235, 0.30, 0.40, 0.40, 1.1),
            timeSec::get);

    simulation.getState();
    backend.pose = new Pose2d(0.5, 0.0, new Rotation2d());
    timeSec.set(0.2);
    simulation.getState();

    backend.pose = new Pose2d(0.9, 0.0, new Rotation2d());
    timeSec.set(0.4);
    DriveSimulationState airborneState = simulation.getState();

    assertFalse(simulation.isBodySupported());
    assertFalse(airborneState.tractionState().tractionAvailable());
    assertEquals(0.0, airborneState.terrainSample().heightMeters(), 1e-9);
    assertFalse(airborneState.pose3d().getZ() <= airborneState.terrainSample().heightMeters());
  }

  @Test
  void bodyHeightUsesFootprintSupportPlaneWhileBridgingAnEdge() {
    FakeBackend backend =
        new FakeBackend(
            new Pose2d(0.65, 0.0, new Rotation2d()),
            new ChassisSpeeds(),
            new ChassisSpeeds());
    AtomicReference<Double> timeSec = new AtomicReference<>(0.0);
    TerrainContactModel contactModel =
        new TerrainContactModel() {
          @Override
          public TerrainSample sample(Pose2d robotPose) {
            double height = robotPose.getX() < 0.6 ? 0.3 : 0.0;
            return new TerrainSample(
                new Pose3d(
                    robotPose.getX(),
                    robotPose.getY(),
                    height,
                    new Rotation3d(0.0, 0.0, robotPose.getRotation().getRadians())),
                0.0,
                0.0,
                height);
          }

          @Override
          public TerrainContactSample sampleContact(
              Pose2d robotPose, ChassisFootprint chassisFootprint) {
            return new TerrainContactSample(
                sample(robotPose),
                TerrainFeature.BLUE_LEFT_BUMP,
                Double.POSITIVE_INFINITY,
                chassisFootprint.groundClearanceMeters(),
                Double.POSITIVE_INFINITY,
                true,
                true);
          }
        };

    TerrainAwareSwerveSimulation simulation =
        new TerrainAwareSwerveSimulation(
            backend,
            contactModel,
            new ChassisFootprint(0.4, 0.4, 0.45, 0.08),
            new ChassisMassProperties(61.235, 0.30, 0.40, 0.40, 1.1),
            timeSec::get);

    DriveSimulationState state = simulation.getState();

    assertTrue(simulation.isBodySupported());
    assertEquals(0.0, state.terrainSample().heightMeters(), 1e-9);
    assertTrue(state.pose3d().getZ() > state.terrainSample().heightMeters());
  }

  @Test
  void landsAndRecoversTractionAfterAirborneBumpExit() {
    FakeBackend backend =
        new FakeBackend(
            new Pose2d(0.0, 0.0, new Rotation2d()),
            new ChassisSpeeds(2.0, 0.0, 0.0),
            new ChassisSpeeds(2.0, 0.0, 0.0));
    AtomicReference<Double> timeSec = new AtomicReference<>(0.0);
    TerrainContactModel contactModel =
        new TerrainContactModel() {
          @Override
          public TerrainSample sample(Pose2d robotPose) {
            double height = robotPose.getX() < 0.6 ? robotPose.getX() : 0.0;
            return new TerrainSample(
                new Pose3d(
                    robotPose.getX(),
                    robotPose.getY(),
                    height,
                    new Rotation3d(0.0, 0.08, robotPose.getRotation().getRadians())),
                0.0,
                0.08,
                height);
          }

          @Override
          public TerrainContactSample sampleContact(
              Pose2d robotPose, ChassisFootprint chassisFootprint) {
            return new TerrainContactSample(
                sample(robotPose),
                TerrainFeature.BLUE_LEFT_BUMP,
                Double.POSITIVE_INFINITY,
                chassisFootprint.groundClearanceMeters(),
                Double.POSITIVE_INFINITY,
                true,
                true);
          }
        };

    TerrainAwareSwerveSimulation simulation =
        new TerrainAwareSwerveSimulation(
            backend,
            contactModel,
            new ChassisFootprint(0.4, 0.4, 0.45, 0.08),
            new ChassisMassProperties(61.235, 0.30, 0.40, 0.40, 1.1),
            timeSec::get);

    simulation.getState();
    backend.pose = new Pose2d(0.5, 0.0, new Rotation2d());
    timeSec.set(0.2);
    simulation.getState();

    backend.pose = new Pose2d(0.9, 0.0, new Rotation2d());
    timeSec.set(0.4);
    simulation.getState();

    backend.pose = new Pose2d(1.2, 0.0, new Rotation2d());
    timeSec.set(1.2);
    DriveSimulationState landedState = simulation.getState();

    assertTrue(simulation.isBodySupported());
    assertTrue(landedState.tractionState().tractionAvailable());
    assertEquals(landedState.terrainSample().heightMeters(), landedState.pose3d().getZ(), 1e-9);
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

  private static final class FakeTerrainContactModel implements TerrainContactModel {
    @Override
    public TerrainSample sample(Pose2d robotPose) {
      return new TerrainSample(
          new Pose3d(
              robotPose.getX(),
              robotPose.getY(),
              0.1,
              new Rotation3d(0.04, 0.06, robotPose.getRotation().getRadians())),
          0.04,
          0.06,
          0.1);
    }

    @Override
    public TerrainContactSample sampleContact(Pose2d robotPose, ChassisFootprint chassisFootprint) {
      return new TerrainContactSample(
          sample(robotPose),
          TerrainFeature.BLUE_LEFT_BUMP,
          Double.POSITIVE_INFINITY,
          chassisFootprint.groundClearanceMeters() - 0.1,
          Double.POSITIVE_INFINITY,
          true,
          true);
    }
  }
}
