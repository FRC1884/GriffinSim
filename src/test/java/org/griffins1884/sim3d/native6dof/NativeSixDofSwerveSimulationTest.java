package org.griffins1884.sim3d.native6dof;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.concurrent.atomic.AtomicReference;
import org.griffins1884.sim3d.ChassisFootprint;
import org.griffins1884.sim3d.ChassisMassProperties;
import org.griffins1884.sim3d.DriveSimulationState;
import org.griffins1884.sim3d.TerrainModel;
import org.griffins1884.sim3d.TerrainSample;
import org.junit.jupiter.api.Test;

class NativeSixDofSwerveSimulationTest {
  private static final ChassisFootprint FOOTPRINT =
      new ChassisFootprint(0.72, 0.72, 0.34, 0.08);
  private static final ChassisMassProperties MASS_PROPERTIES =
      new ChassisMassProperties(61.235, 0.30, 0.72, 0.72, 1.10);
  private static final NativeSixDofSwerveConfig CONFIG =
      NativeSixDofSwerveConfig.rigidSwerve(FOOTPRINT, MASS_PROPERTIES, 0.72, 0.72);

  @Test
  void flatGroundAccelerationProducesForwardMotionAndTraction() {
    AtomicReference<Double> timeSec = new AtomicReference<>(0.0);
    NativeSixDofSwerveSimulation simulation =
        new NativeSixDofSwerveSimulation(CONFIG, flatTerrain(), timeSec::get);
    simulation.resetState(new Pose2d(0.0, 0.0, new Rotation2d()), new ChassisSpeeds(2.0, 0.0, 0.0));

    DriveSimulationState initialState = simulation.getState();
    timeSec.set(0.25);
    DriveSimulationState acceleratedState = simulation.getState();

    assertTrue(acceleratedState.robotRelativeChassisSpeeds().vxMetersPerSecond > 0.0);
    assertTrue(acceleratedState.pose2d().getX() > initialState.pose2d().getX());
    assertTrue(acceleratedState.tractionState().tractionAvailable());
    assertTrue(acceleratedState.tractionState().totalNormalForceNewtons() > 0.0);
  }

  @Test
  void dropCausesSupportLossAndKeepsYawMovingInFreeFlight() {
    AtomicReference<Double> timeSec = new AtomicReference<>(0.0);
    NativeSixDofSwerveSimulation simulation =
        new NativeSixDofSwerveSimulation(CONFIG, dropTerrain(), timeSec::get);
    simulation.resetState(new Pose2d(0.0, 0.0, new Rotation2d()), new ChassisSpeeds(1.8, 0.0, 0.6));

    simulation.getState();
    timeSec.set(0.35);
    DriveSimulationState approachState = simulation.getState();
    timeSec.set(0.80);
    DriveSimulationState airborneState = simulation.getState();

    assertTrue(airborneState.pose3d().getZ() > airborneState.terrainSample().heightMeters());
    assertTrue(airborneState.pose3d().getRotation().getZ() > approachState.pose3d().getRotation().getZ());
    assertTrue(Math.abs(airborneState.imuSample().yawRateRadPerSec()) > 1e-4);
  }

  @Test
  void unevenWheelSupportCreatesRollAndLoadImbalance() {
    AtomicReference<Double> timeSec = new AtomicReference<>(0.0);
    NativeSixDofSwerveSimulation simulation =
        new NativeSixDofSwerveSimulation(CONFIG, sideStepTerrain(), timeSec::get);
    simulation.resetState(new Pose2d(0.0, 0.0, new Rotation2d()), new ChassisSpeeds(0.0, 0.0, 0.0));

    simulation.getState();
    timeSec.set(0.30);
    DriveSimulationState settledState = simulation.getState();

    assertTrue(Math.abs(settledState.pose3d().getRotation().getX()) > 1e-4);
    assertNotEquals(
        settledState.tractionState().frontLeft().normalForceNewtons(),
        settledState.tractionState().frontRight().normalForceNewtons(),
        1e-6);
    assertNotEquals(
        settledState.tractionState().rearLeft().normalForceNewtons(),
        settledState.tractionState().rearRight().normalForceNewtons(),
        1e-6);
  }

  @Test
  void slopeNormalLetsRobotDriftDownhillWithoutDriveCommand() {
    AtomicReference<Double> timeSec = new AtomicReference<>(0.0);
    NativeSixDofSwerveSimulation simulation =
        new NativeSixDofSwerveSimulation(CONFIG, constantSlopeTerrain(), timeSec::get);
    simulation.resetState(new Pose2d(1.0, 0.0, new Rotation2d()), new ChassisSpeeds());

    DriveSimulationState initialState = simulation.getState();
    timeSec.set(0.75);
    DriveSimulationState driftedState = simulation.getState();

    assertTrue(driftedState.pose2d().getX() < initialState.pose2d().getX() - 1e-3);
    assertTrue(driftedState.robotRelativeChassisSpeeds().vxMetersPerSecond < 0.0);
    assertTrue(driftedState.tractionState().totalNormalForceNewtons() > 0.0);
  }

  @Test
  void bodyFrameYawSpinCouplesIntoRollWhenAirborne() {
    AtomicReference<Double> timeSec = new AtomicReference<>(0.0);
    NativeSixDofSwerveSimulation simulation =
        new NativeSixDofSwerveSimulation(CONFIG, flatTerrain(), timeSec::get);
    simulation.resetPose(new Pose2d());
    simulation.setPose3dForTesting(new Pose3d(0.0, 0.0, 1.0, new Rotation3d(0.0, 1.2, 0.0)));
    simulation.setBodyAngularVelocityForTesting(new Translation3d(0.0, 0.0, 1.0));
    simulation.setFieldLinearVelocityForTesting(new Translation3d());

    timeSec.set(0.10);
    DriveSimulationState airborneState = simulation.getState();

    assertTrue(Math.abs(airborneState.pose3d().getRotation().getX()) > 1e-3);
    assertTrue(Math.abs(airborneState.pose3d().getRotation().getY() - 1.2) > 1e-3);
    assertEquals(
        0.0,
        airborneState.tractionState().totalNormalForceNewtons(),
        1e-6,
        "unexpected wheel load: " + airborneState.tractionState().totalNormalForceNewtons());
  }

  private static TerrainModel flatTerrain() {
    return pose ->
        new TerrainSample(
            new Pose3d(pose.getX(), pose.getY(), 0.0, new Rotation3d()),
            0.0,
            0.0,
            0.0);
  }

  private static TerrainModel dropTerrain() {
    return pose -> {
      double height = pose.getX() < 0.9 ? 0.0 : -0.35;
      return new TerrainSample(
          new Pose3d(pose.getX(), pose.getY(), height, new Rotation3d()), 0.0, 0.0, height);
    };
  }

  private static TerrainModel sideStepTerrain() {
    return pose -> {
      double height = pose.getY() >= 0.0 ? 0.16 : 0.0;
      return new TerrainSample(
          new Pose3d(
              pose.getX(),
              pose.getY(),
              height,
              new Rotation3d(height > 0.0 ? 0.12 : 0.0, 0.0, 0.0)),
          height > 0.0 ? 0.12 : 0.0,
          0.0,
          height);
    };
  }

  private static TerrainModel constantSlopeTerrain() {
    return pose -> {
      double slope = 0.22;
      double height = slope * pose.getX();
      double pitch = -Math.atan(slope);
      return new TerrainSample(
          new Pose3d(pose.getX(), pose.getY(), height, new Rotation3d(0.0, pitch, 0.0)),
          0.0,
          pitch,
          height);
    };
  }
}
