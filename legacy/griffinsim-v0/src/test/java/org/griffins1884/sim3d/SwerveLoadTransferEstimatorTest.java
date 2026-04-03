package org.griffins1884.sim3d;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import org.junit.jupiter.api.Test;

class SwerveLoadTransferEstimatorTest {
  @Test
  void keepsFlatRobotLoadsBalanced() {
    ChassisState3d chassisState3d =
        new ChassisState3d(
            new Pose3d(0.0, 0.0, 0.0, new Rotation3d()),
            new Translation3d(),
            new Translation3d(),
            new AngularVelocity3d(0.0, 0.0, 0.0));
    TerrainContactSample terrainContactSample =
        new TerrainContactSample(
            new TerrainSample(chassisState3d.pose(), 0.0, 0.0, 0.0),
            TerrainFeature.FLAT,
            Double.POSITIVE_INFINITY,
            0.05,
            Double.POSITIVE_INFINITY,
            true,
            true);
    ChassisMassProperties chassisMassProperties =
        new ChassisMassProperties(50.0, 0.30, 0.70, 0.60, 1.10);

    SwerveTractionState tractionState =
        SwerveLoadTransferEstimator.estimate(
            chassisState3d, terrainContactSample, chassisMassProperties);

    double expectedWheelLoad = tractionState.totalNormalForceNewtons() / 4.0;
    assertEquals(expectedWheelLoad, tractionState.frontLeft().normalForceNewtons(), 1e-6);
    assertEquals(expectedWheelLoad, tractionState.frontRight().normalForceNewtons(), 1e-6);
    assertEquals(expectedWheelLoad, tractionState.rearLeft().normalForceNewtons(), 1e-6);
    assertEquals(expectedWheelLoad, tractionState.rearRight().normalForceNewtons(), 1e-6);
    assertTrue(tractionState.tractionAvailable());
  }

  @Test
  void shiftsLoadRearwardAndRightwardOnSlopeAndAcceleration() {
    ChassisState3d chassisState3d =
        new ChassisState3d(
            new Pose3d(0.0, 0.0, 0.0, new Rotation3d(0.10, 0.12, 0.0)),
            new Translation3d(),
            new Translation3d(1.5, 0.8, 0.0),
            new AngularVelocity3d(0.0, 0.0, 0.0));
    TerrainContactSample terrainContactSample =
        new TerrainContactSample(
            new TerrainSample(chassisState3d.pose(), 0.10, 0.12, 0.0),
            TerrainFeature.BLUE_LEFT_BUMP,
            Double.POSITIVE_INFINITY,
            -0.02,
            Double.POSITIVE_INFINITY,
            true,
            true);
    ChassisMassProperties chassisMassProperties =
        new ChassisMassProperties(52.0, 0.33, 0.72, 0.62, 1.15);

    SwerveTractionState tractionState =
        SwerveLoadTransferEstimator.estimate(
            chassisState3d, terrainContactSample, chassisMassProperties);

    assertTrue(
        tractionState.rearLeft().normalForceNewtons()
            > tractionState.frontLeft().normalForceNewtons());
    assertTrue(
        tractionState.rearRight().normalForceNewtons()
            > tractionState.frontRight().normalForceNewtons());
    assertTrue(
        tractionState.frontRight().normalForceNewtons()
            > tractionState.frontLeft().normalForceNewtons());
    assertTrue(
        tractionState.rearRight().normalForceNewtons()
            > tractionState.rearLeft().normalForceNewtons());
  }

  @Test
  void disablesTractionCapacityWhenClearanceFails() {
    ChassisState3d chassisState3d =
        new ChassisState3d(
            new Pose3d(0.0, 0.0, 0.0, new Rotation3d()),
            new Translation3d(),
            new Translation3d(),
            new AngularVelocity3d(0.0, 0.0, 0.0));
    TerrainContactSample terrainContactSample =
        new TerrainContactSample(
            new TerrainSample(chassisState3d.pose(), 0.0, 0.0, 0.0),
            TerrainFeature.BLUE_LEFT_TRENCH,
            0.4,
            0.03,
            -0.1,
            true,
            false);

    SwerveTractionState tractionState =
        SwerveLoadTransferEstimator.estimate(
            chassisState3d,
            terrainContactSample,
            new ChassisMassProperties(45.0, 0.28, 0.65, 0.58, 1.05));

    assertFalse(tractionState.tractionAvailable());
    assertEquals(0.0, tractionState.frontLeft().tractionCapacityNewtons(), 1e-9);
    assertEquals(0.0, tractionState.frontRight().tractionCapacityNewtons(), 1e-9);
    assertEquals(0.0, tractionState.rearLeft().tractionCapacityNewtons(), 1e-9);
    assertEquals(0.0, tractionState.rearRight().tractionCapacityNewtons(), 1e-9);
  }
}
