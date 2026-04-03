package org.griffins1884.sim3d.native6dof;

import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.concurrent.atomic.AtomicReference;
import org.griffins1884.sim3d.ChassisFootprint;
import org.griffins1884.sim3d.ChassisMassProperties;
import org.griffins1884.sim3d.DriveSimulationState;
import org.griffins1884.sim3d.TerrainModel;
import org.griffins1884.sim3d.TerrainSample;
import org.junit.jupiter.api.Test;

class NativeSixDofSwerveDynamicsTest {
  private static final ChassisFootprint FOOTPRINT =
      new ChassisFootprint(0.72, 0.72, 0.34, 0.08);
  private static final ChassisMassProperties MASS_PROPERTIES =
      new ChassisMassProperties(61.235, 0.30, 0.72, 0.72, 1.10);
  private static final NativeSixDofSwerveConfig CONFIG =
      NativeSixDofSwerveConfig.rigidSwerve(FOOTPRINT, MASS_PROPERTIES, 0.72, 0.72);

  @Test
  void singleCornerStepProducesWheelByWheelLoadDistribution() {
    AtomicReference<Double> timeSec = new AtomicReference<>(0.0);
    NativeSixDofSwerveSimulation simulation =
        new NativeSixDofSwerveSimulation(CONFIG, singleCornerStepTerrain(), timeSec::get);
    simulation.resetState(new Pose2d(0.0, 0.0, new Rotation2d()), new ChassisSpeeds());

    simulation.getState();
    timeSec.set(0.30);
    DriveSimulationState settledState = simulation.getState();
    double frontLeft = settledState.tractionState().frontLeft().normalForceNewtons();
    double frontRight = settledState.tractionState().frontRight().normalForceNewtons();
    double rearLeft = settledState.tractionState().rearLeft().normalForceNewtons();
    double rearRight = settledState.tractionState().rearRight().normalForceNewtons();

    assertTrue(settledState.tractionState().tractionAvailable());
    assertTrue(settledState.tractionState().totalNormalForceNewtons() > 0.0);
    assertTrue(
        Math.max(Math.max(frontLeft, frontRight), Math.max(rearLeft, rearRight))
                - Math.min(Math.min(frontLeft, frontRight), Math.min(rearLeft, rearRight))
            > 1e-3);
  }

  @Test
  void takeoffRetainsRollAndYawDynamicsInFreeFlight() {
    AtomicReference<Double> timeSec = new AtomicReference<>(0.0);
    NativeSixDofSwerveSimulation simulation =
        new NativeSixDofSwerveSimulation(CONFIG, rollLaunchTerrain(), timeSec::get);
    simulation.resetState(
        new Pose2d(0.0, 0.0, new Rotation2d()), new ChassisSpeeds(1.9, 0.0, 0.35));

    simulation.getState();
    timeSec.set(0.35);
    DriveSimulationState contactState = simulation.getState();
    timeSec.set(0.80);
    DriveSimulationState airborneState = simulation.getState();

    assertTrue(contactState.tractionState().tractionAvailable());
    assertTrue(airborneState.pose3d().getZ() > airborneState.terrainSample().heightMeters());
    assertTrue(
        Math.abs(
                airborneState.pose3d().getRotation().getX()
                    - contactState.pose3d().getRotation().getX())
            > 1e-4);
    assertTrue(
        Math.abs(
                airborneState.pose3d().getRotation().getZ()
                    - contactState.pose3d().getRotation().getZ())
            > 1e-4);
    assertTrue(Math.abs(airborneState.imuSample().rollRateRadPerSec()) > 1e-4);
    assertTrue(Math.abs(airborneState.imuSample().yawRateRadPerSec()) > 1e-4);
  }

  private static TerrainModel singleCornerStepTerrain() {
    return pose -> {
      double height = pose.getX() > 0.15 && pose.getY() > 0.15 ? 0.14 : 0.0;
      return new TerrainSample(
          new Pose3d(
              pose.getX(),
              pose.getY(),
              height,
              new Rotation3d(height > 0.0 ? 0.10 : 0.0, 0.0, 0.0)),
          height > 0.0 ? 0.10 : 0.0,
          0.0,
          height);
    };
  }

  private static TerrainModel rollLaunchTerrain() {
    return pose -> {
      double height;
      if (pose.getX() < 0.9) {
        height = pose.getY() >= 0.0 ? 0.10 : 0.0;
      } else {
        height = -0.35;
      }
      return new TerrainSample(
          new Pose3d(
              pose.getX(),
              pose.getY(),
              height,
              new Rotation3d(height > 0.0 ? 0.10 : 0.0, 0.0, 0.0)),
          height > 0.0 ? 0.10 : 0.0,
          0.0,
          height);
    };
  }
}
