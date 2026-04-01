package org.griffins1884.sim3d.integration;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertInstanceOf;
import static org.junit.jupiter.api.Assertions.assertNotNull;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import java.util.concurrent.atomic.AtomicReference;
import org.griffins1884.sim3d.ChassisFootprint;
import org.griffins1884.sim3d.ChassisMassProperties;
import org.griffins1884.sim3d.CommandableDriveSimulationAdapter;
import org.griffins1884.sim3d.DriveSimulationAdapter;
import org.griffins1884.sim3d.TerrainSample;
import org.griffins1884.sim3d.native6dof.NativeSixDofSwerveConfig;
import org.junit.jupiter.api.Test;

class DriveSimulationFactoriesTest {
  @Test
  void createsNativeSixDofAdapterBehindStableInterface() {
    AtomicReference<Double> timeSec = new AtomicReference<>(0.0);
    NativeSixDofSwerveConfig config =
        NativeSixDofSwerveConfig.rigidSwerve(
            new ChassisFootprint(0.72, 0.72, 0.34, 0.08),
            new ChassisMassProperties(61.235, 0.30, 0.72, 0.72, 1.10),
            0.72,
            0.72);

    DriveSimulationAdapter adapter =
        DriveSimulationFactories.nativeSixDof(
            config,
            pose ->
                new TerrainSample(
                    new edu.wpi.first.math.geometry.Pose3d(
                        pose.getX(), pose.getY(), 0.0, new Rotation3d()),
                    0.0,
                    0.0,
                    0.0),
            timeSec::get);

    adapter.resetPose(new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(15.0)));

    assertNotNull(adapter.getState());
    assertInstanceOf(CommandableDriveSimulationAdapter.class, adapter);
    assertEquals(1.0, adapter.getPose2d().getX(), 1e-9);
    assertEquals(2.0, adapter.getPose2d().getY(), 1e-9);
    assertEquals(15.0, adapter.getPose2d().getRotation().getDegrees(), 1e-9);
  }
}
