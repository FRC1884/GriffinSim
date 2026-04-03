package org.griffins1884.sim3d.integration;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.griffins1884.sim3d.AngularVelocity3d;
import org.griffins1884.sim3d.ChassisState3d;
import org.griffins1884.sim3d.DriveSimulationAdapter;
import org.griffins1884.sim3d.SimImuSample;
import org.griffins1884.sim3d.SwerveTractionState;
import org.griffins1884.sim3d.TerrainContactSample;
import org.griffins1884.sim3d.TerrainFeature;
import org.griffins1884.sim3d.TerrainSample;
import org.griffins1884.sim3d.WheelLoadSample;
import org.junit.jupiter.api.Test;

class DriveSimulationIntegrationsTest {
  @Test
  void createsHolonomicAutoHooksFromSimulationAdapter() {
    FakeAdapter adapter = new FakeAdapter();

    HolonomicAutoHooks hooks = DriveSimulationIntegrations.holonomicAutoHooks(adapter);
    Pose2d resetPose = new Pose2d(3.0, 2.0, Rotation2d.fromDegrees(15.0));
    hooks.poseResetter().accept(resetPose);

    assertEquals(adapter.pose2d, hooks.poseSupplier().get());
    assertEquals(adapter.robotRelativeSpeeds, hooks.robotRelativeChassisSpeedsSupplier().get());
    assertSame(adapter.chassisState3d, hooks.chassisState3dSupplier().get());
    assertSame(adapter.terrainContactSample, hooks.terrainContactSupplier().get());
    assertSame(adapter.tractionState, hooks.tractionStateSupplier().get());
    assertEquals(resetPose, adapter.lastResetPose);
  }

  @Test
  void createsVisualizationFrameWithStaticMarkers() {
    FakeAdapter adapter = new FakeAdapter();
    FieldMarkerSample[] markers =
        new FieldMarkerSample[] {
          new FieldMarkerSample("marker-a", new Pose3d()),
          new FieldMarkerSample("marker-b", new Pose3d(1.0, 2.0, 3.0, new Rotation3d()))
        };

    VisualizationFrame frame =
        DriveSimulationIntegrations.visualizationFrame(adapter, () -> markers);

    assertEquals(adapter.pose3d, frame.robotPose());
    assertSame(adapter.chassisState3d, frame.chassisState3d());
    assertSame(adapter.imuSample, frame.imuSample());
    assertSame(adapter.terrainContactSample, frame.terrainContactSample());
    assertSame(adapter.tractionState, frame.tractionState());
    assertArrayEquals(markers, frame.fieldMarkers());
  }

  private static final class FakeAdapter implements DriveSimulationAdapter {
    private Pose2d pose2d = new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(10.0));
    private Pose3d pose3d = new Pose3d(1.0, 2.0, 0.3, new Rotation3d(0.1, 0.2, 0.3));
    private ChassisState3d chassisState3d =
        new ChassisState3d(
            pose3d,
            new Translation3d(1.2, -0.3, 0.1),
            new Translation3d(0.4, 0.1, -0.2),
            new AngularVelocity3d(0.3, 0.2, 0.1));
    private ChassisSpeeds robotRelativeSpeeds = new ChassisSpeeds(1.1, -0.2, 0.5);
    private ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds(1.0, 0.3, 0.5);
    private SimImuSample imuSample = new SimImuSample(new Rotation3d(0.1, 0.2, 0.3), new AngularVelocity3d(0.3, 0.2, 0.1));
    private TerrainContactSample terrainContactSample =
        new TerrainContactSample(
            new TerrainSample(pose3d, 0.1, 0.2, 0.3),
            TerrainFeature.BLUE_LEFT_BUMP,
            Double.POSITIVE_INFINITY,
            -0.02,
            Double.POSITIVE_INFINITY,
            true,
            true);
    private SwerveTractionState tractionState =
        new SwerveTractionState(
            new WheelLoadSample(100, 110, 0.8),
            new WheelLoadSample(110, 121, 0.9),
            new WheelLoadSample(120, 132, 1.0),
            new WheelLoadSample(130, 143, 1.1),
            460,
            0.95,
            true);
    private Pose2d lastResetPose = null;

    @Override
    public Pose2d getPose2d() {
      return pose2d;
    }

    @Override
    public Pose3d getPose3d() {
      return pose3d;
    }

    @Override
    public ChassisState3d getChassisState3d() {
      return chassisState3d;
    }

    @Override
    public TerrainContactSample getTerrainContactSample() {
      return terrainContactSample;
    }

    @Override
    public SwerveTractionState getTractionState() {
      return tractionState;
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
    public SimImuSample getImuSample() {
      return imuSample;
    }

    @Override
    public TerrainSample getTerrainSample() {
      return terrainContactSample.terrainSample();
    }

    @Override
    public org.griffins1884.sim3d.DriveSimulationState getState() {
      throw new UnsupportedOperationException();
    }

    @Override
    public void resetPose(Pose2d pose) {
      lastResetPose = pose;
      pose2d = pose;
    }

    @Override
    public void resetState(Pose2d pose, ChassisSpeeds robotRelativeSpeeds) {
      throw new UnsupportedOperationException();
    }
  }
}
