package org.griffins1884.sim3d;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import org.junit.jupiter.api.Test;

class TerrainDriveLawsTest {
  private static final TerrainSample TERRAIN_SAMPLE =
      new TerrainSample(new Pose3d(0.0, 0.0, 0.1, new Rotation3d()), 0.0, 0.0, 0.1);

  @Test
  void driveAuthorityFallsToZeroOnBlockedContact() {
    SwerveTractionState tractionState =
        new SwerveTractionState(
            new WheelLoadSample(100.0, 120.0, 1.0),
            new WheelLoadSample(100.0, 120.0, 1.0),
            new WheelLoadSample(100.0, 120.0, 1.0),
            new WheelLoadSample(100.0, 120.0, 1.0),
            400.0,
            1.0,
            false);

    double scale =
        TerrainDriveLaws.driveAuthorityScale(
            tractionState,
            SwerveCorner.FRONT_LEFT,
            new TerrainContactSample(
                TERRAIN_SAMPLE, TerrainFeature.BLUE_HUB, Double.POSITIVE_INFINITY, 0.1, 0.1, false, false));

    assertEquals(0.0, scale, 1e-9);
  }

  @Test
  void steeringAuthorityRespondsToLoadButStaysPositiveOnTraversableTerrain() {
    SwerveTractionState tractionState =
        new SwerveTractionState(
            new WheelLoadSample(120.0, 140.0, 1.2),
            new WheelLoadSample(80.0, 90.0, 0.8),
            new WheelLoadSample(110.0, 130.0, 1.1),
            new WheelLoadSample(90.0, 100.0, 0.9),
            400.0,
            1.0,
            true);

    double frontLeftScale =
        TerrainDriveLaws.steerAuthorityScale(
            tractionState,
            SwerveCorner.FRONT_LEFT,
            new TerrainContactSample(
                TERRAIN_SAMPLE, TerrainFeature.BLUE_LEFT_BUMP, Double.POSITIVE_INFINITY, 0.1, 0.1, true, true));
    double frontRightScale =
        TerrainDriveLaws.steerAuthorityScale(
            tractionState,
            SwerveCorner.FRONT_RIGHT,
            new TerrainContactSample(
                TERRAIN_SAMPLE, TerrainFeature.BLUE_LEFT_BUMP, Double.POSITIVE_INFINITY, 0.1, 0.1, true, true));

    assertTrue(frontLeftScale > frontRightScale);
    assertTrue(frontRightScale > 0.0);
  }

  @Test
  void slopeReducesAuthorityOnTraversableTerrain() {
    SwerveTractionState tractionState =
        new SwerveTractionState(
            new WheelLoadSample(100.0, 100.0, 1.0),
            new WheelLoadSample(100.0, 100.0, 1.0),
            new WheelLoadSample(100.0, 100.0, 1.0),
            new WheelLoadSample(100.0, 100.0, 1.0),
            400.0,
            1.0,
            true);

    TerrainContactSample flatContact =
        new TerrainContactSample(
            new TerrainSample(new Pose3d(), 0.0, 0.0, 0.0),
            TerrainFeature.FLAT,
            Double.POSITIVE_INFINITY,
            0.1,
            0.1,
            true,
            true);
    TerrainContactSample slopedContact =
        new TerrainContactSample(
            new TerrainSample(new Pose3d(0.0, 0.0, 0.1, new Rotation3d(0.0, 0.28, 0.0)), 0.0, 0.28, 0.1),
            TerrainFeature.BLUE_LEFT_BUMP,
            Double.POSITIVE_INFINITY,
            0.1,
            0.1,
            true,
            true);

    double flatDrive =
        TerrainDriveLaws.driveAuthorityScale(tractionState, SwerveCorner.FRONT_LEFT, flatContact);
    double slopedDrive =
        TerrainDriveLaws.driveAuthorityScale(
            tractionState, SwerveCorner.FRONT_LEFT, slopedContact);
    double flatSteer =
        TerrainDriveLaws.steerAuthorityScale(tractionState, SwerveCorner.FRONT_LEFT, flatContact);
    double slopedSteer =
        TerrainDriveLaws.steerAuthorityScale(
            tractionState, SwerveCorner.FRONT_LEFT, slopedContact);

    assertTrue(slopedDrive < flatDrive);
    assertTrue(slopedSteer < flatSteer);
  }
}
