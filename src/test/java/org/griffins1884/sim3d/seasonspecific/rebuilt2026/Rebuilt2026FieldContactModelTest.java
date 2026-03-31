package org.griffins1884.sim3d.seasonspecific.rebuilt2026;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import org.griffins1884.sim3d.ChassisFootprint;
import org.griffins1884.sim3d.TerrainContactSample;
import org.griffins1884.sim3d.TerrainFeature;
import org.junit.jupiter.api.Test;

class Rebuilt2026FieldContactModelTest {
  private static final Rebuilt2026FieldContactModel MODEL = Rebuilt2026FieldContactModel.INSTANCE;

  @Test
  void detectsBlueLeftBumpAndProducesPositiveTerrainHeight() {
    Pose2d robotPose =
        new Pose2d(
            Rebuilt2026FieldContactModel.hubCenterXBlueMeters(),
            5.525,
            new Rotation2d());

    TerrainContactSample sample =
        MODEL.sampleContact(robotPose, new ChassisFootprint(0.9, 0.9, 0.6, 0.08));

    assertEquals(TerrainFeature.BLUE_LEFT_BUMP, sample.feature());
    assertTrue(sample.terrainSample().heightMeters() > 0.15);
    assertTrue(sample.traversableSurface());
    assertTrue(sample.clearanceSatisfied());
  }

  @Test
  void detectsTrenchUnderpassAndChecksOverheadClearance() {
    Translation2d blueTrenchCenter =
        new Translation2d(
            Rebuilt2026FieldContactModel.blueTrenchCenterXMeters(),
            Rebuilt2026FieldContactModel.blueRightTrenchCenterYMeters());

    TerrainContactSample shortRobot =
        MODEL.sampleContact(
            new Pose2d(blueTrenchCenter, Rotation2d.fromDegrees(90.0)),
            new ChassisFootprint(0.9, 0.9, 0.45, 0.08));
    TerrainContactSample tallRobot =
        MODEL.sampleContact(
            new Pose2d(blueTrenchCenter, Rotation2d.fromDegrees(90.0)),
            new ChassisFootprint(0.9, 0.9, 0.7, 0.08));

    assertEquals(TerrainFeature.BLUE_RIGHT_TRENCH, shortRobot.feature());
    assertTrue(shortRobot.clearanceSatisfied());
    assertTrue(shortRobot.overheadClearanceMeters() < Double.POSITIVE_INFINITY);

    assertEquals(TerrainFeature.BLUE_RIGHT_TRENCH, tallRobot.feature());
    assertFalse(tallRobot.clearanceSatisfied());
    assertTrue(tallRobot.overheadClearanceMarginMeters() < 0.0);
  }

  @Test
  void mirrorsBumpAndTrenchFeaturesOnRedSide() {
    Pose2d redRightBumpPose =
        new Pose2d(
            Rebuilt2026FieldContactModel.hubCenterXRedMeters(),
            2.518,
            new Rotation2d());
    Pose2d redRightTrenchPose =
        new Pose2d(
            Rebuilt2026FieldContactModel.redTrenchCenterXMeters(),
            Rebuilt2026FieldContactModel.redRightTrenchCenterYMeters(),
            new Rotation2d());

    TerrainContactSample bumpSample =
        MODEL.sampleContact(redRightBumpPose, new ChassisFootprint(0.9, 0.9, 0.6, 0.08));
    TerrainContactSample trenchSample =
        MODEL.sampleContact(redRightTrenchPose, new ChassisFootprint(0.9, 0.9, 0.45, 0.08));

    assertEquals(TerrainFeature.RED_RIGHT_BUMP, bumpSample.feature());
    assertTrue(bumpSample.terrainSample().heightMeters() > 0.1);
    assertEquals(TerrainFeature.RED_RIGHT_TRENCH, trenchSample.feature());
    assertTrue(trenchSample.clearanceSatisfied());
  }

  @Test
  void blocksHubTowerAndTrenchEdgeRegions() {
    TerrainContactSample hubSample =
        MODEL.sampleContact(
            new Pose2d(
                Rebuilt2026FieldContactModel.hubCenterXBlueMeters(),
                Rebuilt2026FieldContactModel.hubCenterYMeters(),
                new Rotation2d()),
            new ChassisFootprint(0.9, 0.9, 0.45, 0.08));
    TerrainContactSample towerSample =
        MODEL.sampleContact(
            new Pose2d(
                Rebuilt2026FieldContactModel.blueTowerFrontFaceXMeters() - Units.inchesToMeters(12.0),
                Rebuilt2026FieldContactModel.blueTowerCenterYMeters()
                    + Units.inchesToMeters(20.0),
                new Rotation2d()),
            new ChassisFootprint(0.9, 0.9, 0.45, 0.08));
    TerrainContactSample trenchEdgeSample =
        MODEL.sampleContact(
            new Pose2d(
                Rebuilt2026FieldContactModel.blueTrenchCenterXMeters()
                    + Units.inchesToMeters(6.5),
                Rebuilt2026FieldContactModel.blueLeftTrenchCenterYMeters(),
                new Rotation2d()),
            new ChassisFootprint(0.9, 0.9, 0.45, 0.08));

    assertEquals(TerrainFeature.BLUE_HUB, hubSample.feature());
    assertFalse(hubSample.traversableSurface());
    assertFalse(hubSample.clearanceSatisfied());

    assertEquals(TerrainFeature.BLUE_TOWER, towerSample.feature());
    assertFalse(towerSample.traversableSurface());
    assertFalse(towerSample.clearanceSatisfied());

    assertEquals(TerrainFeature.BLUE_LEFT_TRENCH_EDGE, trenchEdgeSample.feature());
    assertFalse(trenchEdgeSample.traversableSurface());
    assertFalse(trenchEdgeSample.clearanceSatisfied());
  }

  @Test
  void bumpHeightChangesAcrossDepthNotAlongSpan() {
    double bumpCenterY = 5.525;
    double entryHeight =
        MODEL.sample(new Pose2d(4.12, bumpCenterY, new Rotation2d())).heightMeters();
    double crestHeight =
        MODEL.sample(
                new Pose2d(
                    Rebuilt2026FieldContactModel.hubCenterXBlueMeters(),
                    bumpCenterY,
                    new Rotation2d()))
            .heightMeters();
    double shiftedSpanHeight =
        MODEL.sample(
                new Pose2d(
                    Rebuilt2026FieldContactModel.hubCenterXBlueMeters(),
                    6.3,
                    new Rotation2d()))
            .heightMeters();

    assertTrue(entryHeight < crestHeight);
    assertEquals(crestHeight, shiftedSpanHeight, 1e-6);
  }

  @Test
  void matchesOfficialAndyMarkFieldDimensions() {
    assertEquals(Units.inchesToMeters(650.12), Rebuilt2026FieldContactModel.fieldLengthMeters(), 1e-9);
    assertEquals(Units.inchesToMeters(316.64), Rebuilt2026FieldContactModel.fieldWidthMeters(), 1e-9);
    assertEquals(Units.inchesToMeters(181.56), Rebuilt2026FieldContactModel.hubCenterXBlueMeters(), 1e-9);
    assertEquals(Units.inchesToMeters(468.56), Rebuilt2026FieldContactModel.hubCenterXRedMeters(), 1e-9);
    assertEquals(Units.inchesToMeters(47.00), Rebuilt2026FieldContactModel.hubCollisionHeightMeters(), 1e-9);
    assertEquals(Units.inchesToMeters(58.41), Rebuilt2026FieldContactModel.hubCollisionWidthMeters(), 1e-9);
  }

  @Test
  void trenchCornerReturnsArePartOfTheEdgeCollisionRegion() {
    Rebuilt2026FieldContactModel.RectRegion[] edgeRegions =
        Rebuilt2026FieldContactModel.blueLeftTrenchEdgeRegions();
    Rebuilt2026FieldContactModel.RectRegion topReturn = edgeRegions[3];

    TerrainContactSample sample =
        MODEL.sampleContact(
            new Pose2d(topReturn.centerX(), topReturn.centerY(), new Rotation2d()),
            new ChassisFootprint(0.9, 0.9, 0.45, 0.08));

    assertEquals(TerrainFeature.BLUE_LEFT_TRENCH_EDGE, sample.feature());
    assertFalse(sample.traversableSurface());
  }
}
