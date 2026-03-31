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
    Pose2d robotPose = new Pose2d(4.604766, 5.5453534, new Rotation2d());

    TerrainContactSample sample =
        MODEL.sampleContact(robotPose, new ChassisFootprint(0.9, 0.9, 0.6, 0.08));

    assertEquals(TerrainFeature.BLUE_LEFT_BUMP, sample.feature());
    assertTrue(sample.terrainSample().heightMeters() > 0.15);
    assertTrue(sample.traversableSurface());
    assertTrue(sample.clearanceSatisfied());
  }

  @Test
  void detectsTrenchUnderpassAndChecksOverheadClearance() {
    Translation2d blueLeftTrenchCenter =
        new Translation2d(
            4.604766, Rebuilt2026FieldContactModel.fieldWidthMeters() - Units.inchesToMeters(25.17));

    TerrainContactSample shortRobot =
        MODEL.sampleContact(
            new Pose2d(blueLeftTrenchCenter, Rotation2d.fromDegrees(90.0)),
            new ChassisFootprint(0.9, 0.9, 0.45, 0.08));
    TerrainContactSample tallRobot =
        MODEL.sampleContact(
            new Pose2d(blueLeftTrenchCenter, Rotation2d.fromDegrees(90.0)),
            new ChassisFootprint(0.9, 0.9, 0.7, 0.08));

    assertEquals(TerrainFeature.BLUE_LEFT_TRENCH, shortRobot.feature());
    assertTrue(shortRobot.clearanceSatisfied());
    assertTrue(shortRobot.overheadClearanceMeters() < Double.POSITIVE_INFINITY);

    assertEquals(TerrainFeature.BLUE_LEFT_TRENCH, tallRobot.feature());
    assertFalse(tallRobot.clearanceSatisfied());
    assertTrue(tallRobot.overheadClearanceMarginMeters() < 0.0);
  }

  @Test
  void mirrorsBumpAndTrenchFeaturesOnRedSide() {
    Pose2d redRightBumpPose = new Pose2d(11.8947438, 2.4973534, new Rotation2d());
    Pose2d redRightTrenchPose = new Pose2d(11.8947438, Units.inchesToMeters(25.17), new Rotation2d());

    TerrainContactSample bumpSample =
        MODEL.sampleContact(redRightBumpPose, new ChassisFootprint(0.9, 0.9, 0.6, 0.08));
    TerrainContactSample trenchSample =
        MODEL.sampleContact(redRightTrenchPose, new ChassisFootprint(0.9, 0.9, 0.45, 0.08));

    assertEquals(TerrainFeature.RED_RIGHT_BUMP, bumpSample.feature());
    assertTrue(bumpSample.terrainSample().heightMeters() > 0.1);
    assertEquals(TerrainFeature.RED_RIGHT_TRENCH, trenchSample.feature());
    assertTrue(trenchSample.clearanceSatisfied());
  }
}
