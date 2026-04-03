package org.griffins1884.sim3d.native6dof;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.concurrent.atomic.AtomicReference;
import org.griffins1884.sim3d.ChassisFootprint;
import org.griffins1884.sim3d.ChassisMassProperties;
import org.griffins1884.sim3d.DriveSimulationState;
import org.griffins1884.sim3d.TerrainModel;
import org.griffins1884.sim3d.PlanarObstacleContactSample;
import org.griffins1884.sim3d.TerrainContactSample;
import org.griffins1884.sim3d.TerrainSample;
import org.griffins1884.sim3d.seasonspecific.rebuilt2026.Rebuilt2026FieldContactModel;
import org.griffins1884.sim3d.seasonspecific.rebuilt2026.Rebuilt2026FieldContactModel.RectRegion;
import org.griffins1884.sim3d.TerrainFeature;
import org.junit.jupiter.api.Test;

class NativeSixDofFieldContactScaffoldingTest {
  private static final ChassisFootprint FOOTPRINT =
      new ChassisFootprint(0.72, 0.72, 0.34, 0.08);
  private static final ChassisMassProperties MASS_PROPERTIES =
      new ChassisMassProperties(61.235, 0.30, 0.72, 0.72, 1.10);
  private static final NativeSixDofSwerveConfig CONFIG =
      NativeSixDofSwerveConfig.rigidSwerve(FOOTPRINT, MASS_PROPERTIES, 0.72, 0.72);

  @Test
  void rebuilt2026ContactModelFlagsHubAndReturnBlockersAsBlockedReferences() {
    TerrainContactSample hubSample =
        Rebuilt2026FieldContactModel.INSTANCE.sampleContact(
            new Pose2d(
                Rebuilt2026FieldContactModel.hubCenterXBlueMeters(),
                Rebuilt2026FieldContactModel.hubCenterYMeters(),
                new Rotation2d()),
            FOOTPRINT);

    RectRegion[] blueLeftReturns = Rebuilt2026FieldContactModel.blueLeftTrenchEdgeRegions();
    TerrainContactSample blueLeftReturnSample = sampleAtCenter(blueLeftReturns[3]);
    TerrainContactSample blueRightReturnSample = sampleAtCenter(blueLeftReturns[4]);

    assertEquals(TerrainFeature.BLUE_HUB, hubSample.feature());
    assertFalse(hubSample.traversableSurface());
    assertFalse(hubSample.clearanceSatisfied());

    assertEquals(TerrainFeature.BLUE_LEFT_TRENCH_EDGE, blueLeftReturnSample.feature());
    assertFalse(blueLeftReturnSample.traversableSurface());
    assertFalse(blueLeftReturnSample.clearanceSatisfied());

    assertEquals(TerrainFeature.BLUE_LEFT_TRENCH_EDGE, blueRightReturnSample.feature());
    assertFalse(blueRightReturnSample.traversableSurface());
    assertFalse(blueRightReturnSample.clearanceSatisfied());
  }

  @Test
  void rebuilt2026ObstacleModelReturnsHardContactSamplesForTrenchReturn() {
    RectRegion topReturnBlocker = Rebuilt2026FieldContactModel.blueLeftTrenchEdgeRegions()[3];
    Translation2d center =
        new Translation2d(
            (topReturnBlocker.minX() + topReturnBlocker.maxX()) * 0.5,
            (topReturnBlocker.minY() + topReturnBlocker.maxY()) * 0.5);

    PlanarObstacleContactSample[] contacts =
        Rebuilt2026FieldContactModel.INSTANCE.samplePlanarObstacleContacts(center, 0.0);

    assertTrue(contacts.length > 0, "expected obstacle contacts at trench return center");
  }

  @Test
  void rebuilt2026ObstacleModelReturnsHardContactSamplesForBlueTowerBackWall() {
    double backWallCenterX =
        Rebuilt2026FieldContactModel.blueTowerFrontFaceXMeters()
            - Rebuilt2026FieldContactModel.towerDepthMeters();
    Translation2d center =
        new Translation2d(backWallCenterX, Rebuilt2026FieldContactModel.blueTowerCenterYMeters());

    PlanarObstacleContactSample[] contacts =
        Rebuilt2026FieldContactModel.INSTANCE.samplePlanarObstacleContacts(center, 0.0);

    assertTrue(contacts.length > 0, "expected obstacle contacts at blue tower back wall center");
  }

  @Test
  void rebuilt2026ObstacleModelReturnsHardContactSamplesForRedTowerBackWall() {
    double backWallCenterX =
        Rebuilt2026FieldContactModel.redTowerFrontFaceXMeters()
            + Rebuilt2026FieldContactModel.towerDepthMeters();
    Translation2d center =
        new Translation2d(backWallCenterX, Rebuilt2026FieldContactModel.redTowerCenterYMeters());

    PlanarObstacleContactSample[] contacts =
        Rebuilt2026FieldContactModel.INSTANCE.samplePlanarObstacleContacts(center, 0.0);

    assertTrue(contacts.length > 0, "expected obstacle contacts at red tower back wall center");
  }

  @Test
  void nativeLateralCollisionAgainstBlueHubShouldStopPenetration() {
    assertNativeHubCollisionStopsPenetration(
        Rebuilt2026FieldContactModel.hubCenterXBlueMeters(),
        Rebuilt2026FieldContactModel.hubCenterYMeters(),
        2.0,
        false);
  }

  @Test
  void nativeLateralCollisionAgainstRedHubShouldStopPenetration() {
    assertNativeHubCollisionStopsPenetration(
        Rebuilt2026FieldContactModel.hubCenterXRedMeters(),
        Rebuilt2026FieldContactModel.hubCenterYMeters(),
        2.0,
        true);
  }

  @Test
  void nativeLateralCollisionAgainstBlueTrenchEndBlockersShouldStopPenetration() {
    assertNativeTrenchReturnCollisionStopsPenetration(
        Rebuilt2026FieldContactModel.blueLeftTrenchEdgeRegions()[3], 1.5, false);
  }

  @Test
  void nativeLateralCollisionAgainstRedTrenchEndBlockersShouldStopPenetration() {
    assertNativeTrenchReturnCollisionStopsPenetration(
        Rebuilt2026FieldContactModel.redLeftTrenchEdgeRegions()[3], 1.5, false);
  }

  @Test
  void nativeHeadOnCollisionAgainstBlueTowerBackWallShouldStopPenetration() {
    assertNativeTowerBackWallCollisionStopsPenetration(
        Rebuilt2026FieldContactModel.blueTowerFrontFaceXMeters()
            - Rebuilt2026FieldContactModel.towerDepthMeters(),
        Rebuilt2026FieldContactModel.blueTowerCenterYMeters(),
        false);
  }

  @Test
  void nativeHeadOnCollisionAgainstRedTowerBackWallShouldStopPenetration() {
    assertNativeTowerBackWallCollisionStopsPenetration(
        Rebuilt2026FieldContactModel.redTowerFrontFaceXMeters()
            + Rebuilt2026FieldContactModel.towerDepthMeters(),
        Rebuilt2026FieldContactModel.redTowerCenterYMeters(),
        true);
  }

  private void assertNativeHubCollisionStopsPenetration(
      double hubCenterXMeters, double hubCenterYMeters, double speedMetersPerSecond, boolean fromRight) {
    AtomicReference<Double> timeSec = new AtomicReference<>(0.0);
    NativeSixDofSwerveSimulation simulation =
        new NativeSixDofSwerveSimulation(
            CONFIG, Rebuilt2026FieldContactModel.INSTANCE, timeSec::get);
    NativeSixDofSwerveSimulation freeSimulation =
        new NativeSixDofSwerveSimulation(CONFIG, flatTerrain(), timeSec::get);
    double startXMeters = hubCenterXMeters + (fromRight ? 0.9 : -0.9);
    double commandedSpeedMetersPerSecond = fromRight ? -speedMetersPerSecond : speedMetersPerSecond;
    Rotation2d heading = Rotation2d.fromRadians(fromRight ? Math.PI : 0.0);
    Pose2d startPose =
        new Pose2d(
            startXMeters,
            hubCenterYMeters,
            heading);

    simulation.resetState(startPose, new ChassisSpeeds(speedMetersPerSecond, 0.0, 0.0));
    freeSimulation.resetState(startPose, new ChassisSpeeds(speedMetersPerSecond, 0.0, 0.0));

    simulation.getState();
    freeSimulation.getState();
    timeSec.set(1.0);
    DriveSimulationState settledState = simulation.getState();
    DriveSimulationState freeState = freeSimulation.getState();

    assertTrue(
        fromRight
            ? settledState.pose2d().getX() > freeState.pose2d().getX() + 0.2
            : settledState.pose2d().getX() < freeState.pose2d().getX() - 0.2,
        "hub collision did not materially limit forward travel: "
            + settledState.pose2d().getX()
            + " vs free "
            + freeState.pose2d().getX());
    assertTrue(
        settledState.robotRelativeChassisSpeeds().vxMetersPerSecond
            < freeState.robotRelativeChassisSpeeds().vxMetersPerSecond - 0.2,
        "hub collision did not materially slow the chassis");
  }

  private void assertNativeTrenchReturnCollisionStopsPenetration(
      RectRegion returnBlocker, double speedMetersPerSecond, boolean fromRight) {
    AtomicReference<Double> timeSec = new AtomicReference<>(0.0);
    NativeSixDofSwerveSimulation simulation =
        new NativeSixDofSwerveSimulation(
            CONFIG, Rebuilt2026FieldContactModel.INSTANCE, timeSec::get);

    double blockerCenterY = (returnBlocker.minY() + returnBlocker.maxY()) * 0.5;
    double startXMeters = fromRight ? returnBlocker.maxX() + 0.9 : returnBlocker.minX() - 0.9;
    Rotation2d heading = Rotation2d.fromRadians(fromRight ? Math.PI : 0.0);

    simulation.resetState(
        new Pose2d(startXMeters, blockerCenterY, heading),
        new ChassisSpeeds(speedMetersPerSecond, 0.0, 0.0));

    simulation.getState();
    timeSec.set(1.0);
    DriveSimulationState settledState = simulation.getState();

    double leadingEdgeX =
        settledState.pose2d().getX()
            + ((fromRight ? -1.0 : 1.0) * (FOOTPRINT.lengthMeters() * 0.5));
    assertTrue(
        fromRight
            ? leadingEdgeX >= returnBlocker.maxX() - 0.03
            : leadingEdgeX <= returnBlocker.minX() + 0.03,
        "leading edge penetrated trench return: "
            + leadingEdgeX
            + " vs blocker "
            + returnBlocker);
    assertTrue(
        settledState.robotRelativeChassisSpeeds().vxMetersPerSecond < 0.5,
        "trench return collision did not materially slow the chassis");
  }

  private void assertNativeTowerBackWallCollisionStopsPenetration(
      double backWallCenterXMeters, double towerCenterYMeters, boolean fromFront) {
    AtomicReference<Double> timeSec = new AtomicReference<>(0.0);
    NativeSixDofSwerveSimulation simulation =
        new NativeSixDofSwerveSimulation(
            CONFIG, Rebuilt2026FieldContactModel.INSTANCE, timeSec::get);
    NativeSixDofSwerveSimulation freeSimulation =
        new NativeSixDofSwerveSimulation(CONFIG, flatTerrain(), timeSec::get);
    Pose2d startPose =
        new Pose2d(
            backWallCenterXMeters + (fromFront ? -0.9 : 0.9),
            towerCenterYMeters,
            Rotation2d.fromRadians(fromFront ? 0.0 : Math.PI));

    simulation.resetState(startPose, new ChassisSpeeds(1.5, 0.0, 0.0));
    freeSimulation.resetState(startPose, new ChassisSpeeds(1.5, 0.0, 0.0));

    simulation.getState();
    freeSimulation.getState();
    timeSec.set(1.0);
    DriveSimulationState settledState = simulation.getState();
    DriveSimulationState freeState = freeSimulation.getState();

    assertTrue(
        fromFront
            ? settledState.pose2d().getX() < backWallCenterXMeters - 0.02
            : settledState.pose2d().getX() > backWallCenterXMeters + 0.02,
        "tower collision allowed chassis center through the back wall: "
            + settledState.pose2d().getX()
            + " vs wall center "
            + backWallCenterXMeters);
    assertTrue(
        fromFront
            ? settledState.pose2d().getX() < freeState.pose2d().getX() - 0.2
            : settledState.pose2d().getX() > freeState.pose2d().getX() + 0.2,
        "tower collision did not materially limit field travel: "
            + settledState.pose2d().getX()
            + " vs free "
            + freeState.pose2d().getX());
    assertTrue(
        settledState.robotRelativeChassisSpeeds().vxMetersPerSecond
            < freeState.robotRelativeChassisSpeeds().vxMetersPerSecond - 0.2,
        "tower collision did not materially slow the chassis");
  }

  private static TerrainContactSample sampleAtCenter(RectRegion region) {
    return Rebuilt2026FieldContactModel.INSTANCE.sampleContact(
        new Pose2d(
            (region.minX() + region.maxX()) * 0.5,
            (region.minY() + region.maxY()) * 0.5,
            new Rotation2d()),
        FOOTPRINT);
  }

  private static TerrainModel flatTerrain() {
    return pose ->
        new TerrainSample(
            new Pose3d(pose.getX(), pose.getY(), 0.0, new Rotation3d()),
            0.0,
            0.0,
            0.0);
  }
}
