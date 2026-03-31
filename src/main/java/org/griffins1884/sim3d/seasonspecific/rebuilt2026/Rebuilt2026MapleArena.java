package org.griffins1884.sim3d.seasonspecific.rebuilt2026;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.dyn4j.dynamics.Settings;
import org.ironmaple.simulation.SimulatedArena;

/**
 * 2026 rebuilt-season Maple arena with explicit 2D blockers for field borders, hubs, trench edge
 * walls, and tower bodies.
 */
public final class Rebuilt2026MapleArena extends SimulatedArena {
  public Rebuilt2026MapleArena() {
    this(true);
  }

  public Rebuilt2026MapleArena(boolean includeTraversalStructures) {
    super(new RebuiltFieldObstaclesMap(includeTraversalStructures));

    Settings settings = physicsWorld.getSettings();
    settings.setMinimumAtRestTime(0.02);
    physicsWorld.setSettings(settings);
  }

  @Override
  public void placeGamePiecesOnField() {
    // GriffinSim leaves game-piece spawning to season repos. This arena covers field collisions only.
  }

  public static final class RebuiltFieldObstaclesMap extends FieldMap {
    private static final double FIELD_X_MIN = 0.0;
    private static final double FIELD_X_MAX = Rebuilt2026FieldContactModel.fieldLengthMeters();
    private static final double FIELD_Y_MIN = 0.0;
    private static final double FIELD_Y_MAX = Rebuilt2026FieldContactModel.fieldWidthMeters();

    private static final double INTERNAL_WALL_THICKNESS = Inches.of(2.0).in(Meters);
    private static final double HUB_SIZE_METERS = Inches.of(47.0).in(Meters);
    private static final double TOWER_WIDTH_METERS = Inches.of(49.25).in(Meters);
    private static final double TOWER_DEPTH_METERS = Inches.of(45.0).in(Meters);
    private static final double TOWER_INNER_OPENING_WIDTH_METERS = Inches.of(32.250).in(Meters);
    private static final double TOWER_SIDE_WIDTH_METERS =
        (TOWER_WIDTH_METERS - TOWER_INNER_OPENING_WIDTH_METERS) * 0.5;
    private static final double TRENCH_DEPTH_METERS = Inches.of(47.0).in(Meters);
    private static final double TRENCH_OPENING_WIDTH_METERS = Inches.of(50.34).in(Meters);
    private static final double TRENCH_TOTAL_WIDTH_METERS = Inches.of(65.65).in(Meters);
    private static final double TRENCH_EDGE_WALL_WIDTH_METERS =
        (TRENCH_TOTAL_WIDTH_METERS - TRENCH_OPENING_WIDTH_METERS) * 0.5;

    RebuiltFieldObstaclesMap(boolean includeTraversalStructures) {
      addFieldBorder();
      addHubObstacles();
      addTrenchEdgeWalls();
      addTowerObstacles();

      if (!includeTraversalStructures) {
        addTraversalStructureBlocks();
      }
    }

    private void addFieldBorder() {
      addBorderLine(
          new Translation2d(FIELD_X_MIN, FIELD_Y_MIN), new Translation2d(FIELD_X_MIN, FIELD_Y_MAX));
      addBorderLine(
          new Translation2d(FIELD_X_MAX, FIELD_Y_MIN), new Translation2d(FIELD_X_MAX, FIELD_Y_MAX));
      addBorderLine(
          new Translation2d(FIELD_X_MIN, FIELD_Y_MIN), new Translation2d(FIELD_X_MAX, FIELD_Y_MIN));
      addBorderLine(
          new Translation2d(FIELD_X_MIN, FIELD_Y_MAX), new Translation2d(FIELD_X_MAX, FIELD_Y_MAX));
    }

    private void addHubObstacles() {
      addRectangularObstacle(
          HUB_SIZE_METERS,
          HUB_SIZE_METERS,
          new Pose2d(
              hubCenterXBlue(),
              Rebuilt2026FieldContactModel.hubCenterYMeters(),
              new Rotation2d()));
      addRectangularObstacle(
          HUB_SIZE_METERS,
          HUB_SIZE_METERS,
          new Pose2d(
              hubCenterXRed(),
              Rebuilt2026FieldContactModel.hubCenterYMeters(),
              new Rotation2d()));
    }

    private void addTrenchEdgeWalls() {
      addRectangularObstacle(
          TRENCH_DEPTH_METERS,
          TRENCH_EDGE_WALL_WIDTH_METERS,
          new Pose2d(
              hubCenterXBlue(),
              FIELD_Y_MAX - TRENCH_OPENING_WIDTH_METERS - (TRENCH_EDGE_WALL_WIDTH_METERS * 0.5),
              new Rotation2d()));
      addRectangularObstacle(
          TRENCH_DEPTH_METERS,
          TRENCH_EDGE_WALL_WIDTH_METERS,
          new Pose2d(
              hubCenterXBlue(),
              TRENCH_OPENING_WIDTH_METERS + (TRENCH_EDGE_WALL_WIDTH_METERS * 0.5),
              new Rotation2d()));
      addRectangularObstacle(
          TRENCH_DEPTH_METERS,
          TRENCH_EDGE_WALL_WIDTH_METERS,
          new Pose2d(
              hubCenterXRed(),
              FIELD_Y_MAX - TRENCH_OPENING_WIDTH_METERS - (TRENCH_EDGE_WALL_WIDTH_METERS * 0.5),
              new Rotation2d()));
      addRectangularObstacle(
          TRENCH_DEPTH_METERS,
          TRENCH_EDGE_WALL_WIDTH_METERS,
          new Pose2d(
              hubCenterXRed(),
              TRENCH_OPENING_WIDTH_METERS + (TRENCH_EDGE_WALL_WIDTH_METERS * 0.5),
              new Rotation2d()));
    }

    private void addTowerObstacles() {
      addTowerSideWalls(
          Rebuilt2026FieldContactModel.blueTowerFrontFaceXMeters(),
          Rebuilt2026FieldContactModel.blueTowerCenterYMeters(),
          true);
      addTowerSideWalls(
          Rebuilt2026FieldContactModel.redTowerFrontFaceXMeters(),
          Rebuilt2026FieldContactModel.redTowerCenterYMeters(),
          false);
    }

    private void addTraversalStructureBlocks() {
      addRectangularObstacle(
          Inches.of(44.4).in(Meters),
          Inches.of(73.0).in(Meters),
          new Pose2d(hubCenterXBlue(), FIELD_Y_MAX - Inches.of(60.0).in(Meters), new Rotation2d()));
      addRectangularObstacle(
          Inches.of(44.4).in(Meters),
          Inches.of(73.0).in(Meters),
          new Pose2d(hubCenterXBlue(), Inches.of(60.0).in(Meters), new Rotation2d()));
      addRectangularObstacle(
          Inches.of(44.4).in(Meters),
          Inches.of(73.0).in(Meters),
          new Pose2d(hubCenterXRed(), FIELD_Y_MAX - Inches.of(60.0).in(Meters), new Rotation2d()));
      addRectangularObstacle(
          Inches.of(44.4).in(Meters),
          Inches.of(73.0).in(Meters),
          new Pose2d(hubCenterXRed(), Inches.of(60.0).in(Meters), new Rotation2d()));
    }

    private void addTowerSideWalls(double frontFaceX, double centerY, boolean blueSide) {
      double towerCenterX = blueSide ? frontFaceX - (TOWER_DEPTH_METERS * 0.5) : frontFaceX + (TOWER_DEPTH_METERS * 0.5);
      double leftWallCenterY =
          centerY
              + (TOWER_INNER_OPENING_WIDTH_METERS * 0.5)
              + (TOWER_SIDE_WIDTH_METERS * 0.5);
      double rightWallCenterY =
          centerY
              - (TOWER_INNER_OPENING_WIDTH_METERS * 0.5)
              - (TOWER_SIDE_WIDTH_METERS * 0.5);

      addRectangularObstacle(
          TOWER_DEPTH_METERS,
          TOWER_SIDE_WIDTH_METERS,
          new Pose2d(towerCenterX, leftWallCenterY, new Rotation2d()));
      addRectangularObstacle(
          TOWER_DEPTH_METERS,
          TOWER_SIDE_WIDTH_METERS,
          new Pose2d(towerCenterX, rightWallCenterY, new Rotation2d()));

      double backFaceX = blueSide ? frontFaceX - TOWER_DEPTH_METERS : frontFaceX + TOWER_DEPTH_METERS;
      addRectangularObstacle(
          INTERNAL_WALL_THICKNESS,
          TOWER_WIDTH_METERS,
          new Pose2d(backFaceX, centerY, new Rotation2d()));
    }

    private static double hubCenterXBlue() {
      return Rebuilt2026FieldContactModel.hubCenterXBlueMeters();
    }

    private static double hubCenterXRed() {
      return Rebuilt2026FieldContactModel.hubCenterXRedMeters();
    }
  }
}
