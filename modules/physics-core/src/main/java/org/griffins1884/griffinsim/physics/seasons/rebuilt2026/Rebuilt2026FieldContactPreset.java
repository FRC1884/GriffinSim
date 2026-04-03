package org.griffins1884.griffinsim.physics.seasons.rebuilt2026;

import java.util.ArrayList;
import java.util.List;
import org.griffins1884.griffinsim.physics.AxisAlignedBoxObstacle;
import org.griffins1884.griffinsim.physics.AxisAlignedObstacleContactGenerator;
import org.griffins1884.griffinsim.physics.CompositeContactGenerator;
import org.griffins1884.griffinsim.physics.ContactGenerator;
import org.griffins1884.griffinsim.physics.FlatFloorContactGenerator;
import org.griffins1884.griffinsim.physics.MaterialProfile;
import org.griffins1884.griffinsim.physics.MaterialProfiles;

public final class Rebuilt2026FieldContactPreset {
  private Rebuilt2026FieldContactPreset() {}

  public static ContactGenerator create() {
    return new CompositeContactGenerator(
        List.of(
            new FlatFloorContactGenerator(0.0, MaterialProfiles.CARPET, bodyMaterials()),
            new Rebuilt2026TerrainContactGenerator(List.of(new BodyHeightProfile("robot", 0.9)), bodyMaterials()),
            new AxisAlignedObstacleContactGenerator(buildObstacles(), bodyMaterials())));
  }

  private static java.util.Map<String, MaterialProfile> bodyMaterials() {
    return java.util.Map.of("robot", MaterialProfiles.DEFAULT_BODY);
  }

  private static List<AxisAlignedBoxObstacle> buildObstacles() {
    List<AxisAlignedBoxObstacle> obstacles = new ArrayList<>();
    addFieldBorders(obstacles);
    addHubObstacles(obstacles);
    addTowerObstacles(obstacles);
    addTrenchEdgeObstacles(obstacles);
    return List.copyOf(obstacles);
  }

  private static void addFieldBorders(List<AxisAlignedBoxObstacle> obstacles) {
    obstacles.add(new AxisAlignedBoxObstacle("west-wall", -Rebuilt2026FieldConstants.WALL_THICKNESS_METERS, 0.0, 0.0, Rebuilt2026FieldConstants.FIELD_WIDTH_METERS, 0.0, 2.0, MaterialProfiles.PAINTED_STEEL));
    obstacles.add(new AxisAlignedBoxObstacle("east-wall", Rebuilt2026FieldConstants.FIELD_LENGTH_METERS, Rebuilt2026FieldConstants.FIELD_LENGTH_METERS + Rebuilt2026FieldConstants.WALL_THICKNESS_METERS, 0.0, Rebuilt2026FieldConstants.FIELD_WIDTH_METERS, 0.0, 2.0, MaterialProfiles.PAINTED_STEEL));
    obstacles.add(new AxisAlignedBoxObstacle("south-wall", 0.0, Rebuilt2026FieldConstants.FIELD_LENGTH_METERS, -Rebuilt2026FieldConstants.WALL_THICKNESS_METERS, 0.0, 0.0, 2.0, MaterialProfiles.PAINTED_STEEL));
    obstacles.add(new AxisAlignedBoxObstacle("north-wall", 0.0, Rebuilt2026FieldConstants.FIELD_LENGTH_METERS, Rebuilt2026FieldConstants.FIELD_WIDTH_METERS, Rebuilt2026FieldConstants.FIELD_WIDTH_METERS + Rebuilt2026FieldConstants.WALL_THICKNESS_METERS, 0.0, 2.0, MaterialProfiles.PAINTED_STEEL));
  }

  private static void addHubObstacles(List<AxisAlignedBoxObstacle> obstacles) {
    obstacles.add(centeredBox("blue-hub", Rebuilt2026FieldConstants.HUB_CENTER_X_BLUE_METERS, Rebuilt2026FieldConstants.HUB_CENTER_Y_METERS, Rebuilt2026FieldConstants.HUB_WIDTH_METERS, Rebuilt2026FieldConstants.HUB_DEPTH_METERS, Rebuilt2026FieldConstants.HUB_HEIGHT_METERS, MaterialProfiles.HDPE));
    obstacles.add(centeredBox("red-hub", Rebuilt2026FieldConstants.HUB_CENTER_X_RED_METERS, Rebuilt2026FieldConstants.HUB_CENTER_Y_METERS, Rebuilt2026FieldConstants.HUB_WIDTH_METERS, Rebuilt2026FieldConstants.HUB_DEPTH_METERS, Rebuilt2026FieldConstants.HUB_HEIGHT_METERS, MaterialProfiles.HDPE));
  }

  private static void addTowerObstacles(List<AxisAlignedBoxObstacle> obstacles) {
    addTower(obstacles, "blue", Rebuilt2026FieldConstants.TOWER_FRONT_FACE_X_BLUE_METERS, Rebuilt2026FieldConstants.TOWER_CENTER_Y_BLUE_METERS, true);
    addTower(obstacles, "red", Rebuilt2026FieldConstants.TOWER_FRONT_FACE_X_RED_METERS, Rebuilt2026FieldConstants.TOWER_CENTER_Y_RED_METERS, false);
  }

  private static void addTower(List<AxisAlignedBoxObstacle> obstacles, String prefix, double frontFaceX, double centerY, boolean blueSide) {
    double depth = Rebuilt2026FieldConstants.TOWER_DEPTH_METERS;
    double width = Rebuilt2026FieldConstants.TOWER_WIDTH_METERS;
    double sideWidth = (width - Rebuilt2026FieldConstants.TOWER_OPENING_WIDTH_METERS) * 0.5;
    double centerX = blueSide ? frontFaceX - (depth * 0.5) : frontFaceX + (depth * 0.5);
    double leftY = centerY + (Rebuilt2026FieldConstants.TOWER_OPENING_WIDTH_METERS * 0.5) + (sideWidth * 0.5);
    double rightY = centerY - (Rebuilt2026FieldConstants.TOWER_OPENING_WIDTH_METERS * 0.5) - (sideWidth * 0.5);
    obstacles.add(centeredBox(prefix + "-tower-left", centerX, leftY, depth, sideWidth, 2.0));
    obstacles.add(centeredBox(prefix + "-tower-right", centerX, rightY, depth, sideWidth, 2.0));
    double backWallCenterX = blueSide ? frontFaceX - depth : frontFaceX + depth;
    obstacles.add(centeredBox(prefix + "-tower-back", backWallCenterX, centerY, Rebuilt2026FieldConstants.WALL_THICKNESS_METERS, width, 2.0));
  }

  private static void addTrenchEdgeObstacles(List<AxisAlignedBoxObstacle> obstacles) {
    addTrench(obstacles, "blue-upper", Rebuilt2026FieldConstants.TRENCH_CENTER_X_BLUE_METERS, Rebuilt2026FieldConstants.TRENCH_UPPER_CENTER_Y_METERS);
    addTrench(obstacles, "blue-lower", Rebuilt2026FieldConstants.TRENCH_CENTER_X_BLUE_METERS, Rebuilt2026FieldConstants.TRENCH_LOWER_CENTER_Y_METERS);
    addTrench(obstacles, "red-upper", Rebuilt2026FieldConstants.TRENCH_CENTER_X_RED_METERS, Rebuilt2026FieldConstants.TRENCH_UPPER_CENTER_Y_METERS);
    addTrench(obstacles, "red-lower", Rebuilt2026FieldConstants.TRENCH_CENTER_X_RED_METERS, Rebuilt2026FieldConstants.TRENCH_LOWER_CENTER_Y_METERS);
  }

  private static void addTrench(List<AxisAlignedBoxObstacle> obstacles, String id, double centerX, double centerY) {
    double halfDepth = Rebuilt2026FieldConstants.TRENCH_FOOTPRINT_DEPTH_METERS * 0.5;
    double halfWidth = Rebuilt2026FieldConstants.TRENCH_FOOTPRINT_WIDTH_METERS * 0.5;
    obstacles.add(new AxisAlignedBoxObstacle(id + "-trench-top-edge", centerX - halfDepth, centerX + halfDepth, centerY + halfWidth - Rebuilt2026FieldConstants.TRENCH_EDGE_THICKNESS_METERS, centerY + halfWidth, 0.0, Rebuilt2026FieldConstants.TRENCH_OPENING_HEIGHT_METERS, MaterialProfiles.HDPE));
    obstacles.add(new AxisAlignedBoxObstacle(id + "-trench-bottom-edge", centerX - halfDepth, centerX + halfDepth, centerY - halfWidth, centerY - halfWidth + Rebuilt2026FieldConstants.TRENCH_EDGE_THICKNESS_METERS, 0.0, Rebuilt2026FieldConstants.TRENCH_OPENING_HEIGHT_METERS, MaterialProfiles.HDPE));
  }

  private static AxisAlignedBoxObstacle centeredBox(String id, double centerX, double centerY, double width, double depth, double height) {
    return centeredBox(id, centerX, centerY, width, depth, height, MaterialProfiles.PAINTED_STEEL);
  }

  private static AxisAlignedBoxObstacle centeredBox(String id, double centerX, double centerY, double width, double depth, double height, MaterialProfile material) {
    return new AxisAlignedBoxObstacle(id, centerX - (width * 0.5), centerX + (width * 0.5), centerY - (depth * 0.5), centerY + (depth * 0.5), 0.0, height, material);
  }
}
