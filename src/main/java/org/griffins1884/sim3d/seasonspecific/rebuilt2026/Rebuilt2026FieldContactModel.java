package org.griffins1884.sim3d.seasonspecific.rebuilt2026;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import java.util.ArrayList;
import java.util.List;
import org.griffins1884.sim3d.ChassisFootprint;
import org.griffins1884.sim3d.TerrainContactModel;
import org.griffins1884.sim3d.TerrainContactSample;
import org.griffins1884.sim3d.TerrainFeature;
import org.griffins1884.sim3d.TerrainSample;
import org.griffins1884.sim3d.integration.FieldMarkerProvider;
import org.griffins1884.sim3d.integration.FieldMarkerSample;

/**
 * Standalone 2026 rebuilt field terrain/contact model.
 *
 * <p>This encodes the traversable bump surfaces and trench underpass clearance windows as explicit field
 * geometry, without depending on any robot-season repository or deploy assets.
 */
public final class Rebuilt2026FieldContactModel implements TerrainContactModel, FieldMarkerProvider {
  private static final double GRADIENT_SAMPLE_METERS = 0.02;

  private static final double FIELD_LENGTH_METERS = 16.518;
  private static final double FIELD_WIDTH_METERS = 8.043;
  private static final double CAD_FIELD_ORIGIN_X_METERS = 8.2497799;
  private static final double CAD_FIELD_ORIGIN_Y_METERS = 4.0213534;

  private static final RectRegion BLUE_HUB_BOUNDS = cadRect(-4257.7, -2772.5, -742.3, 742.3);
  private static final RectRegion RED_HUB_BOUNDS = cadRect(2772.5, 4257.7, -742.3, 742.3);

  private static final RectRegion BLUE_LEFT_BUMP_BOUNDS = cadRect(-4208.7, -3081.1, 517.9, 2751.1);
  private static final RectRegion BLUE_RIGHT_BUMP_BOUNDS =
      cadRect(-4208.7, -3081.1, -2751.1, -517.9);
  private static final RectRegion RED_LEFT_BUMP_BOUNDS = cadRect(3081.1, 4208.7, 517.9, 2751.1);
  private static final RectRegion RED_RIGHT_BUMP_BOUNDS = cadRect(3081.1, 4208.7, -2751.1, -517.9);

  private static final RectRegion BLUE_TRENCH_BOUNDS = cadRect(-4241.8, -3048.0, -4117.2, 4254.5);
  private static final RectRegion RED_TRENCH_BOUNDS = cadRect(3048.0, 4241.8, -4117.2, 4254.5);
  private static final RectRegion BLUE_LEFT_TRENCH_BLOCK_BOUNDS =
      cadRect(-4241.8, -3048.0, 2451.1, 4254.5);
  private static final RectRegion BLUE_RIGHT_TRENCH_BLOCK_BOUNDS =
      cadRect(-4241.8, -3048.0, -4117.2, -2451.1);
  private static final RectRegion RED_LEFT_TRENCH_BLOCK_BOUNDS =
      cadRect(3048.0, 4241.8, 2451.1, 4254.5);
  private static final RectRegion RED_RIGHT_TRENCH_BLOCK_BOUNDS =
      cadRect(3048.0, 4241.8, -4117.2, -2451.1);
  private static final RectRegion BLUE_TRENCH_OPEN_BOUNDS =
      cadRect(-4241.8, -3048.0, -2451.1, 2451.1);
  private static final RectRegion RED_TRENCH_OPEN_BOUNDS =
      cadRect(3048.0, 4241.8, -2451.1, 2451.1);
  private static final RectRegion BLUE_LEFT_TRENCH_OUTER_SKIRT =
      cadRect(-4229.1, -3060.7, 2751.1, 2755.9);
  private static final RectRegion BLUE_LEFT_TRENCH_INNER_SKIRT =
      cadRect(-4229.1, -3060.7, 2451.1, 2455.9);
  private static final RectRegion BLUE_LEFT_TRENCH_LEFT_SKIRT =
      cadRect(-4241.8, -3746.5, 2455.9, 2751.1);
  private static final RectRegion BLUE_LEFT_TRENCH_RIGHT_SKIRT =
      cadRect(-3543.3, -3048.0, 2455.9, 2751.1);
  private static final RectRegion BLUE_RIGHT_TRENCH_OUTER_SKIRT =
      cadRect(-4229.1, -3060.7, -2755.9, -2751.1);
  private static final RectRegion BLUE_RIGHT_TRENCH_INNER_SKIRT =
      cadRect(-4229.1, -3060.7, -2455.9, -2451.1);
  private static final RectRegion BLUE_RIGHT_TRENCH_LEFT_SKIRT =
      cadRect(-4241.8, -3746.5, -2751.1, -2455.9);
  private static final RectRegion BLUE_RIGHT_TRENCH_RIGHT_SKIRT =
      cadRect(-3543.3, -3048.0, -2751.1, -2455.9);
  private static final RectRegion RED_LEFT_TRENCH_OUTER_SKIRT =
      cadRect(3060.7, 4229.1, 2751.1, 2755.9);
  private static final RectRegion RED_LEFT_TRENCH_INNER_SKIRT =
      cadRect(3060.7, 4229.1, 2451.1, 2455.9);
  private static final RectRegion RED_LEFT_TRENCH_LEFT_SKIRT =
      cadRect(3048.0, 3543.3, 2455.9, 2751.1);
  private static final RectRegion RED_LEFT_TRENCH_RIGHT_SKIRT =
      cadRect(3746.5, 4241.8, 2455.9, 2751.1);
  private static final RectRegion RED_RIGHT_TRENCH_OUTER_SKIRT =
      cadRect(3060.7, 4229.1, -2755.9, -2751.1);
  private static final RectRegion RED_RIGHT_TRENCH_INNER_SKIRT =
      cadRect(3060.7, 4229.1, -2455.9, -2451.1);
  private static final RectRegion RED_RIGHT_TRENCH_LEFT_SKIRT =
      cadRect(3048.0, 3543.3, -2751.1, -2455.9);
  private static final RectRegion RED_RIGHT_TRENCH_RIGHT_SKIRT =
      cadRect(3746.5, 4241.8, -2751.1, -2455.9);

  private static final double BUMP_HEIGHT_METERS = Units.inchesToMeters(6.513);
  private static final double BUMP_RAMP_FRACTION = 0.34;
  private static final double TRENCH_OPENING_HEIGHT_METERS = Units.inchesToMeters(22.25);

  private static final double TRENCH_EDGE_WALL_WIDTH_METERS = Units.inchesToMeters(7.655);
  private static final double TOWER_INNER_OPENING_WIDTH_METERS = Units.inchesToMeters(32.250);
  private static final double TOWER_UPRIGHT_OFFSET_METERS = Units.inchesToMeters(0.75);
  private static final double TOWER_UPRIGHT_HEIGHT_METERS = Units.inchesToMeters(72.1);

  private static final double TOWER_FRONT_FACE_X_METERS =
      cadXToField(-7125.9);
  private static final double RED_TOWER_FRONT_FACE_X_METERS =
      cadXToField(7125.9);
  private static final double TOWER_WIDTH_METERS =
      cadYSpanToField(336.6 - (-914.4));
  private static final double TOWER_DEPTH_METERS =
      cadXSpanToField(-7125.9 - (-8319.7));
  private static final double TOWER_SIDE_WIDTH_METERS =
      (TOWER_WIDTH_METERS - TOWER_INNER_OPENING_WIDTH_METERS) * 0.5;
  private static final double BLUE_TOWER_CENTER_Y_METERS =
      cadYToField((-914.4 + 336.6) * 0.5);
  private static final double RED_TOWER_CENTER_Y_METERS =
      cadYToField((-336.6 + 914.4) * 0.5);

  public static final Rebuilt2026FieldContactModel INSTANCE = new Rebuilt2026FieldContactModel();

  private Rebuilt2026FieldContactModel() {}

  @Override
  public TerrainSample sample(Pose2d robotPose) {
    Translation2d translation = robotPose.getTranslation();
    double z = bumpHeightMeters(translation);
    double[] gradient = terrainGradient(translation);

    double yaw = robotPose.getRotation().getRadians();
    double forwardSlope = (gradient[0] * Math.cos(yaw)) + (gradient[1] * Math.sin(yaw));
    double rightSlope = (-gradient[0] * Math.sin(yaw)) + (gradient[1] * Math.cos(yaw));

    double pitch = -Math.atan(forwardSlope);
    double roll = -Math.atan(rightSlope);
    Pose3d pose3d =
        new Pose3d(robotPose.getX(), robotPose.getY(), z, new Rotation3d(roll, pitch, yaw));
    return new TerrainSample(pose3d, roll, pitch, z);
  }

  @Override
  public TerrainContactSample sampleContact(Pose2d robotPose, ChassisFootprint chassisFootprint) {
    TerrainSample terrainSample = sample(robotPose);
    TerrainFeature feature = featureAt(robotPose.getTranslation());
    double overheadClearanceMeters = overheadClearanceMeters(feature);
    double underbodyClearanceMarginMeters =
        chassisFootprint.groundClearanceMeters() - terrainSample.heightMeters();
    double overheadClearanceMarginMeters =
        Double.isFinite(overheadClearanceMeters)
            ? overheadClearanceMeters - chassisFootprint.heightMeters()
            : Double.POSITIVE_INFINITY;
    boolean traversableSurface = traversableSurface(feature);
    boolean clearanceSatisfied = traversableSurface && overheadClearanceMarginMeters >= 0.0;
    return new TerrainContactSample(
        terrainSample,
        feature,
        overheadClearanceMeters,
        underbodyClearanceMarginMeters,
        overheadClearanceMarginMeters,
        traversableSurface,
        clearanceSatisfied);
  }

  @Override
  public FieldMarkerSample[] getFieldMarkers() {
    List<FieldMarkerSample> markers = new ArrayList<>();
    addHubMarkers(markers);
    addBumpMarkers(markers);
    addTrenchMarkers(markers);
    addTrenchEdgeMarkers(markers);
    addTowerMarkers(markers);
    return markers.toArray(FieldMarkerSample[]::new);
  }

  public TerrainFeature featureAt(Translation2d position) {
    if (contains(BLUE_HUB_BOUNDS, position)) {
      return TerrainFeature.BLUE_HUB;
    }
    if (contains(RED_HUB_BOUNDS, position)) {
      return TerrainFeature.RED_HUB;
    }
    if (inBlueTower(position)) {
      return TerrainFeature.BLUE_TOWER;
    }
    if (inRedTower(position)) {
      return TerrainFeature.RED_TOWER;
    }
    if (contains(BLUE_LEFT_TRENCH_BLOCK_BOUNDS, position)) {
      return TerrainFeature.BLUE_LEFT_TRENCH_EDGE;
    }
    if (contains(BLUE_RIGHT_TRENCH_BLOCK_BOUNDS, position)) {
      return TerrainFeature.BLUE_RIGHT_TRENCH_EDGE;
    }
    if (contains(RED_LEFT_TRENCH_BLOCK_BOUNDS, position)) {
      return TerrainFeature.RED_LEFT_TRENCH_EDGE;
    }
    if (contains(RED_RIGHT_TRENCH_BLOCK_BOUNDS, position)) {
      return TerrainFeature.RED_RIGHT_TRENCH_EDGE;
    }
    if (contains(BLUE_LEFT_BUMP_BOUNDS, position)) {
      return TerrainFeature.BLUE_LEFT_BUMP;
    }
    if (contains(BLUE_RIGHT_BUMP_BOUNDS, position)) {
      return TerrainFeature.BLUE_RIGHT_BUMP;
    }
    if (contains(RED_LEFT_BUMP_BOUNDS, position)) {
      return TerrainFeature.RED_LEFT_BUMP;
    }
    if (contains(RED_RIGHT_BUMP_BOUNDS, position)) {
      return TerrainFeature.RED_RIGHT_BUMP;
    }
    if (contains(BLUE_TRENCH_OPEN_BOUNDS, position)) {
      return position.getY() >= hubCenterYMeters()
          ? TerrainFeature.BLUE_LEFT_TRENCH
          : TerrainFeature.BLUE_RIGHT_TRENCH;
    }
    if (contains(RED_TRENCH_OPEN_BOUNDS, position)) {
      return position.getY() >= hubCenterYMeters()
          ? TerrainFeature.RED_LEFT_TRENCH
          : TerrainFeature.RED_RIGHT_TRENCH;
    }
    return TerrainFeature.FLAT;
  }

  public double bumpHeightMeters(Translation2d fieldTranslation) {
    return Math.max(
        Math.max(
            profileHeight(fieldTranslation, BLUE_LEFT_BUMP_BOUNDS),
            profileHeight(fieldTranslation, BLUE_RIGHT_BUMP_BOUNDS)),
        Math.max(
            profileHeight(fieldTranslation, RED_LEFT_BUMP_BOUNDS),
            profileHeight(fieldTranslation, RED_RIGHT_BUMP_BOUNDS)));
  }

  private double[] terrainGradient(Translation2d point) {
    double dzdx =
        (bumpHeightMeters(point.plus(new Translation2d(GRADIENT_SAMPLE_METERS, 0.0)))
                - bumpHeightMeters(point.minus(new Translation2d(GRADIENT_SAMPLE_METERS, 0.0))))
            / (2.0 * GRADIENT_SAMPLE_METERS);
    double dzdy =
        (bumpHeightMeters(point.plus(new Translation2d(0.0, GRADIENT_SAMPLE_METERS)))
                - bumpHeightMeters(point.minus(new Translation2d(0.0, GRADIENT_SAMPLE_METERS))))
            / (2.0 * GRADIENT_SAMPLE_METERS);
    return new double[] {dzdx, dzdy};
  }

  private double profileHeight(Translation2d point, RectRegion bounds) {
    if (!contains(bounds, point)) {
      return 0.0;
    }
    double progress = normalizedProgress(bounds.minX, bounds.maxX, point.getX());
    double rampFraction = Math.min(BUMP_RAMP_FRACTION, 0.5);
    if (progress <= rampFraction) {
      return BUMP_HEIGHT_METERS * (progress / rampFraction);
    }
    if (progress >= 1.0 - rampFraction) {
      return BUMP_HEIGHT_METERS * ((1.0 - progress) / rampFraction);
    }
    return BUMP_HEIGHT_METERS;
  }

  private double overheadClearanceMeters(TerrainFeature feature) {
    return switch (feature) {
      case BLUE_LEFT_TRENCH, BLUE_RIGHT_TRENCH, RED_LEFT_TRENCH, RED_RIGHT_TRENCH ->
          TRENCH_OPENING_HEIGHT_METERS;
      default -> Double.POSITIVE_INFINITY;
    };
  }

  private boolean traversableSurface(TerrainFeature feature) {
    return switch (feature) {
      case BLUE_HUB,
          RED_HUB,
          BLUE_TOWER,
          RED_TOWER,
          BLUE_LEFT_TRENCH_EDGE,
          BLUE_RIGHT_TRENCH_EDGE,
          RED_LEFT_TRENCH_EDGE,
          RED_RIGHT_TRENCH_EDGE -> false;
      default -> true;
    };
  }

  private static boolean contains(RectRegion region, Translation2d point) {
    return point.getX() >= region.minX
        && point.getX() <= region.maxX
        && point.getY() >= region.minY
        && point.getY() <= region.maxY;
  }

  private static double normalizedProgress(double min, double max, double value) {
    double span = Math.max(max - min, 1e-9);
    return Math.max(0.0, Math.min(1.0, (value - min) / span));
  }

  private static double hubCenterXBlue() {
    return BLUE_HUB_BOUNDS.centerX();
  }

  private static double hubCenterXRed() {
    return RED_HUB_BOUNDS.centerX();
  }

  private static boolean inBlueTower(Translation2d position) {
    return contains(blueTowerLeftWall(), position)
        || contains(blueTowerRightWall(), position)
        || contains(blueTowerBackWall(), position);
  }

  private static boolean inRedTower(Translation2d position) {
    return contains(redTowerLeftWall(), position)
        || contains(redTowerRightWall(), position)
        || contains(redTowerBackWall(), position);
  }

  private static RectRegion blueTowerLeftWall() {
    double towerCenterX = TOWER_FRONT_FACE_X_METERS - (TOWER_DEPTH_METERS * 0.5);
    double centerY =
        BLUE_TOWER_CENTER_Y_METERS
            + (TOWER_INNER_OPENING_WIDTH_METERS * 0.5)
            + (TOWER_SIDE_WIDTH_METERS * 0.5);
    return centeredRect(towerCenterX, centerY, TOWER_DEPTH_METERS, TOWER_SIDE_WIDTH_METERS);
  }

  private static RectRegion blueTowerRightWall() {
    double towerCenterX = TOWER_FRONT_FACE_X_METERS - (TOWER_DEPTH_METERS * 0.5);
    double centerY =
        BLUE_TOWER_CENTER_Y_METERS
            - (TOWER_INNER_OPENING_WIDTH_METERS * 0.5)
            - (TOWER_SIDE_WIDTH_METERS * 0.5);
    return centeredRect(towerCenterX, centerY, TOWER_DEPTH_METERS, TOWER_SIDE_WIDTH_METERS);
  }

  private static RectRegion blueTowerBackWall() {
    return centeredRect(
        TOWER_FRONT_FACE_X_METERS - TOWER_DEPTH_METERS,
        BLUE_TOWER_CENTER_Y_METERS,
        Units.inchesToMeters(2.0),
        TOWER_WIDTH_METERS);
  }

  private static RectRegion redTowerLeftWall() {
    double towerCenterX = RED_TOWER_FRONT_FACE_X_METERS + (TOWER_DEPTH_METERS * 0.5);
    double centerY =
        RED_TOWER_CENTER_Y_METERS
            + (TOWER_INNER_OPENING_WIDTH_METERS * 0.5)
            + (TOWER_SIDE_WIDTH_METERS * 0.5);
    return centeredRect(towerCenterX, centerY, TOWER_DEPTH_METERS, TOWER_SIDE_WIDTH_METERS);
  }

  private static RectRegion redTowerRightWall() {
    double towerCenterX = RED_TOWER_FRONT_FACE_X_METERS + (TOWER_DEPTH_METERS * 0.5);
    double centerY =
        RED_TOWER_CENTER_Y_METERS
            - (TOWER_INNER_OPENING_WIDTH_METERS * 0.5)
            - (TOWER_SIDE_WIDTH_METERS * 0.5);
    return centeredRect(towerCenterX, centerY, TOWER_DEPTH_METERS, TOWER_SIDE_WIDTH_METERS);
  }

  private static RectRegion redTowerBackWall() {
    return centeredRect(
        RED_TOWER_FRONT_FACE_X_METERS + TOWER_DEPTH_METERS,
        RED_TOWER_CENTER_Y_METERS,
        Units.inchesToMeters(2.0),
        TOWER_WIDTH_METERS);
  }

  private static RectRegion centeredRect(
      double centerX, double centerY, double widthMeters, double heightMeters) {
    return new RectRegion(
        centerX - (widthMeters * 0.5),
        centerX + (widthMeters * 0.5),
        centerY - (heightMeters * 0.5),
        centerY + (heightMeters * 0.5));
  }

  public static double fieldLengthMeters() {
    return FIELD_LENGTH_METERS;
  }

  public static double fieldWidthMeters() {
    return FIELD_WIDTH_METERS;
  }

  public static double hubCenterXBlueMeters() {
    return hubCenterXBlue();
  }

  public static double hubCenterXRedMeters() {
    return hubCenterXRed();
  }

  public static double hubCenterYMeters() {
    return BLUE_HUB_BOUNDS.centerY();
  }

  public static double hubCollisionSizeMeters() {
    return (BLUE_HUB_BOUNDS.maxX - BLUE_HUB_BOUNDS.minX + BLUE_HUB_BOUNDS.maxY - BLUE_HUB_BOUNDS.minY)
        * 0.25;
  }

  public static double hubCollisionWidthMeters() {
    return BLUE_HUB_BOUNDS.maxX - BLUE_HUB_BOUNDS.minX;
  }

  public static double hubCollisionHeightMeters() {
    return BLUE_HUB_BOUNDS.maxY - BLUE_HUB_BOUNDS.minY;
  }

  public static double blueTowerFrontFaceXMeters() {
    return TOWER_FRONT_FACE_X_METERS;
  }

  public static double redTowerFrontFaceXMeters() {
    return RED_TOWER_FRONT_FACE_X_METERS;
  }

  public static double towerWidthMeters() {
    return TOWER_WIDTH_METERS;
  }

  public static double towerDepthMeters() {
    return TOWER_DEPTH_METERS;
  }

  public static double blueTowerCenterYMeters() {
    return BLUE_TOWER_CENTER_Y_METERS;
  }

  public static double redTowerCenterYMeters() {
    return RED_TOWER_CENTER_Y_METERS;
  }

  public static double blueTrenchCenterXMeters() {
    return BLUE_TRENCH_BOUNDS.centerX();
  }

  public static double redTrenchCenterXMeters() {
    return RED_TRENCH_BOUNDS.centerX();
  }

  public static double trenchDepthMeters() {
    return BLUE_TRENCH_BOUNDS.maxX - BLUE_TRENCH_BOUNDS.minX;
  }

  public static double blueLeftTrenchBlockCenterYMeters() {
    return BLUE_LEFT_TRENCH_BLOCK_BOUNDS.centerY();
  }

  public static double blueRightTrenchBlockCenterYMeters() {
    return BLUE_RIGHT_TRENCH_BLOCK_BOUNDS.centerY();
  }

  public static double redLeftTrenchBlockCenterYMeters() {
    return RED_LEFT_TRENCH_BLOCK_BOUNDS.centerY();
  }

  public static double redRightTrenchBlockCenterYMeters() {
    return RED_RIGHT_TRENCH_BLOCK_BOUNDS.centerY();
  }

  public static double leftTrenchBlockWidthMeters() {
    return BLUE_LEFT_TRENCH_BLOCK_BOUNDS.maxY - BLUE_LEFT_TRENCH_BLOCK_BOUNDS.minY;
  }

  public static double rightTrenchBlockWidthMeters() {
    return BLUE_RIGHT_TRENCH_BLOCK_BOUNDS.maxY - BLUE_RIGHT_TRENCH_BLOCK_BOUNDS.minY;
  }

  public static RectRegion[] blueLeftTrenchSkirts() {
    return new RectRegion[] {
      BLUE_LEFT_TRENCH_LEFT_SKIRT,
      BLUE_LEFT_TRENCH_RIGHT_SKIRT,
      BLUE_LEFT_TRENCH_INNER_SKIRT,
      BLUE_LEFT_TRENCH_OUTER_SKIRT
    };
  }

  public static RectRegion[] blueRightTrenchSkirts() {
    return new RectRegion[] {
      BLUE_RIGHT_TRENCH_LEFT_SKIRT,
      BLUE_RIGHT_TRENCH_RIGHT_SKIRT,
      BLUE_RIGHT_TRENCH_INNER_SKIRT,
      BLUE_RIGHT_TRENCH_OUTER_SKIRT
    };
  }

  public static RectRegion[] redLeftTrenchSkirts() {
    return new RectRegion[] {
      RED_LEFT_TRENCH_LEFT_SKIRT,
      RED_LEFT_TRENCH_RIGHT_SKIRT,
      RED_LEFT_TRENCH_INNER_SKIRT,
      RED_LEFT_TRENCH_OUTER_SKIRT
    };
  }

  public static RectRegion[] redRightTrenchSkirts() {
    return new RectRegion[] {
      RED_RIGHT_TRENCH_LEFT_SKIRT,
      RED_RIGHT_TRENCH_RIGHT_SKIRT,
      RED_RIGHT_TRENCH_INNER_SKIRT,
      RED_RIGHT_TRENCH_OUTER_SKIRT
    };
  }

  private static void addHubMarkers(List<FieldMarkerSample> markers) {
    addMarker(markers, "blue-hub", BLUE_HUB_BOUNDS.center3d(BUMP_HEIGHT_METERS * 0.5));
    addMarker(markers, "red-hub", RED_HUB_BOUNDS.center3d(BUMP_HEIGHT_METERS * 0.5));
  }

  private static void addBumpMarkers(List<FieldMarkerSample> markers) {
    addMarker(markers, "blue-left-bump", BLUE_LEFT_BUMP_BOUNDS.center3d(BUMP_HEIGHT_METERS * 0.5));
    addMarker(
        markers, "blue-right-bump", BLUE_RIGHT_BUMP_BOUNDS.center3d(BUMP_HEIGHT_METERS * 0.5));
    addMarker(markers, "red-left-bump", RED_LEFT_BUMP_BOUNDS.center3d(BUMP_HEIGHT_METERS * 0.5));
    addMarker(markers, "red-right-bump", RED_RIGHT_BUMP_BOUNDS.center3d(BUMP_HEIGHT_METERS * 0.5));
  }

  private static void addTrenchMarkers(List<FieldMarkerSample> markers) {
    addMarker(
        markers,
        "blue-left-trench-open",
        BLUE_TRENCH_OPEN_BOUNDS.center3d(TRENCH_OPENING_HEIGHT_METERS));
    addMarker(
        markers,
        "blue-right-trench-open",
        BLUE_TRENCH_OPEN_BOUNDS.center3d(TRENCH_OPENING_HEIGHT_METERS));
    addMarker(
        markers,
        "red-left-trench-open",
        RED_TRENCH_OPEN_BOUNDS.center3d(TRENCH_OPENING_HEIGHT_METERS));
    addMarker(
        markers,
        "red-right-trench-open",
        RED_TRENCH_OPEN_BOUNDS.center3d(TRENCH_OPENING_HEIGHT_METERS));
  }

  private static void addTrenchEdgeMarkers(List<FieldMarkerSample> markers) {
    addMarker(
        markers,
        "blue-left-trench-edge",
        BLUE_LEFT_TRENCH_BLOCK_BOUNDS.center3d(TRENCH_OPENING_HEIGHT_METERS * 0.5));
    addMarker(
        markers,
        "blue-right-trench-edge",
        BLUE_RIGHT_TRENCH_BLOCK_BOUNDS.center3d(TRENCH_OPENING_HEIGHT_METERS * 0.5));
    addMarker(
        markers,
        "red-left-trench-edge",
        RED_LEFT_TRENCH_BLOCK_BOUNDS.center3d(TRENCH_OPENING_HEIGHT_METERS * 0.5));
    addMarker(
        markers,
        "red-right-trench-edge",
        RED_RIGHT_TRENCH_BLOCK_BOUNDS.center3d(TRENCH_OPENING_HEIGHT_METERS * 0.5));
  }

  private static void addTowerMarkers(List<FieldMarkerSample> markers) {
    double blueLeftUprightY = BLUE_TOWER_CENTER_Y_METERS + TOWER_INNER_OPENING_WIDTH_METERS * 0.5 + TOWER_UPRIGHT_OFFSET_METERS;
    double blueRightUprightY = BLUE_TOWER_CENTER_Y_METERS - TOWER_INNER_OPENING_WIDTH_METERS * 0.5 - TOWER_UPRIGHT_OFFSET_METERS;
    double redLeftUprightY = RED_TOWER_CENTER_Y_METERS + TOWER_INNER_OPENING_WIDTH_METERS * 0.5 + TOWER_UPRIGHT_OFFSET_METERS;
    double redRightUprightY = RED_TOWER_CENTER_Y_METERS - TOWER_INNER_OPENING_WIDTH_METERS * 0.5 - TOWER_UPRIGHT_OFFSET_METERS;

    addMarker(markers, "blue-tower-left-upright", new Translation3d(TOWER_FRONT_FACE_X_METERS, blueLeftUprightY, TOWER_UPRIGHT_HEIGHT_METERS * 0.5));
    addMarker(markers, "blue-tower-right-upright", new Translation3d(TOWER_FRONT_FACE_X_METERS, blueRightUprightY, TOWER_UPRIGHT_HEIGHT_METERS * 0.5));
    addMarker(markers, "red-tower-left-upright", new Translation3d(FIELD_LENGTH_METERS - TOWER_FRONT_FACE_X_METERS, redLeftUprightY, TOWER_UPRIGHT_HEIGHT_METERS * 0.5));
    addMarker(markers, "red-tower-right-upright", new Translation3d(FIELD_LENGTH_METERS - TOWER_FRONT_FACE_X_METERS, redRightUprightY, TOWER_UPRIGHT_HEIGHT_METERS * 0.5));
    addMarker(markers, "blue-tower-back-wall", blueTowerBackWall().center3d(TOWER_UPRIGHT_HEIGHT_METERS * 0.5));
    addMarker(markers, "red-tower-back-wall", redTowerBackWall().center3d(TOWER_UPRIGHT_HEIGHT_METERS * 0.5));
  }

  private static void addMarker(List<FieldMarkerSample> markers, String id, Translation3d translation3d) {
    markers.add(new FieldMarkerSample(id, new Pose3d(translation3d, new Rotation3d())));
  }

  private static RectRegion cadRect(
      double cadMinXMm, double cadMaxXMm, double cadMinYMm, double cadMaxYMm) {
    return new RectRegion(
        cadXToField(cadMinXMm),
        cadXToField(cadMaxXMm),
        cadYToField(cadMinYMm),
        cadYToField(cadMaxYMm));
  }

  private static double cadXToField(double cadXMillimeters) {
    return (cadXMillimeters / 1000.0) + CAD_FIELD_ORIGIN_X_METERS;
  }

  private static double cadYToField(double cadYMillimeters) {
    return (cadYMillimeters / 1000.0) + CAD_FIELD_ORIGIN_Y_METERS;
  }

  private static double cadXSpanToField(double cadSpanMillimeters) {
    return cadSpanMillimeters / 1000.0;
  }

  private static double cadYSpanToField(double cadSpanMillimeters) {
    return cadSpanMillimeters / 1000.0;
  }

  public record RectRegion(double minX, double maxX, double minY, double maxY) {
    Translation3d center3d(double z) {
      return new Translation3d((minX + maxX) * 0.5, (minY + maxY) * 0.5, z);
    }

    double centerX() {
      return (minX + maxX) * 0.5;
    }

    double centerY() {
      return (minY + maxY) * 0.5;
    }
  }
}
