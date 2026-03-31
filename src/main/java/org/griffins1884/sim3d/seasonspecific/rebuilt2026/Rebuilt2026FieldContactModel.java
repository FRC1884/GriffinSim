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

  private static final double HUB_NEAR_FACE_X_METERS = 4.007866;
  private static final double OPP_HUB_NEAR_FACE_X_METERS = 11.2978438;
  private static final double HUB_WIDTH_METERS = Units.inchesToMeters(47.0);
  private static final double HUB_CENTER_Y_METERS = 4.0213534;

  private static final double BUMP_SPAN_Y_METERS = Units.inchesToMeters(73.0);
  private static final double BUMP_DEPTH_X_METERS = Units.inchesToMeters(44.4);
  private static final double BUMP_HEIGHT_METERS = Units.inchesToMeters(6.513);

  private static final double TRENCH_DEPTH_X_METERS = Units.inchesToMeters(47.0);
  private static final double TRENCH_OPENING_WIDTH_METERS = Units.inchesToMeters(50.34);
  private static final double TRENCH_OPENING_HEIGHT_METERS = Units.inchesToMeters(22.25);
  private static final double TOWER_FRONT_FACE_X_METERS = Units.inchesToMeters(53.51);
  private static final double TOWER_INNER_OPENING_WIDTH_METERS = Units.inchesToMeters(32.250);
  private static final double TOWER_UPRIGHT_OFFSET_METERS = Units.inchesToMeters(0.75);
  private static final double TOWER_UPRIGHT_HEIGHT_METERS = Units.inchesToMeters(72.1);
  private static final double BLUE_TOWER_CENTER_Y_METERS = 3.7301932;
  private static final double RED_TOWER_CENTER_Y_METERS = 4.3124882;

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

    double pitch = Math.atan(forwardSlope);
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
    boolean traversableSurface = true;
    boolean clearanceSatisfied = overheadClearanceMarginMeters >= 0.0;
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
    addBumpMarkers(markers);
    addTrenchMarkers(markers);
    addTowerMarkers(markers);
    return markers.toArray(FieldMarkerSample[]::new);
  }

  public TerrainFeature featureAt(Translation2d position) {
    if (contains(bumpBoundsBlueLeft(), position)) {
      return TerrainFeature.BLUE_LEFT_BUMP;
    }
    if (contains(bumpBoundsBlueRight(), position)) {
      return TerrainFeature.BLUE_RIGHT_BUMP;
    }
    if (contains(bumpBoundsRedLeft(), position)) {
      return TerrainFeature.RED_LEFT_BUMP;
    }
    if (contains(bumpBoundsRedRight(), position)) {
      return TerrainFeature.RED_RIGHT_BUMP;
    }
    if (contains(trenchBoundsBlueLeft(), position)) {
      return TerrainFeature.BLUE_LEFT_TRENCH;
    }
    if (contains(trenchBoundsBlueRight(), position)) {
      return TerrainFeature.BLUE_RIGHT_TRENCH;
    }
    if (contains(trenchBoundsRedLeft(), position)) {
      return TerrainFeature.RED_LEFT_TRENCH;
    }
    if (contains(trenchBoundsRedRight(), position)) {
      return TerrainFeature.RED_RIGHT_TRENCH;
    }
    return TerrainFeature.FLAT;
  }

  public double bumpHeightMeters(Translation2d fieldTranslation) {
    return Math.max(
        Math.max(
            profileHeight(fieldTranslation, bumpBoundsBlueLeft()),
            profileHeight(fieldTranslation, bumpBoundsBlueRight())),
        Math.max(
            profileHeight(fieldTranslation, bumpBoundsRedLeft()),
            profileHeight(fieldTranslation, bumpBoundsRedRight())));
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
    double progress = normalizedProgress(bounds.minY, bounds.maxY, point.getY());
    return Math.sin(Math.PI * progress) * BUMP_HEIGHT_METERS;
  }

  private double overheadClearanceMeters(TerrainFeature feature) {
    return switch (feature) {
      case BLUE_LEFT_TRENCH, BLUE_RIGHT_TRENCH, RED_LEFT_TRENCH, RED_RIGHT_TRENCH ->
          TRENCH_OPENING_HEIGHT_METERS;
      default -> Double.POSITIVE_INFINITY;
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

  private static RectRegion bumpBoundsBlueLeft() {
    return new RectRegion(
        hubCenterXBlue() - BUMP_DEPTH_X_METERS * 0.5,
        hubCenterXBlue() + BUMP_DEPTH_X_METERS * 0.5,
        HUB_CENTER_Y_METERS + HUB_WIDTH_METERS * 0.5,
        HUB_CENTER_Y_METERS + HUB_WIDTH_METERS * 0.5 + BUMP_SPAN_Y_METERS);
  }

  private static RectRegion bumpBoundsBlueRight() {
    return new RectRegion(
        hubCenterXBlue() - BUMP_DEPTH_X_METERS * 0.5,
        hubCenterXBlue() + BUMP_DEPTH_X_METERS * 0.5,
        HUB_CENTER_Y_METERS - HUB_WIDTH_METERS * 0.5 - BUMP_SPAN_Y_METERS,
        HUB_CENTER_Y_METERS - HUB_WIDTH_METERS * 0.5);
  }

  private static RectRegion bumpBoundsRedLeft() {
    return new RectRegion(
        hubCenterXRed() - BUMP_DEPTH_X_METERS * 0.5,
        hubCenterXRed() + BUMP_DEPTH_X_METERS * 0.5,
        HUB_CENTER_Y_METERS + HUB_WIDTH_METERS * 0.5,
        HUB_CENTER_Y_METERS + HUB_WIDTH_METERS * 0.5 + BUMP_SPAN_Y_METERS);
  }

  private static RectRegion bumpBoundsRedRight() {
    return new RectRegion(
        hubCenterXRed() - BUMP_DEPTH_X_METERS * 0.5,
        hubCenterXRed() + BUMP_DEPTH_X_METERS * 0.5,
        HUB_CENTER_Y_METERS - HUB_WIDTH_METERS * 0.5 - BUMP_SPAN_Y_METERS,
        HUB_CENTER_Y_METERS - HUB_WIDTH_METERS * 0.5);
  }

  private static RectRegion trenchBoundsBlueLeft() {
    return new RectRegion(
        hubCenterXBlue() - TRENCH_DEPTH_X_METERS * 0.5,
        hubCenterXBlue() + TRENCH_DEPTH_X_METERS * 0.5,
        FIELD_WIDTH_METERS - TRENCH_OPENING_WIDTH_METERS,
        FIELD_WIDTH_METERS);
  }

  private static RectRegion trenchBoundsBlueRight() {
    return new RectRegion(
        hubCenterXBlue() - TRENCH_DEPTH_X_METERS * 0.5,
        hubCenterXBlue() + TRENCH_DEPTH_X_METERS * 0.5,
        0.0,
        TRENCH_OPENING_WIDTH_METERS);
  }

  private static RectRegion trenchBoundsRedLeft() {
    return new RectRegion(
        hubCenterXRed() - TRENCH_DEPTH_X_METERS * 0.5,
        hubCenterXRed() + TRENCH_DEPTH_X_METERS * 0.5,
        FIELD_WIDTH_METERS - TRENCH_OPENING_WIDTH_METERS,
        FIELD_WIDTH_METERS);
  }

  private static RectRegion trenchBoundsRedRight() {
    return new RectRegion(
        hubCenterXRed() - TRENCH_DEPTH_X_METERS * 0.5,
        hubCenterXRed() + TRENCH_DEPTH_X_METERS * 0.5,
        0.0,
        TRENCH_OPENING_WIDTH_METERS);
  }

  private static double hubCenterXBlue() {
    return HUB_NEAR_FACE_X_METERS + HUB_WIDTH_METERS * 0.5;
  }

  private static double hubCenterXRed() {
    return OPP_HUB_NEAR_FACE_X_METERS + HUB_WIDTH_METERS * 0.5;
  }

  public static double fieldLengthMeters() {
    return FIELD_LENGTH_METERS;
  }

  public static double fieldWidthMeters() {
    return FIELD_WIDTH_METERS;
  }

  private static void addBumpMarkers(List<FieldMarkerSample> markers) {
    addMarker(markers, "blue-left-bump", bumpBoundsBlueLeft().center3d(BUMP_HEIGHT_METERS * 0.5));
    addMarker(markers, "blue-right-bump", bumpBoundsBlueRight().center3d(BUMP_HEIGHT_METERS * 0.5));
    addMarker(markers, "red-left-bump", bumpBoundsRedLeft().center3d(BUMP_HEIGHT_METERS * 0.5));
    addMarker(markers, "red-right-bump", bumpBoundsRedRight().center3d(BUMP_HEIGHT_METERS * 0.5));
  }

  private static void addTrenchMarkers(List<FieldMarkerSample> markers) {
    addMarker(
        markers,
        "blue-left-trench-open",
        trenchBoundsBlueLeft().center3d(TRENCH_OPENING_HEIGHT_METERS));
    addMarker(
        markers,
        "blue-right-trench-open",
        trenchBoundsBlueRight().center3d(TRENCH_OPENING_HEIGHT_METERS));
    addMarker(
        markers,
        "red-left-trench-open",
        trenchBoundsRedLeft().center3d(TRENCH_OPENING_HEIGHT_METERS));
    addMarker(
        markers,
        "red-right-trench-open",
        trenchBoundsRedRight().center3d(TRENCH_OPENING_HEIGHT_METERS));
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
  }

  private static void addMarker(List<FieldMarkerSample> markers, String id, Translation3d translation3d) {
    markers.add(new FieldMarkerSample(id, new Pose3d(translation3d, new Rotation3d())));
  }

  private record RectRegion(double minX, double maxX, double minY, double maxY) {
    Translation3d center3d(double z) {
      return new Translation3d((minX + maxX) * 0.5, (minY + maxY) * 0.5, z);
    }
  }
}
