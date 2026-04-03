package org.griffins1884.griffinsim.physics.seasons.rebuilt2026;

public final class Rebuilt2026FieldConstants {
  public static final double FIELD_LENGTH_METERS = inchesToMeters(650.12);
  public static final double FIELD_WIDTH_METERS = inchesToMeters(316.64);

  public static final double HUB_CENTER_X_BLUE_METERS = inchesToMeters(181.56);
  public static final double HUB_CENTER_X_RED_METERS = inchesToMeters(468.56);
  public static final double HUB_CENTER_Y_METERS = inchesToMeters(158.32);
  public static final double HUB_WIDTH_METERS = inchesToMeters(58.41);
  public static final double HUB_DEPTH_METERS = inchesToMeters(47.00);
  public static final double HUB_HEIGHT_METERS = inchesToMeters(47.0);

  public static final double TOWER_OPENING_WIDTH_METERS = inchesToMeters(32.25);
  public static final double TOWER_WIDTH_METERS = inchesToMeters(53.0);
  public static final double TOWER_DEPTH_METERS = inchesToMeters(24.0);
  public static final double TOWER_FRONT_FACE_X_BLUE_METERS = inchesToMeters(62.0);
  public static final double TOWER_FRONT_FACE_X_RED_METERS = FIELD_LENGTH_METERS - TOWER_FRONT_FACE_X_BLUE_METERS;
  public static final double TOWER_CENTER_Y_BLUE_METERS = inchesToMeters(67.0);
  public static final double TOWER_CENTER_Y_RED_METERS = FIELD_WIDTH_METERS - TOWER_CENTER_Y_BLUE_METERS;

  public static final double TRENCH_FOOTPRINT_DEPTH_METERS = inchesToMeters(24.97);
  public static final double TRENCH_FOOTPRINT_WIDTH_METERS = inchesToMeters(47.00);
  public static final double TRENCH_OPENING_HEIGHT_METERS = inchesToMeters(22.25);
  public static final double TRENCH_CENTER_X_BLUE_METERS = inchesToMeters(111.0);
  public static final double TRENCH_CENTER_X_RED_METERS = FIELD_LENGTH_METERS - TRENCH_CENTER_X_BLUE_METERS;
  public static final double TRENCH_UPPER_CENTER_Y_METERS = inchesToMeters(261.0);
  public static final double TRENCH_LOWER_CENTER_Y_METERS = FIELD_WIDTH_METERS - TRENCH_UPPER_CENTER_Y_METERS;
  public static final double TRENCH_EDGE_THICKNESS_METERS = inchesToMeters(2.0);

  public static final double WALL_THICKNESS_METERS = inchesToMeters(2.0);

  private Rebuilt2026FieldConstants() {}

  public static double inchesToMeters(double inches) {
    return inches * 0.0254;
  }
}
