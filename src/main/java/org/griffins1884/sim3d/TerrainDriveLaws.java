package org.griffins1884.sim3d;

/** Shared scaling laws that translate terrain contact and wheel loading into drivetrain authority. */
public final class TerrainDriveLaws {
  private TerrainDriveLaws() {}

  public static double driveAuthorityScale(
      SwerveTractionState tractionState, SwerveCorner corner, TerrainContactSample contactSample) {
    if (tractionState == null || corner == null) {
      return 1.0;
    }

    double normalizedLoad = clamp(tractionState.forCorner(corner).normalizedLoad(), 0.55, 1.35);
    double driveScale = clamp(0.35 + (0.65 * normalizedLoad), 0.45, 1.25);
    if (contactSample == null) {
      return driveScale;
    }
    if (!contactSample.traversableSurface()) {
      return 0.0;
    }
    if (!contactSample.clearanceSatisfied()) {
      driveScale *= 0.2;
    }
    return driveScale;
  }

  public static double steerAuthorityScale(
      SwerveTractionState tractionState, SwerveCorner corner, TerrainContactSample contactSample) {
    if (tractionState == null || corner == null) {
      return contactSample != null && !contactSample.traversableSurface() ? 0.0 : 1.0;
    }

    double normalizedLoad = clamp(tractionState.forCorner(corner).normalizedLoad(), 0.7, 1.25);
    double steerScale = clamp(0.75 + (0.25 * normalizedLoad), 0.75, 1.1);
    if (contactSample == null) {
      return steerScale;
    }
    if (!contactSample.traversableSurface()) {
      return 0.0;
    }
    if (!contactSample.clearanceSatisfied()) {
      steerScale *= 0.35;
    }
    return steerScale;
  }

  private static double clamp(double value, double min, double max) {
    return Math.max(min, Math.min(max, value));
  }
}
