package org.griffins1884.griffinsim.frc.scenario;

import java.util.List;

public record HolonomicPoseFollowerSpec(
    List<PoseWaypoint2d> waypoints,
    double initialXMeters,
    double initialYMeters,
    double initialThetaRadians,
    double kPTranslation,
    double kPRotation,
    double maxPwm,
    double metersPerTickAtFullPwm,
    double radiansPerTickAtFullPwm,
    double positionToleranceMeters,
    double thetaToleranceRadians) {
  public HolonomicPoseFollowerSpec {
    waypoints = List.copyOf(waypoints == null ? List.of() : waypoints);
    if (waypoints.size() < 2) {
      throw new IllegalArgumentException("waypoints must contain at least 2 entries");
    }
    if (kPTranslation <= 0.0 || kPRotation <= 0.0) {
      throw new IllegalArgumentException("controller gains must be positive");
    }
    if (maxPwm < 0.0) {
      throw new IllegalArgumentException("maxPwm must be non-negative");
    }
    if (metersPerTickAtFullPwm <= 0.0 || radiansPerTickAtFullPwm <= 0.0) {
      throw new IllegalArgumentException("tick response scales must be positive");
    }
    if (positionToleranceMeters < 0.0 || thetaToleranceRadians < 0.0) {
      throw new IllegalArgumentException("tolerances must be non-negative");
    }
  }
}
