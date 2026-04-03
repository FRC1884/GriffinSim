package org.griffins1884.griffinsim.frc.scenario;

import java.util.List;

public record PathFollower2dSpec(
    List<Waypoint2d> waypoints,
    double initialXMeters,
    double initialYMeters,
    double kP,
    double maxPwm,
    double metersPerTickAtFullPwm,
    double positionToleranceMeters) {
  public PathFollower2dSpec {
    waypoints = List.copyOf(waypoints == null ? List.of() : waypoints);
    if (waypoints.size() < 2) {
      throw new IllegalArgumentException("waypoints must contain at least 2 entries");
    }
    if (kP <= 0.0) {
      throw new IllegalArgumentException("kP must be positive");
    }
    if (maxPwm < 0.0) {
      throw new IllegalArgumentException("maxPwm must be non-negative");
    }
    if (metersPerTickAtFullPwm <= 0.0) {
      throw new IllegalArgumentException("metersPerTickAtFullPwm must be positive");
    }
    if (positionToleranceMeters < 0.0) {
      throw new IllegalArgumentException("positionToleranceMeters must be non-negative");
    }
  }
}
