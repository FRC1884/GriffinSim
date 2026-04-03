package org.griffins1884.griffinsim.frc.scenario;

public record PoseWaypoint2d(int tick, double xMeters, double yMeters, double thetaRadians, double pwmMagnitude) {
  public PoseWaypoint2d {
    if (tick < 0) {
      throw new IllegalArgumentException("tick must be non-negative");
    }
    if (pwmMagnitude < 0.0) {
      throw new IllegalArgumentException("pwmMagnitude must be non-negative");
    }
  }

  public PoseWaypoint2d(int tick, double xMeters, double yMeters, double thetaRadians) {
    this(tick, xMeters, yMeters, thetaRadians, 0.5);
  }
}
