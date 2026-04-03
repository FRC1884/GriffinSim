package org.griffins1884.griffinsim.frc.scenario;

public record PoseSetpoint2d(
    int tick,
    double xMeters,
    double yMeters,
    double thetaRadians,
    double vxMetersPerTick,
    double vyMetersPerTick,
    double omegaRadiansPerTick,
    double pwmMagnitude) {
  public PoseSetpoint2d {
    if (tick < 0) {
      throw new IllegalArgumentException("tick must be non-negative");
    }
    if (pwmMagnitude < 0.0) {
      throw new IllegalArgumentException("pwmMagnitude must be non-negative");
    }
  }

  public PoseSetpoint2d(
      int tick,
      double xMeters,
      double yMeters,
      double thetaRadians,
      double vxMetersPerTick,
      double vyMetersPerTick,
      double omegaRadiansPerTick) {
    this(tick, xMeters, yMeters, thetaRadians, vxMetersPerTick, vyMetersPerTick, omegaRadiansPerTick, 1.0);
  }
}
