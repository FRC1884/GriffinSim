package org.griffins1884.griffinsim.frc.scenario;

public record Waypoint2d(int tick, double xMeters, double yMeters, double pwmMagnitude) {
  public Waypoint2d {
    if (tick < 0) {
      throw new IllegalArgumentException("tick must be non-negative");
    }
    if (pwmMagnitude < 0.0) {
      throw new IllegalArgumentException("pwmMagnitude must be non-negative");
    }
  }

  public Waypoint2d(int tick, double xMeters, double yMeters) {
    this(tick, xMeters, yMeters, 0.5);
  }
}
