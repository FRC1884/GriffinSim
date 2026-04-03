package org.griffins1884.griffinsim.frc.scenario;

public record Waypoint1d(int tick, double xMeters, double pwmMagnitude) {
  public Waypoint1d {
    if (tick < 0) {
      throw new IllegalArgumentException("tick must be non-negative");
    }
    if (pwmMagnitude < 0.0) {
      throw new IllegalArgumentException("pwmMagnitude must be non-negative");
    }
  }

  public Waypoint1d(int tick, double xMeters) {
    this(tick, xMeters, 0.5);
  }
}
