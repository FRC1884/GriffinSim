package org.griffins1884.griffinsim.frc.scenario;

public record ScheduledPwmCommand(int startTick, int endTickInclusive, double value) {
  public ScheduledPwmCommand {
    if (startTick < 0) {
      throw new IllegalArgumentException("startTick must be non-negative");
    }
    if (endTickInclusive < startTick) {
      throw new IllegalArgumentException("endTickInclusive must be >= startTick");
    }
  }
}
