package org.griffins1884.griffinsim.frc;

public record ControlHostConfig(
    ClockMode clockMode, long physicsStepNanos, long controlStepNanos, int queueCapacity) {
  public ControlHostConfig {
    if (clockMode == null) {
      throw new NullPointerException("clockMode");
    }
    if (physicsStepNanos <= 0 || controlStepNanos <= 0) {
      throw new IllegalArgumentException("step durations must be positive");
    }
    if (queueCapacity <= 0) {
      throw new IllegalArgumentException("queueCapacity must be positive");
    }
  }

  public static ControlHostConfig defaultLockstep() {
    return new ControlHostConfig(ClockMode.LOCKSTEP, 5_000_000L, 20_000_000L, 32);
  }
}
