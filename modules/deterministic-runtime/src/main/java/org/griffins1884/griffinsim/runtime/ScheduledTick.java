package org.griffins1884.griffinsim.runtime;

import java.util.List;

public record ScheduledTick(int stepId, long simTimeNanos, List<Long> physicsStepTimesNanos) {
  public ScheduledTick {
    if (stepId < 0) {
      throw new IllegalArgumentException("stepId must be non-negative");
    }
    if (simTimeNanos < 0) {
      throw new IllegalArgumentException("simTimeNanos must be non-negative");
    }
    physicsStepTimesNanos = List.copyOf(physicsStepTimesNanos);
  }
}
