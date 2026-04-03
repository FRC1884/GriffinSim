package org.griffins1884.griffinsim.runtime;

import java.util.ArrayList;
import java.util.List;

public final class DeterministicScheduler {
  private final long physicsStepNanos;
  private final long controlStepNanos;
  private final int physicsStepsPerControlTick;
  private int stepId;
  private long simTimeNanos;

  public DeterministicScheduler(long physicsStepNanos, long controlStepNanos) {
    if (physicsStepNanos <= 0 || controlStepNanos <= 0) {
      throw new IllegalArgumentException("step durations must be positive");
    }
    if (controlStepNanos % physicsStepNanos != 0) {
      throw new IllegalArgumentException("control step must be an integer multiple of physics step");
    }
    this.physicsStepNanos = physicsStepNanos;
    this.controlStepNanos = controlStepNanos;
    this.physicsStepsPerControlTick = (int) (controlStepNanos / physicsStepNanos);
  }

  public synchronized ScheduledTick nextTick() {
    List<Long> substeps = new ArrayList<>(physicsStepsPerControlTick);
    for (int i = 0; i < physicsStepsPerControlTick; i++) {
      simTimeNanos += physicsStepNanos;
      substeps.add(simTimeNanos);
    }
    stepId += 1;
    return new ScheduledTick(stepId, simTimeNanos, substeps);
  }

  public long physicsStepNanos() {
    return physicsStepNanos;
  }

  public long controlStepNanos() {
    return controlStepNanos;
  }

  public int physicsStepsPerControlTick() {
    return physicsStepsPerControlTick;
  }
}
