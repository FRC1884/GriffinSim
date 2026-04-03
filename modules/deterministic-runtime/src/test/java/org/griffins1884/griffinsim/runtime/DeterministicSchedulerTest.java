package org.griffins1884.griffinsim.runtime;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class DeterministicSchedulerTest {
  @Test
  void generatesFixedSubstepsForEachControlTick() {
    DeterministicScheduler scheduler = new DeterministicScheduler(5_000_000L, 20_000_000L);

    ScheduledTick first = scheduler.nextTick();
    ScheduledTick second = scheduler.nextTick();

    assertEquals(1, first.stepId());
    assertEquals(20_000_000L, first.simTimeNanos());
    assertEquals(4, first.physicsStepTimesNanos().size());
    assertEquals(5_000_000L, first.physicsStepTimesNanos().get(0));
    assertEquals(20_000_000L, first.physicsStepTimesNanos().get(3));
    assertEquals(2, second.stepId());
    assertEquals(40_000_000L, second.simTimeNanos());
  }
}
