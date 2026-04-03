package org.griffins1884.griffinsim.frc.scenario;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.List;
import org.junit.jupiter.api.Test;

class ScenarioEndpointFactoryTest {
  @Test
  void pwmTimelineSelectsExpectedValuesPerTick() {
    PwmCommandTimeline timeline =
        new PwmCommandTimeline(
            List.of(
                new ScheduledPwmCommand(0, 1, 0.2),
                new ScheduledPwmCommand(2, 3, 0.8),
                new ScheduledPwmCommand(4, 5, -0.4)),
            0.0);

    assertEquals(0.2, timeline.valueAtTick(0));
    assertEquals(0.2, timeline.valueAtTick(1));
    assertEquals(0.8, timeline.valueAtTick(2));
    assertEquals(-0.4, timeline.valueAtTick(5));
    assertEquals(0.0, timeline.valueAtTick(6));
  }
}
