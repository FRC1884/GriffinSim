package org.griffins1884.griffinsim.frc.scenario;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.List;
import org.junit.jupiter.api.Test;

class WaypointTimelineCompilerTest {
  @Test
  void compilesWaypointsIntoDirectionalPwmSegments() {
    PwmCommandTimeline timeline =
        WaypointTimelineCompiler.compile1d(
            List.of(
                new Waypoint1d(0, 0.0, 0.3),
                new Waypoint1d(2, 1.0, 0.6),
                new Waypoint1d(5, -0.5, 0.4)));

    assertEquals(0.6, timeline.valueAtTick(0));
    assertEquals(0.6, timeline.valueAtTick(1));
    assertEquals(-0.4, timeline.valueAtTick(2));
    assertEquals(-0.4, timeline.valueAtTick(4));
    assertEquals(0.0, timeline.valueAtTick(5));
  }
}
