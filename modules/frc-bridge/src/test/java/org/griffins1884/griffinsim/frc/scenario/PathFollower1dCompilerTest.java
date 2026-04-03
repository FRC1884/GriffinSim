package org.griffins1884.griffinsim.frc.scenario;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import org.junit.jupiter.api.Test;

class PathFollower1dCompilerTest {
  @Test
  void compilesControllerWaypointsIntoBoundedDeterministicTimeline() {
    PwmCommandTimeline timeline =
        PathFollower1dCompiler.compile(
            new PathFollower1dSpec(
                List.of(new Waypoint1d(0, 0.0, 0.5), new Waypoint1d(2, 1.0, 0.8), new Waypoint1d(4, -0.5, 0.4)),
                0.0,
                1.2,
                0.9,
                0.2,
                0.01));

    assertEquals(0.0, timeline.valueAtTick(0));
    assertTrue(timeline.valueAtTick(1) > 0.0);
    assertTrue(timeline.valueAtTick(3) < 0.0);
    assertEquals(0.0, timeline.valueAtTick(4));
  }
}
