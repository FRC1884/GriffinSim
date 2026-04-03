package org.griffins1884.griffinsim.frc.scenario;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import org.junit.jupiter.api.Test;

class PathFollower2dCompilerTest {
  @Test
  void compiles2dControllerWaypointsIntoPlanarPwmPlan() {
    HolonomicPwmCommandPlan plan =
        PathFollower2dCompiler.compile(
            new PathFollower2dSpec(
                List.of(
                    new Waypoint2d(0, 0.0, 0.0, 0.2),
                    new Waypoint2d(1, 1.0, 0.5, 0.8),
                    new Waypoint2d(3, -0.5, -0.25, 0.4)),
                0.0,
                0.0,
                1.2,
                0.9,
                0.2,
                0.01));

    assertTrue(plan.xTimeline().valueAtTick(1) > 0.0);
    assertTrue(plan.yTimeline().valueAtTick(1) > 0.0);
    assertTrue(plan.xTimeline().valueAtTick(2) >= 0.0);
    assertTrue(plan.yTimeline().valueAtTick(2) >= 0.0);
    assertEquals(0.0, plan.xTimeline().valueAtTick(3));
    assertEquals(0.0, plan.yTimeline().valueAtTick(3));
    assertEquals(0.0, plan.xTimeline().valueAtTick(4));
    assertEquals(0.0, plan.yTimeline().valueAtTick(4));
  }
}
